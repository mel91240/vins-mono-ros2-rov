#!/usr/bin/env python3
"""Carte de COUVERTURE temps reel pour guider le pilote du ROV.

Objectif (la mission) : montrer EN DIRECT les zones deja filmees, pour que le
pilote au joystick soit sur de tout couvrir -- sans trou.

Principe (CPU pur, AUCUN CUDA -- tourne sur le portable) :
  - VINS-Mono triangule des points 3D de ce que la camera VOIT REELLEMENT
    (topics point_cloud = fenetre courante, history_cloud = points marginalises
    persistants). Un point triangule = une zone effectivement filmee.
  - On voxelise ces points dans une grille 3D (resolution reglable). Chaque
    voxel touche passe "couvert" (vert). Les trous = pas encore filme.
  - On dessine aussi le FRUSTUM de la camera a sa pose courante (camera_pose),
    pour que le pilote voie ou il pointe MAINTENANT.

Pourquoi se baser sur les points VINS et pas sur une projection du frustum au
sol : projeter le frustum exige de connaitre la distance a la surface (plan du
fond, ou paroi) -- inconnue sans capteur. Les points que VINS a deja triangules
DONNENT cette geometrie gratuitement et honnetement (on n'affiche couvert que
ce qui est vraiment vu). Amelioration v2 possible : remplir le footprint entre
la camera et les points (couverture plus "pleine" sur surfaces peu texturees).

Sortie : MarkerArray sur /coverage/markers (a afficher dans RViz, frame "world").

Usage :
  python3 coverage_map.py --ros-args \
      -p resolution:=0.3 -p hfov_deg:=110.0 -p vfov_deg:=90.0 \
      -p frustum_range:=3.0 -p publish_hz:=2.0
  (topics par defaut : /vins_estimator/{point_cloud,history_cloud,camera_pose})
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion -> matrice de rotation 3x3 (lignes), sans dependance tf."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]


def mat_vec(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


class CoverageMap(Node):
    def __init__(self):
        super().__init__("coverage_map")
        self.declare_parameter("resolution", 0.3)        # taille tuile/voxel (m)
        self.declare_parameter("mode", "2d")             # "2d" = tapis vu de dessus, "3d" = cubes
        self.declare_parameter("hfov_deg", 110.0)        # FOV horizontal camera ROV
        self.declare_parameter("vfov_deg", 90.0)         # FOV vertical (approx)
        self.declare_parameter("frustum_range", 3.0)     # longueur du frustum dessine (m)
        self.declare_parameter("publish_hz", 2.0)        # cadence d'affichage
        self.declare_parameter("max_cubes", 50000)       # garde-fou perf RViz
        self.declare_parameter("pc_topic", "/vins_estimator/point_cloud")
        self.declare_parameter("hist_topic", "/vins_estimator/history_cloud")
        self.declare_parameter("pose_topic", "/vins_estimator/camera_pose")

        self.res = self.get_parameter("resolution").value
        self.mode = self.get_parameter("mode").value
        self.hfov = math.radians(self.get_parameter("hfov_deg").value)
        self.vfov = math.radians(self.get_parameter("vfov_deg").value)
        self.frange = self.get_parameter("frustum_range").value
        self.max_cubes = int(self.get_parameter("max_cubes").value)

        self.cells = {}          # 2d: (ix,iy)->hits ; 3d: (ix,iy,iz)->hits
        self.ground_z = None     # z le plus bas vu -> niveau du tapis 2d
        self.cam_pose = None     # (position[3], R[3][3])
        self.capped_warned = False

        self.create_subscription(
            PointCloud, self.get_parameter("pc_topic").value, self.on_cloud, 10)
        self.create_subscription(
            PointCloud, self.get_parameter("hist_topic").value, self.on_cloud, 10)
        self.create_subscription(
            Odometry, self.get_parameter("pose_topic").value, self.on_pose, 10)

        self.pub = self.create_publisher(MarkerArray, "/coverage/markers", 1)
        hz = self.get_parameter("publish_hz").value
        self.create_timer(1.0 / hz, self.publish_markers)
        self.get_logger().info(
            f"coverage_map : res={self.res} m, FOV={math.degrees(self.hfov):.0f}x"
            f"{math.degrees(self.vfov):.0f} deg -> /coverage/markers (frame world)")

    def on_cloud(self, msg):
        r = self.res
        if self.mode == "2d":
            for p in msg.points:
                if self.ground_z is None or p.z < self.ground_z:
                    self.ground_z = p.z
                key = (math.floor(p.x / r), math.floor(p.y / r))
                self.cells[key] = self.cells.get(key, 0) + 1
        else:
            for p in msg.points:
                key = (math.floor(p.x / r), math.floor(p.y / r), math.floor(p.z / r))
                self.cells[key] = self.cells.get(key, 0) + 1

    def on_pose(self, msg):
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        self.cam_pose = ([t.x, t.y, t.z], quat_to_rot(q.x, q.y, q.z, q.w))

    def publish_markers(self):
        arr = MarkerArray()
        arr.markers.append(self.build_cubes())
        f = self.build_frustum()
        if f is not None:
            arr.markers.append(f)
        self.pub.publish(arr)

    def build_cubes(self):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "coverage"
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        r = self.res
        if self.mode == "2d":
            # tuiles PLATES posees au sol -> tapis "vu de dessus", lisible
            m.scale.x = m.scale.y = r * 0.95
            m.scale.z = 0.02
        else:
            m.scale.x = m.scale.y = m.scale.z = r * 0.95
        m.pose.orientation.w = 1.0

        z0 = self.ground_z if self.ground_z is not None else 0.0
        items = self.cells
        if len(items) > self.max_cubes:
            if not self.capped_warned:
                self.get_logger().warn(
                    f"{len(items)} cellules couvertes > max_cubes={self.max_cubes} : "
                    f"affichage tronque (augmente 'resolution' ou 'max_cubes').")
                self.capped_warned = True
            items = dict(list(items.items())[: self.max_cubes])

        for key, hits in items.items():
            pt = Point()
            pt.x = (key[0] + 0.5) * r
            pt.y = (key[1] + 0.5) * r
            pt.z = z0 if self.mode == "2d" else (key[2] + 0.5) * r
            m.points.append(pt)
            # plus c'est filme, plus c'est opaque (vert uni = couvert)
            a = min(0.9, 0.35 + 0.1 * hits)
            m.colors.append(ColorRGBA(r=0.1, g=0.85, b=0.2, a=a))
        return m

    def build_frustum(self):
        if self.cam_pose is None:
            return None
        C, R = self.cam_pose
        # axe optique camera = +Z ; coins du frustum a la distance 'frange'
        tx = math.tan(self.hfov / 2.0)
        ty = math.tan(self.vfov / 2.0)
        corners_cam = [
            [tx, ty, 1.0], [-tx, ty, 1.0], [-tx, -ty, 1.0], [tx, -ty, 1.0],
        ]
        corners_w = []
        for c in corners_cam:
            d = mat_vec(R, c)
            corners_w.append([C[0] + d[0] * self.frange,
                              C[1] + d[1] * self.frange,
                              C[2] + d[2] * self.frange])

        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "frustum"
        m.id = 1
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color = ColorRGBA(r=0.1, g=0.6, b=1.0, a=0.9)
        m.pose.orientation.w = 1.0

        def add(a, b):
            pa, pb = Point(), Point()
            pa.x, pa.y, pa.z = a[0], a[1], a[2]
            pb.x, pb.y, pb.z = b[0], b[1], b[2]
            m.points.append(pa)
            m.points.append(pb)

        for cw in corners_w:        # apex -> coins
            add(C, cw)
        for i in range(4):          # rectangle au loin
            add(corners_w[i], corners_w[(i + 1) % 4])
        return m


def main():
    rclpy.init()
    node = CoverageMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
