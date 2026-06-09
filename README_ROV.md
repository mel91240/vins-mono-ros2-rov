# VINS-Mono ROS 2 — usage ROV

Adaptation de VINS-Mono (ROS 2 Humble) pour le SLAM visuel-inertiel d'un ROV.
Ce guide couvre le **lancement** et le **changement de dataset**. Pour le détail
des packages, voir `README.md`.

## Build

```bash
cd ~/rov_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
> Astuce : ajouter `--symlink-install` évite de rebuilder à chaque édition d'un
> fichier launch/config.

## Lancement (une seule commande)

Le bringup démarre les 3 nœuds (`feature_tracker` + `vins_estimator` +
`pose_graph`) **avec les bons namespaces** + RViz :

```bash
# Terminal 1 : la pile VINS
ros2 launch vins_estimator rov_bringup.launch.py

# Terminal 2 : le dataset (rejeu)
ros2 bag play <chemin_du_bag> --clock
```

⚠️ **Important** : il FAUT passer par ce launch (ou par les flags
`-r __ns:=/vins_estimator` et `-r __ns:=/pose_graph`). Lancer les nœuds avec un
simple `ros2 run` sans namespace casse silencieusement la **fermeture de boucle**
(le `pose_graph` ne reçoit plus les keyframes → `vins_result_loop.csv` vide).

## Changer de dataset

Le dataset est défini par le **fichier de config** passé en argument
`config_file:=` (chemin **absolu**). Par défaut : `mclab_1`.

| Dataset | `config_file:=` | Topics attendus dans le bag |
|---|---|---|
| **mclab** (défaut) | `…/config_pkg/config/mclab/mclab_1_config.yaml` | `/alphasense_driver_ros/cam0` + `/alphasense_driver_ros/imu` |
| **EuRoC** | `…/config_pkg/config/euroc/euroc_config.yaml` | `/cam0/image_raw` + `/imu0` |

(`…` = `/home/melanie/rov_ws/src/VINS-MONO-ROS2`)

Exemple EuRoC :
```bash
ros2 launch vins_estimator rov_bringup.launch.py \
  config_file:=/home/melanie/rov_ws/src/VINS-MONO-ROS2/config_pkg/config/euroc/euroc_config.yaml
```
> Le `config_file` doit pointer un fichier **existant**, sinon : `ERROR: Wrong
> path to settings` puis crash (`Invalid topic name`). Les topics du config
> doivent correspondre à ceux du bag joué.

## Ajouter un nouveau dataset

1. Créer `config_pkg/config/<mon_dataset>/<mon_dataset>_config.yaml`
   (copier un config existant comme base).
2. Régler dans ce fichier : `imu_topic`, `image_topic`, le modèle caméra
   (`model_type`, intrinsèques/distorsion), les paramètres de bruit IMU, et
   `estimate_extrinsic`/`estimate_td` au besoin.
3. Lancer avec `config_file:=…/config/<mon_dataset>/<mon_dataset>_config.yaml`.
4. Si le config est nouveau dans `config_pkg`, refaire un `colcon build` (sauf en
   `--symlink-install`).

## Options du launch

| Argument | Défaut | Rôle |
|---|---|---|
| `config_file` | mclab_1 | fichier de config/calibration (chemin absolu) |
| `use_sim_time` | `true` | `true` en rejeu de bag (`--clock`), `false` en **live** (webcam+IMU) |
| `rviz` | `true` | lancer RViz (`rviz:=false` pour s'en passer) |

## Topics utiles dans RViz

| Affichage | Topic | Type |
|---|---|---|
| Trajectoire **avant** loop (VIO brut) | `/vins_estimator/path` | Path |
| Trajectoire **après** loop (optimisée) | `/pose_graph/pose_graph_path` | Path |
| Nuage courant | `/vins_estimator/point_cloud` | PointCloud |
| Carte accumulée | `/vins_estimator/history_cloud` | PointCloud |
| Arêtes de fermeture de boucle | `/pose_graph/pose_graph` | MarkerArray |

> `/pose_graph/no_loop_path` n'est publié **que** si `loop_closure: 0`.
> `/pose_graph/match_image` n'apparaît **que** lorsqu'une boucle est détectée.
