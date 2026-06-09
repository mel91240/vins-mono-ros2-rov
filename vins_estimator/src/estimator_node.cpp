#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::msg::Imu::SharedPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::SharedPtr> feature_buf;
queue<sensor_msgs::msg::PointCloud::SharedPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = 0;

void predict(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::msg::Imu::SharedPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::msg::Imu::SharedPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!((imu_buf.back()->header.stamp.sec+imu_buf.back()->header.stamp.nanosec * (1e-9)) > (feature_buf.front()->header.stamp.sec+feature_buf.front()->header.stamp.nanosec * (1e-9)) + estimator.td))
        {
            //RCUTILS_LOG_WARN("wait for imu, only should happen at the beginning");
            sum_of_wait++;
            return measurements;
        }

        if (!((imu_buf.front()->header.stamp.sec+imu_buf.front()->header.stamp.nanosec * (1e-9)) < (feature_buf.front()->header.stamp.sec+feature_buf.front()->header.stamp.nanosec * (1e-9)) + estimator.td))
        {
            RCUTILS_LOG_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::msg::PointCloud::SharedPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::msg::Imu::SharedPtr> IMUs;
        while ((imu_buf.front()->header.stamp.sec+imu_buf.front()->header.stamp.nanosec * (1e-9)) < (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9)) + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            RCUTILS_LOG_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    
    // Detect bag restart
    if (t < last_imu_t - 1.0)
    {
        RCUTILS_LOG_WARN("imu jump back in time! t: %f, last: %f", t, last_imu_t);
        last_imu_t = t;
    }

    if (t < last_imu_t)
    {
        RCUTILS_LOG_WARN("imu message in disorder!");
        return;
    }

    last_imu_t = t;

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::msg::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}


void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        RCUTILS_LOG_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

void relocalization_callback(const sensor_msgs::msg::PointCloud::SharedPtr points_msg)
{
    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}

// thread: visual-inertial odometry
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, sensor_msgs::msg::PointCloud::SharedPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });

        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.sec+imu_msg->header.stamp.nanosec * (1e-9);
                double img_t = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9) + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    assert(dt >= 0);
                    current_time = t;
                    estimator.processIMU(dt, Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
                                            Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
                    //                                            imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    assert(dt_1 >= 0);
                    assert(dt_2 >= 0);
                    assert(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    Vector3d acc = w1 * Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z) +
                                    w2 * Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                    Vector3d gyr = w1 * Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z) +
                                    w2 * Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                    estimator.processIMU(dt_1, acc, gyr);
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, acc.x(), acc.y(), acc.z(), gyr.x(), gyr.y(), gyr.z());
                }
            }
            //printf("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                assert(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::msg::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            if (relo_buf.size() != 0)
            {
                pubRelocalization(estimator);
            }
            //RCUTILS_LOG_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("vins_estimator");
    readParameters(n);
    estimator.setParameter();
#ifdef EIGEN_DONT_PARALLELIZE
    RCUTILS_LOG_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    RCUTILS_LOG_WARN("waiting for image and imu...");

    registerPub(n);

    auto sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)), imu_callback);
    auto sub_image = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/feature_tracker/restart", rclcpp::QoS(rclcpp::KeepLast(2000)), restart_callback);
    auto sub_relo_points = n->create_subscription<sensor_msgs::msg::PointCloud>("/pose_graph/match_points", rclcpp::QoS(rclcpp::KeepLast(2000)), relocalization_callback);

    std::thread measurement_process{process};
    rclcpp::spin(n);

    return 0;
}
