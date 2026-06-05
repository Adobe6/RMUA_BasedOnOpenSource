#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <time.h>
#include <stdlib.h>
#include <random>
#include <mutex>
#include "eskf.hpp"

std::random_device rd{};
std::mt19937 gen{rd()};
std::normal_distribution<double> gauss_dist{0.0, 1.0};
ErrorStateKalmanFilter* g_eskf_ptr;
int odo_cnt = 0;
ros::Publisher g_eskf_odom_puber;
ros::Publisher g_corrected_odom_puber;
ros::Publisher g_corrected_cloud_puber;

nav_msgs::Odometry g_latest_lio_odom;
bool g_have_lio_odom = false;
bool g_have_gps_anchor = false;
Eigen::Vector3d g_gps_to_odom_offset = Eigen::Vector3d::Zero();
Eigen::Vector3d g_latest_gps_pos_in_odom = Eigen::Vector3d::Zero();
Eigen::Vector3d g_lio_pos_at_latest_gps = Eigen::Vector3d::Zero();

void odom_local_ned_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void lio_odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void cloud_registered_cb(const sensor_msgs::PointCloud2::ConstPtr& msg);
void init_pose_ned_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
