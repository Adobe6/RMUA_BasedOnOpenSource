#include "imu_gps_odometry.hpp"


int main(int argc, char** argv)
{
    //(重力， P_位置不确定度_std, P_速度不确定度_std, P_角度不确定度_std, P_角速度bias不确定度_std, P_加速度bias不确定度_std,
    //gps位置测量噪声_std gpsz姿态测量噪声_std, imu角速度测量噪声_std, imu加速度测量噪声_std)
    g_eskf_ptr = new ErrorStateKalmanFilter(-9.81083, 0.1, 0.01, 0.1, 0.0003158085227, 0.001117221, 0.001, 100.0, 0.00143, 0.0386);
    ros::init(argc, argv, "odometry"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    g_eskf_odom_puber = n.advertise<geometry_msgs::PoseStamped>("/eskf_odom", 1);
    g_corrected_odom_puber = n.advertise<nav_msgs::Odometry>("/gps_corrected_odometry", 20);
    g_corrected_cloud_puber = n.advertise<sensor_msgs::PointCloud2>("/cloud_registered_corrected", 10);
    ros::Subscriber odom_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, odom_local_ned_cb);
    ros::Subscriber lio_odom_suber = n.subscribe<nav_msgs::Odometry>("/Odometry", 20, lio_odom_cb);
    ros::Subscriber cloud_suber = n.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 5, cloud_registered_cb);
    ros::Subscriber imu_suber = n.subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, imu_cb);//imu数据
    ros::Subscriber init_pose_suber = n.subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, init_pose_ned_cb);
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete g_eskf_ptr;
    return 0;
}

void init_pose_ned_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!g_eskf_ptr->m_isInitailed)
    {
        Eigen::Quaternion tmpq(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Matrix4d r0 = Eigen::Matrix4d::Identity();
        r0.block<3,3>(0, 0) = tmpq.toRotationMatrix();
        r0.block<3,1>(0, 3) << msg->pose.position.x, msg->pose.position.y,msg->pose.position.z;
        g_eskf_ptr->Init(r0, Eigen::Vector3d::Zero(),msg->header.stamp.toNSec());
        g_eskf_ptr->m_isInitailed = true;
    }
}

static Eigen::Vector3d gpsPoseToOdomAxes(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    return Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.y, -msg->pose.position.z);
}

static Eigen::Vector3d odomPosition(const nav_msgs::Odometry& msg)
{
    return Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}

static Eigen::Vector3d correctedFrameTranslation()
{
    return g_latest_gps_pos_in_odom - g_lio_pos_at_latest_gps;
}

static void publishCorrectedOdom()
{
    if (!g_have_lio_odom || !g_have_gps_anchor)
        return;

    nav_msgs::Odometry corrected = g_latest_lio_odom;
    const Eigen::Vector3d lio_pos = odomPosition(g_latest_lio_odom);
    const Eigen::Vector3d corrected_pos = lio_pos + correctedFrameTranslation();

    corrected.header.frame_id = g_latest_lio_odom.header.frame_id.empty() ? "odom" : g_latest_lio_odom.header.frame_id;
    corrected.pose.pose.position.x = corrected_pos.x();
    corrected.pose.pose.position.y = corrected_pos.y();
    corrected.pose.pose.position.z = corrected_pos.z();

    g_corrected_odom_puber.publish(corrected);
}

void lio_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    g_latest_lio_odom = *msg;
    g_have_lio_odom = true;
    publishCorrectedOdom();
}

void cloud_registered_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!g_have_gps_anchor)
        return;

    sensor_msgs::PointCloud2 corrected = *msg;
    corrected.header.frame_id = g_latest_lio_odom.header.frame_id.empty() ? "odom" : g_latest_lio_odom.header.frame_id;

    const Eigen::Vector3d translation = correctedFrameTranslation();
    sensor_msgs::PointCloud2Iterator<float> iter_x(corrected, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(corrected, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(corrected, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        *iter_x += static_cast<float>(translation.x());
        *iter_y += static_cast<float>(translation.y());
        *iter_z += static_cast<float>(translation.z());
    }

    g_corrected_cloud_puber.publish(corrected);
}

void odom_local_ned_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // ROS_INFO("Get odom_local_ned_cd\n  orientation: %f-%f-%f-%f\n  position: %f-%f-%f\n", 
    // msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, //姿态四元数
    // msg->pose.position.x, msg->pose.position.y,msg->pose.position.z);
    odo_cnt ++;
    g_eskf_ptr->correct(Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y,msg->pose.position.z), 
        Eigen::Quaterniond(msg->pose.orientation.w,msg->pose.orientation.x, msg->pose.orientation.y,msg->pose.orientation.z));

    if (!g_have_lio_odom)
        return;

    const Eigen::Vector3d gps_pos = gpsPoseToOdomAxes(msg);
    const Eigen::Vector3d lio_pos = odomPosition(g_latest_lio_odom);
    if (!g_have_gps_anchor)
    {
        g_gps_to_odom_offset = lio_pos - gps_pos;
        g_have_gps_anchor = true;
        ROS_INFO_STREAM("[imu_gps_odometry] GPS odom anchor initialized, offset: "
                        << g_gps_to_odom_offset.transpose());
    }

    g_latest_gps_pos_in_odom = gps_pos + g_gps_to_odom_offset;
    g_lio_pos_at_latest_gps = lio_pos;
    publishCorrectedOdom();
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("Get imu data.\n %f %f %f \n %f %f %f", msg->angular_velocity.x, msg->angular_velocity.y,
    // msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    if(g_eskf_ptr->m_isInitailed)
    {
        Eigen::Vector3d pos, vel, angle_vel;
        Eigen::Quaterniond q;
        g_eskf_ptr->Predict(Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z), 
            Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), 
            pos, vel, angle_vel, q, msg->header.stamp.toNSec());
        geometry_msgs::PoseStamped msg2;
        msg2.header.stamp = msg->header.stamp;
        msg2.pose.position.x = pos.x();
        msg2.pose.position.y = pos.y();
        msg2.pose.position.z = pos.z();
        msg2.pose.orientation.w = q.w();
        msg2.pose.orientation.x = q.x();
        msg2.pose.orientation.y = q.y();
        msg2.pose.orientation.z = q.z();
        // msg2.twist.twist.linear.x = vel.x();
        // msg2.twist.twist.linear.y = vel.y();
        // msg2.twist.twist.linear.z = vel.z();
        // msg2.twist.twist.angular.x = angle_vel.x();
        // msg2.twist.twist.angular.y = angle_vel.y();
        // msg2.twist.twist.angular.z = angle_vel.z();
        g_eskf_odom_puber.publish(msg2);
    }
}
