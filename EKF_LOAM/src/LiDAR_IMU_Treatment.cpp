#include "settings_ekf_loam.h"

using namespace std;

//variables

Eigen::Matrix3d Ro;
Eigen::MatrixXd Ho(4,4), H1(4,4), H2(4,4), H(4,4);

std::string chassis_frame;

class LiDAR_IMU_Treatment{

private:
    // NodeHandler
    ros::NodeHandle nh;

    // time
    ros::Time Time_tf;

    // Subscribes
    ros::Subscriber subOdometry;
    ros::Subscriber subLaserCloudCornerLast;
    ros::Subscriber subLaserCloudSurfLast;
    ros::Subscriber subImu;

    //Publishers
    ros::Publisher pubOdometryOut;
    ros::Publisher pubLaserCloudCornerOut;
    ros::Publisher pubLaserCloudSurfOut;
    ros::Publisher pubImuOut;

    // Odometry
    nav_msgs::Odometry odometryOut;

    // String
    std::string childFrameId;
    std::string frameId;


    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    
public:

    LiDAR_IMU_Treatment():
        nh("~")
        {
            //Subscriber topics
            subOdometry = nh.subscribe<nav_msgs::Odometry>("/Odometry", 5, &LiDAR_IMU_Treatment::odometryHandler, this);
            subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/livox_less_sharp_cloud", 2, &LiDAR_IMU_Treatment::laserCloudCornerLastHandler, this);
            subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/livox_less_flat_cloud", 2, &LiDAR_IMU_Treatment::laserCloudSurfLastHandler, this);
            subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, &LiDAR_IMU_Treatment::imuHandler, this);

            //publisher topics  
            pubOdometryOut = nh.advertise<nav_msgs::Odometry> ("/Odometry_to_LiDARMapping", 5);
            pubLaserCloudCornerOut = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner", 2);
            pubLaserCloudSurfOut = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 2);
            pubImuOut = nh.advertise<sensor_msgs::Imu> ("/imu_to_LiDARMapping", 50);

        initialization();
        allocateMemory();
    }

    ~LiDAR_IMU_Treatment(){}

    void initialization(){
        // initialization
        H2 << 0, 1, 0, 0, 
              0, 0, 1, 0, 
              1, 0, 0, 0, 
              0, 0, 0, 1; // (z) forward (y) up
    }

    void allocateMemory(){
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg){
        Time_tf = odometryMsg->header.stamp;

        Eigen::Quaterniond q;
        q.x() = odometryMsg->pose.pose.orientation.x;
        q.y() = odometryMsg->pose.pose.orientation.y;
        q.z() = odometryMsg->pose.pose.orientation.z;
        q.w() = odometryMsg->pose.pose.orientation.w;
        Ro = q.toRotationMatrix();

        Ho.block(0,0,3,3) = Ro;
        Ho.block(0,3,3,1) << odometryMsg->pose.pose.position.x, odometryMsg->pose.pose.position.y, odometryMsg->pose.pose.position.z;
        Ho.block(3,0,1,4) << 0,0,0,1;

        H = H2*Ho*H2.inverse();
        childFrameId = "lidar_odom";
        frameId = "slam_init";

        // covariance
        odometryOut.header.stamp = ros::Time::now();
        odometryOut.pose.covariance = odometryMsg->pose.covariance; 
        

        Eigen::Matrix3d R = H.block(0,0,3,3);
        Eigen::Quaterniond q_out(R);

        odometryOut.header.frame_id = frameId;
        odometryOut.child_frame_id = childFrameId;
        
        odometryOut.pose.pose.orientation.x = q_out.x();
        odometryOut.pose.pose.orientation.y = q_out.y();
        odometryOut.pose.pose.orientation.z = q_out.z();
        odometryOut.pose.pose.orientation.w = q_out.w();
        odometryOut.pose.pose.position.x = H(0,3); 
        odometryOut.pose.pose.position.y = H(1,3); 
        odometryOut.pose.pose.position.z = H(2,3); 

        // velocities:
        odometryOut.twist.twist.linear = odometryMsg->twist.twist.linear;
        odometryOut.twist.twist.angular = odometryMsg->twist.twist.angular;
        
        // publish
        pubOdometryOut.publish(odometryOut);
    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg(*msg, *laserCloudCornerLast);
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        pubLaserCloudCornerOut.publish(laserCloudTemp);
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        pcl::fromROSMsg(*msg, *laserCloudSurfLast);
        sensor_msgs::PointCloud2 laserCloudTemp;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        pubLaserCloudSurfOut.publish(laserCloudTemp);
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn){
        sensor_msgs::Imu ImuTempOut = *imuIn;
        ImuTempOut.header.stamp = ros::Time::now();
        pubImuOut.publish(ImuTempOut);
    }
};

int main (int argc, char **argv)
{
    // node init
    ros::init(argc, argv, "LiDAR_IMU_Treatment");
    
    ROS_INFO("\033[1;32m---->\033[0m LiDAR & IMU Treatment Started.");

    ros::NodeHandle nh_; 

    LiDAR_IMU_Treatment SC3D;

    ros::spin();
    return 0;
}