/**

Reads RAW and DMP Data from an Arduino MPU6050 inertial measurement unit,
processes it and publishs imu msgs to ROS

Author: Marcel Stuettgen
email: stuettgen@fh-aachen.de
Aachen, 2014
*/

#ifndef __CRIUS_NODE_H__
#define __CRIUS_NODE_H__


#include <math.h>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>



class CriusNode
{
public:
    CriusNode();
    ~CriusNode();

private:

    //Nodehandle
    ros::NodeHandle n_;

    //Publishers and Subscribers
    ros::Subscriber sub_float_;       //subscriber to MPU6050 float data
    ros::Subscriber sub_int_;         //subscriber to MPU6050 integer data
    ros::Publisher imu_pub_;          //imu message publisher
    ros::Publisher imu_raw_pub_;      //imu message publisher
    ros::Publisher imu_free_pub_;     //imu message publisher
    ros::Publisher mag_pub_;          //magnetometer publisher
    tf::TransformBroadcaster tf_bc_;  //transform broadcaster
    tf::TransformListener tf_l_;      //transform listener

    ros::Publisher untransformed_imu_pub_;
    ros::Publisher transformed_imu_pub_;

    //ROS parameters
    std::string imu_frame_id_;        //name of imu frame
    std::string imu_base_frame_id_;   //name of imu base frame
    std::string base_frame_id_;       //name of base frame
    std::string float_data_topic_;    //topic of MPU6050 float data
    std::string int_data_topic_;      //topic of MPU6050 integer data
    std::string imu_msg_topic_;       //topic of ROS IMU data message
    std::string imu_free_msg_topic_;  //topic of ROS IMU data message
    std::string imu_raw_msg_topic_;   //topic of ROS IMU RAW data message
    std::string imu_mag_msg_topic_;   //topic of ROS IMU magnetic field message
    bool publish_tf_;                 //wether to publish a TF to ROS or not

    //subscriber callbacks for publishers
    void _onFloatDataReceived( const std_msgs::Float32MultiArray& floatData );
    void _onIntDataReceived( const std_msgs::Int16MultiArray& intData );

    //internal data storage
    tf::Quaternion q_;
    tf::Quaternion q_angle_correction;
    tf::Vector3 acceleration_;
    tf::Vector3 freeAcceleration_;
    tf::Vector3 worldAcceleration_;
    tf::Vector3 gyro_;
    tf::Vector3 magnetometer_;
    tf::Vector3 attitude_; //yaw pitch roll
    tf::Vector3 raw_acceleration_;
    tf::Vector3 raw_gyro_;
    tf::Vector3 gravity_;

    double qw_offset;
    double qx_offset;
    double qy_offset;
    double qz_offset;

};

#endif // __CRIUS_NODE_H__
