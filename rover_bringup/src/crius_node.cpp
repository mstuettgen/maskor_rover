#include <rover_bringup/CriusNode.h>
#include <sensor_msgs/Imu.h>
#include <boost/array.hpp>

const static double RAD2DEG = 180.0/M_PI;

//content of float array (see MPU6050 firmware)
enum floatArrayContent {
    quaternion_x,             // DMP quaternion
    quaternion_y,
    quaternion_z,
    quaternion_w,
    //accel_x,        // DMP acceleration
    //accel_y,
    //accel_z,
    //freeaccel_x,	// DMP free acceleration
    //freeaccel_y,
    //freeaccel_z,
    //worldaccel_x,	// DMP world frame acceleration
    //worldaccel_y,
    //worldaccel_z,
    //gyro_x,         // DMP gyro
    //gyro_y,
    //gyro_z,
    //mag_x,          // DMP magnetometer
    //mag_y,
    //mag_z,
    //yaw,            // DMP yaw pitch roll
    //pitch,
    //roll,
    //gravity_x,      // DMP gravity orientation
    //gravity_y,
    //gravity_z,
    N_FLOATS        // array size
};

//content of integer array (see MP0U605 firmware)
enum intArrayContent {
    raw_accel_x,	// RAW acceleration
    raw_accel_y,
    raw_accel_z,
    raw_gyro_x,     // RAW gyro
    raw_gyro_y,
    raw_gyro_z,
    raw_mag_x,          // magnetometer readings
    raw_mag_y,
    raw_mag_z,
    N_INTS		    // array size
};


CriusNode::CriusNode()
{

    //get params from launchfile
    n_.param<std::string>("/MPU6050_node/imu_frame_id", imu_frame_id_, "imu_link");
    n_.param<std::string>("/MPU6050_node/imu_base_frame_id", imu_base_frame_id_, "imu_base_link");
    n_.param<std::string>("/MPU6050_node/base_frame_id", base_frame_id_, "base_link");
    n_.param<std::string>("/MPU6050_node/float_data_topic", float_data_topic_, "/mpu6050/floatdata");
    n_.param<std::string>("/MPU6050_node/int_data_topic", int_data_topic_, "/mpu6050/intdata");
    n_.param<std::string>("/MPU6050_node/imu_msg_topic", imu_msg_topic_, "/imu/data");
    n_.param<std::string>("/MPU6050_node/imu_raw_msg_topic", imu_raw_msg_topic_, "/imu/data_raw");
    n_.param<std::string>("/MPU6050_node/imu_free_msg_topic", imu_free_msg_topic_, "/imu/data_free");
    n_.param<std::string>("/MPU6050_node/imu_mag_msg_topic", imu_mag_msg_topic_, "/imu/magnetometer_raw");
    n_.param<bool>("/MPU6050_node/publish_tf", publish_tf_ , false);

    //init publishers and subcribers
    sub_float_  = n_.subscribe(float_data_topic_.c_str(), 1, &CriusNode::_onFloatDataReceived, this);
    sub_int_    = n_.subscribe(int_data_topic_.c_str(), 1, &CriusNode::_onIntDataReceived, this);
    imu_pub_    = n_.advertise<sensor_msgs::Imu>(imu_msg_topic_.c_str(),10);
    imu_free_pub_= n_.advertise<sensor_msgs::Imu>(imu_free_msg_topic_.c_str(),10);


    //imu_raw_pub_= n_.advertise<sensor_msgs::Imu>(imu_raw_msg_topic_.c_str(),1000);
    //mag_pub_    = n_.advertise<sensor_msgs::MagneticField>(imu_mag_msg_topic_.c_str(),1000);

    //untransformed_imu_pub_ = n_.advertise<sensor_msgs::Imu>("/imu/untransformed",1000);
    //transformed_imu_pub_ = n_.advertise<sensor_msgs::Imu>("/imu/transformed",1000);
}

CriusNode::~CriusNode()
{

}


//read integer data from MPU6050
void CriusNode::_onIntDataReceived( const std_msgs::Int16MultiArray &rawData )
{
    // rawacceleration
    int raw_ax = rawData.data[raw_accel_x];
    int raw_ay = rawData.data[raw_accel_y];
    int raw_az = rawData.data[raw_accel_z];
    raw_acceleration_ = tf::Vector3(raw_ax,raw_ay,raw_az);

    //gyro
    int raw_gx = rawData.data[raw_gyro_x];
    int raw_gy = rawData.data[raw_gyro_y];
    int raw_gz = rawData.data[raw_gyro_z];
    raw_gyro_ = tf::Vector3(raw_gx,raw_gy,raw_gz);

    //magnetometer
    int mag_x = rawData.data[raw_mag_x];
    int mag_y = rawData.data[raw_mag_y];
    int mag_z = rawData.data[raw_mag_z];
    magnetometer_ = tf::Vector3(mag_x, mag_y, mag_z);
}



//read float data from MPU6050
void CriusNode::_onFloatDataReceived( const std_msgs::Float32MultiArray& floatData )
{
    /// ** Read message data **

    /// because it is not clear how the interal DMP of the MPU6050 calculates the stuff like acceleration,
    /// freeacceleration, etc... so we just take the quaternion and do the raw data conversions ourselves

    //get quaternion
    q_.setW(floatData.data[quaternion_w]);
    q_.setX(floatData.data[quaternion_x]);
    q_.setY(floatData.data[quaternion_y]);
    q_.setZ(floatData.data[quaternion_z]);

    q_.normalize();

    //IMU TF Transformation
    //convert quaternion to stamped quaternion (required for transformation)
    tf::Stamped<tf::Quaternion> q_in, q_transformed;
    q_in.frame_id_ = imu_frame_id_;
    q_in.setData(q_);

    //convert acceleration to stamped acceleration (required for transformation)
    tf::Stamped<tf::Vector3> accel_in, accel_transformed;
    accel_in.frame_id_ = imu_frame_id_;
    accel_in.setData(raw_acceleration_);

    //convert angular velocities to stamped  angular velocities (required for transformation)
    tf::Stamped<tf::Vector3> gyro_in, gyro_transformed;
    gyro_in.frame_id_ = imu_frame_id_;
    gyro_in.setData(raw_gyro_);

//    //transform test
//    tf::Stamped<tf::Vector3> untransformed, transformed;
//    untransformed.frame_id_ = imu_frame_id_;
//    untransformed.setData(tf::Vector3(5,1,-9.81));
//    tf_l_.transformVector(imu_base_frame_id_, untransformed, transformed);
//    ROS_INFO("x: %f, y: %f, z: %f", untransformed.x(), untransformed.y(), untransformed.z());
//    ROS_INFO("x: %f, y: %f, z: %f", transformed.x(), transformed.y(), transformed.z());
//    ROS_INFO("*****************************************");

    //apply IMU TF to IMU data (imu_frame -> imu_base_frame)
    tf_l_.transformQuaternion(imu_base_frame_id_, q_in , q_transformed);
    tf_l_.transformVector(imu_base_frame_id_, accel_in, accel_transformed);
    tf_l_.transformVector(imu_base_frame_id_, gyro_in, gyro_transformed);

    q_ = q_transformed;
    q_.normalize();
    raw_acceleration_ = accel_transformed;
    raw_gyro_ = gyro_transformed;

    //# get expected direction of gravity from quaternion
    float q[4] = {q_.getW(), q_.getX(), q_.getY(), q_.getZ()};
    float g[3];
    g[0] = 2 * (q[1] * q[3] - q[0] * q[2]); //x
    g[1] = 2 * (q[0] * q[1] + q[2] * q[3]); //y
    g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]; //z


    //hardcoded gravity correction angles (measured!)
    tf::Matrix3x3 gravity_correction_rotate;
    gravity_correction_rotate.setRPY(0.013 , -0.035 , -0.064);

    tf::Vector3 gravity_corrected = tf::Vector3(g[0],g[1],g[2]) * gravity_correction_rotate;

    //ROS_INFO("gx: %f, gy: %f, gz: %f (g)", gravity_corrected.x(), gravity_corrected.y(), gravity_corrected.z());

    //TODO: calculate covariances somehow, maybe anyone knows how to do it?
    boost::array<double,9> identity_covariance_matrix = { 1.0, 0.0, 0.0,
                                                          0.0, 1.0, 0.0,
                                                          0.0, 0.0, 1.0 };
 /*
    //publish ROS IMU RAW message
    sensor_msgs::Imu imurawmsg;
    imurawmsg.header.stamp = ros::Time::now();
    imurawmsg.header.frame_id = imu_frame_id_.c_str();
    imurawmsg.orientation.x = q_.x();
    imurawmsg.orientation.y = q_.y();
    imurawmsg.orientation.z = q_.z();
    imurawmsg.orientation.w = q_.w();
    imurawmsg.orientation_covariance = identity_covariance_matrix;
    imurawmsg.angular_velocity.x = raw_gyro_.x();
    imurawmsg.angular_velocity.y = raw_gyro_.y();
    imurawmsg.angular_velocity.z = raw_gyro_.z();
    imurawmsg.angular_velocity_covariance = identity_covariance_matrix;
    imurawmsg.linear_acceleration.x = raw_acceleration_.x();
    imurawmsg.linear_acceleration.y = raw_acceleration_.y();
    imurawmsg.linear_acceleration.z = raw_acceleration_.z();
    imurawmsg.linear_acceleration_covariance = identity_covariance_matrix;
    imu_raw_pub_.publish(imurawmsg);
*/


    //publish ROS IMU message
    sensor_msgs::Imu imumsg;
    imumsg.header.stamp = ros::Time::now();
    imumsg.header.frame_id = imu_frame_id_.c_str();
    imumsg.orientation.x = q_.x();
    imumsg.orientation.y = q_.y();
    imumsg.orientation.z = q_.z();
    imumsg.orientation.w = q_.w();
    imumsg.orientation_covariance = identity_covariance_matrix;
    imumsg.angular_velocity.x = raw_gyro_.x() / 16.4f;  //16.4 LSB = 1 rad/sec
    imumsg.angular_velocity.y = raw_gyro_.y() / 16.4f;
    imumsg.angular_velocity.z = raw_gyro_.z() / 16.4f;
    imumsg.angular_velocity_covariance = identity_covariance_matrix;
    imumsg.linear_acceleration.x = raw_acceleration_.x() / 16384.0f * 9.81f; //16384 LSB = 9.81 m/s² = 1g
    imumsg.linear_acceleration.y = raw_acceleration_.y() / 16384.0f * 9.81f;
    imumsg.linear_acceleration.z = raw_acceleration_.z() / 16384.0f * 9.81f;
    imumsg.linear_acceleration_covariance = identity_covariance_matrix;
    imu_pub_.publish(imumsg);


    //publish ROS IMU message (free acceleration)
    sensor_msgs::Imu imufreemsg;
    imufreemsg.header.stamp = ros::Time::now();
    imufreemsg.header.frame_id = imu_frame_id_.c_str();
    imufreemsg.orientation.x = q_.x();
    imufreemsg.orientation.y = q_.y();
    imufreemsg.orientation.z = q_.z();
    imufreemsg.orientation.w = q_.w();
    imufreemsg.orientation_covariance = identity_covariance_matrix;
    imufreemsg.angular_velocity.x = raw_gyro_.x() / 16.4f;  //16.4 LSB = 1 rad/sec
    imufreemsg.angular_velocity.y = raw_gyro_.y() / 16.4f;
    imufreemsg.angular_velocity.z = raw_gyro_.z() / 16.4f;
    imufreemsg.angular_velocity_covariance = identity_covariance_matrix;
    imufreemsg.linear_acceleration.x = ( raw_acceleration_.x() / 16384.0f - gravity_corrected.x() )  * 9.81f; //16384 LSB = 9.81 m/s² = 1g
    imufreemsg.linear_acceleration.y = ( raw_acceleration_.y() / 16384.0f - gravity_corrected.y() )  * 9.81f;
    imufreemsg.linear_acceleration.z = ( raw_acceleration_.z() / 16384.0f - gravity_corrected.z() )  * 9.81f;
    imufreemsg.linear_acceleration_covariance = identity_covariance_matrix;
    imu_free_pub_.publish(imufreemsg);

/*
    //magnetometer message
    sensor_msgs::MagneticField magmsg;
    magmsg.header.stamp = ros::Time::now();
    magmsg.header.frame_id = imu_frame_id_;
    magmsg.magnetic_field.x = magnetometer_.x();
    magmsg.magnetic_field.y = magnetometer_.y();
    magmsg.magnetic_field.z = magnetometer_.z();
    magmsg.magnetic_field_covariance = identity_covariance_matrix;
    mag_pub_.publish(magmsg);
*/

    //publish TF to ROS
    if(publish_tf_)
    {
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 1.0) );
        transform.setRotation(tf::Quaternion(q_.x(), q_.y(), q_.z(), q_.w()));
        //transform.setRotation(tf::Quaternion(q_.w(), q_.x(), q_.y(), q_.z()));
        tf_bc_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id_ , "imu_debug_frame" ));
    }
}

int main(int argc, char **argv)
{
    //ros::Rate r(100.0);

    //Initiate ROS
    ros::init(argc, argv, "mpu6050_node");

    //Create an object of class IMU_Filter that will take care of everything
    CriusNode c;

    ros::spin();

    return 0;
}
