/*

Copyright Marcel Stüttgen 2015

simple ros node that will publih float data to /mavros/setpoint/att_throttle for throttle control
and a tf /

 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "setpoint_attitude_publisher");
  ros::NodeHandle nh;
  ros::Publisher att_thrust_pub = nh.advertise<std_msgs::Float32>("/mavros/setpoint/att_throttle", 1000);   
  tf::TransformBroadcaster attitude_tf_bc;
  
  ros::Rate loop_rate(10);

  double ros_roll;
  double ros_pitch;
  double ros_yaw;
  double ros_throttle;
  
  int count = 0;
  while (ros::ok())
  {
    nh.param<double>("ros_roll", ros_roll, 0.0);
    nh.param<double>("ros_pitch", ros_pitch, 0.0);
    nh.param<double>("ros_yaw", ros_yaw, 0.0);
    nh.param<double>("ros_throttle", ros_throttle, 0.0);

    tf::Transform tf;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, ros_yaw);
    tf.sendTransform(tf::StampedTransform


    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}

