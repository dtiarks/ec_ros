#include <ros/ros.h>


#include <iostream>

#include "ec_ros.h"




int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ec_ros");
  ros::NodeHandle nh;
  
  align *ec_ros = new align("orient_trans.pcd", EUCLIDEAN_SEGMENT);
  ec_ros->nh=nh;

  // Create a ROS publisher for the output point cloud
  ros::Publisher pubScene = nh.advertise<sensor_msgs::PointCloud2> ("scene", 1);
  ros::Publisher pubRef = nh.advertise<sensor_msgs::PointCloud2> ("ref", 1);
  ros::Publisher pubAligned = nh.advertise<sensor_msgs::PointCloud2> ("aligned", 1);
  ros::Publisher pubSegmented = nh.advertise<sensor_msgs::PointCloud2> ("segmented", 1);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("raw_scene", 1000, &align::cloud_cb,ec_ros);

 
  // Spin
  ros::Rate loop_rate(0.5);
  while(ros::ok())
  {
	  ros::spinOnce ();
	  loop_rate.sleep();
	  ec_ros->publishScene(&pubScene);
    ec_ros->publishSegmented(&pubSegmented);
	  ec_ros->publishRef(&pubRef);
	  ec_ros->publishAligned(&pubAligned);
  }
}





