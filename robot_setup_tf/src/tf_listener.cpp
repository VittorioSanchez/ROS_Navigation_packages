#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

tf::TransformListener* p_listener;
ros::Publisher* p_publisher;


void LaserCallback(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
{
    ROS_INFO("I heard: [%.2f]", cloud_msg->points[0].x);
  
    try{
    sensor_msgs::PointCloud base_cloud;
    p_listener->transformPointCloud("base_link", ros::Time::now()-ros::Duration(0.02), *cloud_msg, "base_laser", base_cloud);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
       cloud_msg->points[0].x, cloud_msg->points[0].y, cloud_msg->points[0].z,
       base_cloud.points[0].x, base_cloud.points[0].y, base_cloud.points[0].z, base_cloud.header.stamp.toSec());

    p_publisher->publish(base_cloud);
      
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Publisher transformed_cloud_pub = n.advertise<sensor_msgs::PointCloud>("tf_laser", 50); //Publisher for the transformed PointCloud
  p_publisher = &transformed_cloud_pub;
  
  tf::TransformListener listener(ros::Duration(10)); //PointCloud Transform listener
  p_listener = &listener;
  
  //We create our listener on the laser topic where PointClouds are published
  //We will call and listen a transform every time a PointCloud is published
  ros::Subscriber sub = n.subscribe("laser", 1000, LaserCallback);

  ros::spin();

  return 0;
}
