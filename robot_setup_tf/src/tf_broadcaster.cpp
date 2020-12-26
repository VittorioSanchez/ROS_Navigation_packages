#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    //We create a broadcaster that will listen on ROS topics and wait for the call of a transformation from base_laser to base_link
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.34, 0.0, 0.47)),
        ros::Time::now(),"base_link", "base_laser")); //MODIFY WITH THE REAL OFFSET BETWEEN LASER AND CENTER OF THE CAR
    r.sleep();
  }
}
