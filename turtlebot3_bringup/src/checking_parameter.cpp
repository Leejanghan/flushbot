#include "ros/ros.h"
#include "std_msgs/Float32.h"

void cb(const std_msgs::Float32::ConstPtr&msg){
	ROS_INFO("%f", msg->data);
}

int main(int argc, char **argv){
	ros::init(argc,argv, " theta_sub");
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("theta",1000, cb); 
	ros::spin();
}	
