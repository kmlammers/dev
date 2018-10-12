/*




*/



#include "ros/ros.h"


#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "object_recognition_msgs/RecognizedObject.h"

#include "sstream"
#include <stdint.h>
#include <string>
#include <iostream>




//ros::Subscriber object_sub;
//ros::Publisher object_pub;

void object_listener(const std_msgs::Float32MultiArray input);
void pcl_listener(const std_msgs::Float32MultiArray input);

void object_listener(const std_msgs::Float32MultiArray input){
	
	



}

void pcl_listener(const std_msgs::Float32MultiArray input){
	
	



}

void publisher(){

	object_recognition_msgs::RecognizedObject object;

	//object.point_clouds == 


}


int main(int argc, char **argv){

	ros::init(argc, argv, "object_converter");

	ros::NodeHandle nh;


	
	ros::Publisher 	object_pub = nh.advertise<object_recognition_msgs::RecognizedObject>("/recognized_object_array", 10);

//ros::Publisher 	object_pub = nh.advertise<std_msgs::String>("/recognized_object_array", 10);

ros::Subscriber	object_sub = nh.subscribe("/objects", 10, object_listener);
ros::Subscriber pcl_sub = nh.subscribe("/velodyne_points", 10, pcl_listener);

	ros::spin();


}
