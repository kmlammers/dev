#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/input.h>
#include <unistd.h>
// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
ros::Publisher pub;
ros::Publisher display_publisher;
ros::Publisher marker_pub;


std::string inputString;
const double jump_threshold = 0.0;
const double eef_step = 0.01;
std::vector < float > temp_letter;
std::vector < float > temp_coordinates;
std::vector < float > current_letter;
std::vector < float > tablet_coordinate_x;
std::vector < float > tablet_coordinate_y;

//std::atomic_boolean stop = false;

void take_text_input();
void take_tablet_input();
void get_tablet_input();
void build_output();
std::vector < float > build_letter(char letter);
void build_path();

namespace rvt = rviz_visual_tools;

void take_tablet_input() {
   std::cout << "READY TO TAKE TABLET INPUT!!\n";
   //std::cout << "Press 'S' to start input, draw, and press 'X' to stop. \n";

   char ch;
   bool loop = false;
   /*
      while(loop==false)
       {
        std::cout<<"press escape to end loop"<<std::endl;
        ch=std::getchar();
        if(ch==27)
        loop=true;
       }*/

   //std::cout<<"loop terminated"<<std::endl;

   ///get_tablet_input();

   //boost::thread t(get_tablet_input); // Separate thread for loop.
   //t.start(); // This actually starts a thread.

   // Wait for input character (this will suspend the main thread, but the loop
   // thread will keep running).
   //cin.get();

   // Set the atomic boolean to true. The loop thread will exit from 
   // loop and terminate.
   //stop = true;

   // ... other actions ...

   //return EXIT_SUCCESS; 

   get_tablet_input();
   std::cout << " \n";
   build_path();
}

void get_tablet_input() {
   //std::string exit = "false";
   std::string flag = "";
   struct input_event event;
   int i = 0;
   //while(flag == "") {
   std::cout << "\n RECORDING \n";
   for (int i = 0; i < 2500; i++) {

      if (read(0, & event, sizeof(event)) != sizeof(event)) {
         exit(EXIT_FAILURE);
      }

      switch (event.type) {
      case EV_ABS:
         switch (event.code) {
         case ABS_X:
            printf("ABS_X: %d\n", event.value * 0.0002645833);
            tablet_coordinate_x.push_back(event.value * 0.0002645833);
            break;
         case ABS_Y:
            printf("ABS_Y: %d\n", event.value * 0.0002645833);
            tablet_coordinate_y.push_back(event.value * 0.0002645833);
            break;
         default:
            //std::cout << "keep drawing \n";
            break;
         }
      }

      //std::cout << "here \n";
      if (tablet_coordinate_x.size() != tablet_coordinate_y.size()) {
         std::cout << " BAD SIZES! \n";

      }

      std::cout << "i: " << i << "\n";
   } //end of WHILE
   std::cout << "\n DONE RECORDING \n";

} //end of function

void build_path() {
	
	float x_val = 0.0;
	float y_val = 0.0;
	std::vector < float > test_x;
	std::vector < float > test_y;
	float s = 0.1;
	
	for (float i=0; i < 200; i++){
	
		test_x.push_back(x_val);
		
		x_val += s;
		
	}

	for (float i=0; i < 200; i++){
	
		test_y.push_back(y_val);
		
		y_val += s;
	}
	
	for (int i=0; i < test_x.size(); i++){
		
		std::cout << "test_x: " << test_x[i] << "test_y: " << test_y[i] << "\n";	
		
	}
	
   std::cout << "***BUILD_PATH*** \n";


   if (tablet_coordinate_x.size() != tablet_coordinate_y.size()) {
      std::cout << " BAD SIZES! \n";
   }
	
	 //std::cout << "X SiZE: " << tablet_coordinate_x.size() << "\n";
   //std::cout << "y SiZE: " << tablet_coordinate_y.size() << "\n";
	
	
 // if(tablet_coordinate_x.size() < tablet_coordinate_y.size()){
	  
	//ablet_coordinate_y.resize(tablet_coordinate_x.size());
	  
 // }
//else{
	
	// tablet_coordinate_x.resize(tablet_coordinate_y.size());
	
//}
	
	
   std::cout << "X SiZE: " << tablet_coordinate_x.size() << "\n";
   std::cout << "y SiZE: " << tablet_coordinate_y.size() << "\n";
   
  std::cout << "NOW!!! \n";
	
  ros::AsyncSpinner spinner(1);
  spinner.start();
	
  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
	
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group("group1");
	std::cout << "2!!! \n";
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  std::cout << "3!!! \n";
	
	// Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup("group1");
	
  std::cout << "4!!! \n";
	

 // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("arm_7_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);


	std::cout << "6!!! \n";
	
	
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  //Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  //text_pose.translation().z() = 1.75;
  //visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	std::cout << "7!!! \n";
	
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

	
	std::cout << "8!!! \n";
	
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
	geometry_msgs::Pose start_pose;
	
	robot_state::RobotState start_state(*move_group.getCurrentState());
	
	start_state.setFromIK(joint_model_group, start_pose);
	
	
   std::vector<geometry_msgs::Pose> waypoints;
   waypoints.push_back(start_pose);

   //geometry_msgs::Pose start_pose;

   //start_state.setFromIK(joint_model_group, start_pose);

   //start_pose == move_group.getCurrentState(); 
	
	//---------------------------
	// z is front to back 
	
	//start_pose.position.x = 0.0;
	//start_pose.position.y = 0.0;
	//start_pose.position.z = 0.0;

    //waypoints.push_back(start_pose);
	
	geometry_msgs::Pose new_pose = start_pose;
	
	
	new_pose.position.x = 0.84024;
	new_pose.position.y = 0.100;
	new_pose.position.z = 0.76821;
	waypoints.push_back(new_pose);
	
	
	//new_pose.position.y = 0.5;
	//waypoints.push_back(new_pose);
	
	
	//new_pose.position.y -= 1.83138e-05;
	//waypoints.push_back(new_pose);
	
	
  std::cout << "9!!! \n";
  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations

  for (int i = 0; i < test_x.size(); i++) {

      //std::cout << "Letter (X,Y): " << tablet_coordinate_x[i] << "," << tablet_coordinate_y[i] <<   "\n";
      geometry_msgs::Pose new_pose;
	  
	  
	  /*visualization_msgs::Marker new_marker;
	  new_marker.header.frame_id = "world";
	  new_marker.ns = "my_namespace";
	  new_marker.type = visualization_msgs::Marker::SPHERE;
	  new_marker.action = visualization_msgs::Marker::ADD;
	  new_marker.pose.position.x = 0.4;
	  new_marker.pose.position.y = 0.4;
	  new_marker.pose.position.z = 0.4;
	  new_marker.color.a = 1.0; // Don't forget to set the alpha!
	  new_marker.color.r = 0.0;
	  new_marker.color.g = 1.0;
	  new_marker.color.b = 0.0;
	  new_marker.pose.orientation.x = 0.0;
	  new_marker.pose.orientation.y = 0.0;
	  new_marker.pose.orientation.z = 0.0;
	  new_marker.pose.orientation.w = 1.0;*/
	  
	  
	  
      //new_pose.position.z = test_x[i] ;
	  //std::cout << "x: " << tablet_coordinate_x[i] << " y: " << tablet_coordinate_y[i] << "\n"; 
      //new_pose.position.y = test_y[i] ;
	  
	 // if((tablet_coordinate_x[i] && tablet_coordinate_y[i]) == 0.0){
		  
		//std::cout << "hello"; 
		  
	  //}
      //new_pose.position.x = 1.0;
     //waypoints.push_back(new_pose); // up and out
	  
	  //marker_pub.publish(new_marker);
   
  }
	
	 //move_group.setPoseTarget(new_pose);
     //move_group.setGoalTolerance(0.01);
		
  std::cout << "10!!! \n";
  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
 // move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
	
	//------------------------------------
	std::cout << "11!!! \n";
     move_group.move();
    //------------------------------------

	/*for (int i = 0; i < tablet_coordinate_x.size(); i++) {

      std::cout << "Letter (X,Y): " << tablet_coordinate_x[i] << "," << tablet_coordinate_y[i] <<   "\n";
      //current_letter = build_letter(input_data[i]);
      //current_letter = build_letter();
      geometry_msgs::Pose new_pose;
      new_pose.position.x = tablet_coordinate_x[i];
      new_pose.position.y = tablet_coordinate_y[i];
      new_pose.position.z = 0.0;
      waypoints.push_back(new_pose); // up and out
   }
   
   
   moveit_msgs::RobotTrajectory traj_msg;
   //std::double suc = group;
   double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, traj_msg);
   
   std::cout << " fraction:: " << fraction*100.0 << "\n";
   
   
   //	moveit_msgs::DisplayRobotState display_state;

   //	display_state.state = robot_state;
   //	display_publisher.publish(display_state);
   
   

   // Visualize the plan in RViz
  
 // visual_tools.deleteAllMarkers();
  //visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  //visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  //for (std::size_t i = 0; i < waypoints.size(); ++i)
    //visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  //visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  // std::cout << "suc: " << suc;*/
  

}

int main(int argc, char * argv[]) {
   // Initialize ROS
   ros::init(argc, argv, "tablet_publisher");
   ros::NodeHandle nh;

   //Create a ROS subscriber for the input point cloud
   //ros::Subscriber pcl_sub = nh.subscribe ("velodyne_points", 1, cloud_cb);
   //ros::Subscriber object_sub = nh.subscribe ("object_corners", 1, object_corner_cb);

   // Create a ROS publisher for the output point cloud
   pub = nh.advertise < std_msgs::String > ("pub", 1);
   display_publisher = nh.advertise < moveit_msgs::DisplayRobotState > ("/display_robot_state", 1);
   marker_pub = nh.advertise<visualization_msgs::Marker>( "markers", 0 );

   //--------------------------------------------------
   //RUN PROGRAM
   //take_input();
   //char letter = 'a';
   //take_input();
   //take_tablet_input();
	 build_path();
   std::cout << "DONE!.\n";
   //--------------------------------------------------

   // Spin
   ros::spin();
}
