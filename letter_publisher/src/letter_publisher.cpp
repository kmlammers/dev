#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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
            printf("ABS_X: %d\n", event.value);
            tablet_coordinate_x.push_back(event.value);
            break;
         case ABS_Y:
            printf("ABS_Y: %d\n", event.value);
            tablet_coordinate_y.push_back(event.value);
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

   std::cout << "***BUILD_PATH*** \n";

   std::cout << "X SiZE: " << tablet_coordinate_x.size() << "\n";
   std::cout << "y SiZE: " << tablet_coordinate_y.size() << "\n";

   if (tablet_coordinate_x.size() != tablet_coordinate_y.size()) {
      std::cout << " BAD SIZES! \n";
   }
   
    std::vector < geometry_msgs::Pose > waypoints;
    static const std::string PLANNING_GROUP = "group1";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); 
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


   robot_state::RobotState start_state(*move_group.getCurrentState());
  // moveit_msgs::RobotState robot_state;

   std::cout << "BUILDING NEW POSES \n";

   geometry_msgs::Pose start_pose;

   start_state.setFromIK(joint_model_group, start_pose);

   //start_pose == move_group.getCurrentState();   

   waypoints.push_back(start_pose);
   

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
   
   std::cout << " fraction:: " << fraction*100.0 << "\n";*/
   
   
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
  
  // std::cout << "suc: " << suc;

  

}

int main(int argc, char * argv[]) {
   // Initialize ROS
   ros::init(argc, argv, "letter_publisher");
   ros::NodeHandle nh;

   //Create a ROS subscriber for the input point cloud
   //ros::Subscriber pcl_sub = nh.subscribe ("velodyne_points", 1, cloud_cb);
   //ros::Subscriber object_sub = nh.subscribe ("object_corners", 1, object_corner_cb);

   // Create a ROS publisher for the output point cloud
   pub = nh.advertise < std_msgs::String > ("pub", 1);
   display_publisher = nh.advertise < moveit_msgs::DisplayRobotState > ("/display_robot_state", 1);

   //--------------------------------------------------
   //RUN PROGRAM
   //take_input();
   //char letter = 'a';
   //take_input();
   take_tablet_input();
   std::cout << "DONE!.\n";
   //--------------------------------------------------

   // Spin
   ros::spin();
}
