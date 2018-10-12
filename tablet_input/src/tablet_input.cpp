#include <ros/ros.h>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/input.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"


int main (int argc, char *argv[])
{
  // Initialize ROS
  ros::init (argc, argv, "tablet_input");
  ros::NodeHandle nh;

   struct input_event event;

   printf("***HERE*** \n");
  
    for (;;) {
        if (read(0, &event, sizeof(event)) != sizeof(event)) {
            exit(EXIT_FAILURE);
        }

        switch(event.type) {
        case EV_ABS:
            switch (event.code) {
            case ABS_X:
                printf("ABS_X: %d\n", event.value);
                break;
            case ABS_Y:
                printf("ABS_Y: %d\n", event.value);
                break;
            case ABS_PRESSURE:
                printf("ABS_PRESSURE: %d\n", event.value);
                break;
            default:
                break;
            }   }
    }

  // Spin
  ros::spin ();
}
