#include <ros/ros.h>
// PCL specific includes

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "std_msgs/Float32MultiArray.h"

ros::Publisher trimmed_pub;

void object_box_cb();
void cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
void object_corner_cb (const std_msgs::Float32MultiArray input);

float topright_x;
float topright_y;
float bottomleft_x;
float bottomleft_y;
int corner_counter = 0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
//void cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
	printf("HERE.\n");
	Eigen::Vector4f minPoint;
	minPoint[0]= bottomleft_x;  // define minimum point x
	minPoint[1]= bottomleft_y;  // define minimum point y
	minPoint[2]= 1.0;  // define minimum point z
	minPoint[3]= 1.0;
	Eigen::Vector4f maxPoint;
	maxPoint[0]= topright_x;  // define max point x
	maxPoint[1]= topright_y;  // define max point y
	maxPoint[2]= 1.0;  // define max point z 
	maxPoint[3]= 1.0;

	printf("MIN_x. %f \n", bottomleft_x);
	printf("MIN_y. %f \n", bottomleft_y);
	printf("max_x. %f \n", topright_x);
	printf("max_y. %f \n", topright_y);


	for(int i=0; i < maxPoint.size(); i++){

		//eprintf("max poin:. %f \n", maxPoint[i]);

	}

	for(int i=0; i < minPoint.size(); i++){

		///printf("min poin:. %f \n", minPoint[i]);

	}

	

	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered ;//= new pcl::PCLPointCloud2;

  	// Convert to PCL data type
  	pcl_conversions::toPCL(*input, *cloud);

	pcl::CropBox<pcl::PCLPointCloud2> cropper(true);
	cropper.setMax(maxPoint);
	cropper.setMin(minPoint);
	cropper.setInputCloud(cloudPtr);
	cropper.filter(cloud_filtered);
	//cropper.filter(filteredIndices);

	
	 // Perform the actual filtering
	 //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	 //sor.setInputCloud (cloudPtr);
	 //sor.setLeafSize (0.1, 0.1, 0.1);
	 //sor.filter (cloud_filtered);
	

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//sensor_msgs::PointCloud2 data;
	
	pcl_conversions::fromPCL(cloud_filtered, output);

	//output.data == data.data;
	

	// Publish the data
	trimmed_pub.publish (output);
	//printf("NOW.\n");
}

void object_corner_cb (const std_msgs::Float32MultiArray input){
	
	
	topright_x = input.data[corner_counter + 3];
	topright_y = input.data[corner_counter + 2];
	bottomleft_x = input.data[corner_counter + 1];
	bottomleft_y = input.data[corner_counter + 0];

	
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_cropper");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber pcl_sub = nh.subscribe ("velodyne_points", 1, cloud_cb);
  ros::Subscriber object_sub = nh.subscribe ("object_corners", 1, object_corner_cb);


  // Create a ROS publisher for the output point cloud
  trimmed_pub = nh.advertise<sensor_msgs::PointCloud2> ("trimmed_pc", 1);

  // Spin
  ros::spin ();
}
