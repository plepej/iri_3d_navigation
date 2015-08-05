#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
using namespace std;

ofstream pose_data_x;
ofstream pose_data_y;
ofstream pose_data_z;

ofstream gpath_data_x;
ofstream gpath_data_y;
ofstream gpath_data_z;

ofstream lpath_data_x;
ofstream lpath_data_y;
ofstream lpath_data_z;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void poseCB(const geometry_msgs::PoseStamped pose)
{
  ROS_INFO("I heard POSE");
  pose_data_x << pose.pose.position.x << endl;
  pose_data_y << pose.pose.position.y << endl;
  pose_data_z << pose.pose.position.z << endl;
}

void global_pathCB(const nav_msgs::Path path)
{
	ROS_INFO("I heard GLOBAL PATH");
	for (int i = 0; i < path.poses.size(); i++)
	{
		gpath_data_x << path.poses[i].pose.position.x << endl;
		gpath_data_y << path.poses[i].pose.position.y << endl;
		gpath_data_z << path.poses[i].pose.position.z << endl;
	}
}

void local_pathCB(const nav_msgs::Path path)
{
	ROS_INFO("I heard LOCAL PATH");
	for (int i = 0; i < path.poses.size(); i++)
	{
		lpath_data_x << path.poses[i].pose.position.x << endl;
		lpath_data_y << path.poses[i].pose.position.y << endl;
		lpath_data_z << path.poses[i].pose.position.z << endl;
	}
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bag_to_data");

  ros::NodeHandle n;

  //ros subs
  ros::Subscriber sub = n.subscribe<geometry_msgs::PoseStamped>("/odom_fake", 1, poseCB);
  ros::Subscriber subgb = n.subscribe<nav_msgs::Path>("/global_path", 1, global_pathCB);
  ros::Subscriber sublp = n.subscribe<nav_msgs::Path>("/local_path", 1, local_pathCB);

	//open files
	pose_data_x.open ("/home/peter/bag_data/pose_data_x.txt");
	pose_data_y.open ("/home/peter/bag_data/pose_data_y.txt");
	pose_data_z.open ("/home/peter/bag_data/pose_data_z.txt");

	gpath_data_x.open ("/home/peter/bag_data/gpath_data_x.txt");
	gpath_data_y.open ("/home/peter/bag_data/gpath_data_y.txt");
	gpath_data_z.open ("/home/peter/bag_data/gpath_data_z.txt");

	lpath_data_x.open ("/home/peter/bag_data/lpath_data_x.txt");
	lpath_data_y.open ("/home/peter/bag_data/lpath_data_y.txt");
	lpath_data_z.open ("/home/peter/bag_data/lpath_data_z.txt");

    ros::Rate loop_rate(50);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

  return 0;
}
