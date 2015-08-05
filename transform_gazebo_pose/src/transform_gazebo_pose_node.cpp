#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelState.h>

#include <boost/program_options.hpp>
#include <iostream>
using namespace std;

/*****************************************************************************************************************
* Initialization
*/

std::string pose_update_topic;

//publisher
ros::Publisher pub;
ros::Publisher gazebo_pub;

void init()
{
	//robot pose subscriber from optitrack
}

void chatterCallback(const geometry_msgs::TransformStamped &msg)
{
  //ROS_INFO("I heard ...");

    /*
  geometry_msgs::PoseStamped pub_msg;
  pub_msg.header.frame_id = msg.header.frame_id;

  pub_msg.pose.position.x = msg.transform.translation.x;
  pub_msg.pose.position.y = msg.transform.translation.y;
  pub_msg.pose.position.z = msg.transform.translation.z;

  pub_msg.pose.orientation.x = msg.transform.rotation.x;
  pub_msg.pose.orientation.y = msg.transform.rotation.y;
  pub_msg.pose.orientation.z = msg.transform.rotation.z;
  pub_msg.pose.orientation.w = msg.transform.rotation.w;

  pub.publish(pub_msg);
  */

    gazebo_msgs::ModelState state;
    state.model_name = "quadrotor";
    state.pose.position.x = msg.transform.translation.x;
    state.pose.position.y = msg.transform.translation.y;
    state.pose.position.z = msg.transform.translation.z;

    state.pose.orientation.x = msg.transform.rotation.x;
    state.pose.orientation.y = msg.transform.rotation.y;
    state.pose.orientation.z = msg.transform.rotation.z;
    state.pose.orientation.w = msg.transform.rotation.w;

    state.reference_frame = "world";

    gazebo_pub.publish(state);
}

int main (int argc, char** argv)
{

  ros::init(argc, argv, "transform_gazebo_pose");
  ros::NodeHandle n;

  //subscriber
  ros::Subscriber sub = n.subscribe("/kinton/pose", 1, chatterCallback);
  //publisher
  pub = n.advertise<geometry_msgs::PoseStamped>("/command/pose", 1);
  gazebo_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

  ros::spin();

  return 0;
}
