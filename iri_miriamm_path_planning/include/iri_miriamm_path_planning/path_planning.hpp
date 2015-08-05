/** \brief
 *  \file iri_miriamm_path_planning.cpp
 *  \author Peter Lepej
 *  \date 12.12.2012
 *  \version 1.0
 */

#ifndef iri_miriamm_path_planning_hpp___
#define iri_miriamm_path_planning_hpp___ //header is included only once

//basic
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <list>
#include <cmath>
#include <sstream>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <message_filters/subscriber.h>


//path_planning
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
/*
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
*/

//visualization and tf
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

//octomap
#include <octomap_msgs/GetOctomap.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/GetOctomapResponse.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/BoundingBoxQueryRequest.h>
#include <octomap_msgs/BoundingBoxQueryResponse.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap/AbstractOcTree.h>

#include <octomap/AbstractOccupancyOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/CountingOcTree.h>
#include <octomap/OcTreeIterator.hxx>
#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeBaseImpl.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTreeNode.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap/math/Utils.h>

//octomap visualization in rviz
#include <visualization_msgs/MarkerArray.h>
#include "visualization_msgs/Marker.h"

//octomap server
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <octomap_server/OctomapServer.h>

//kdl parser
#include <kdl_parser/kdl_parser.hpp>

using namespace std;
//using octomap_msgs::GetOctomap;
using namespace octomap;
//using namespace octomath;

#define PI 3.14159265

namespace iri_miriamm_path_planning  //split up code more classes whit the same name
{
    class iri_miriamm_path_planning
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        iri_miriamm_path_planning(const iri_miriamm_path_planning &src);


        protected:
        ros::NodeHandle nh_;

        //subscribers
        ros::Subscriber pose_update_sub;
        ros::Subscriber goal_pose_sub;
        ros::Subscriber octomap_sub;
        //publishers
        ros::Publisher marker_pub;
        ros::Publisher path_pub;
        ros::Publisher path_marker_pub;
        ros::Publisher octree_marker_pub1;
        ros::Publisher colision_marker_pub;
        ros::Publisher box_marker_pub;

        public:

        //global arguments
        std::string pose_update_topic;
        std::string set_goal_topic;
        std::string octomap_topic;
        std::string path_topic;
        std::string global_frame;

        //path planning
        double yaw, pitch, roll;
        double global_x, global_y, global_z;
        double curr_pose_x, curr_pose_y, curr_pose_z;
        double rob_col_x, rob_col_y, rob_col_z;
        int co_pl;
        nav_msgs::Path publish_path;
        octomap::OcTree* octree;
        bool pose_occupied;
        double colision_check_per_rob_size;
        bool show_markers;
        double desired_altitute;
        double expand_path_search;
        double start_pose_x;
        double start_pose_y;
        double start_pose_z;
        double occuped_area_x[500];
        double occuped_area_y[500];
        double occuped_area_z[500];
        int co_occ;
        int start_pose_co;
        int end_pose_co;
        int start_path_co;
        double global_x_orig, global_y_orig, global_z_orig;
        bool second_part_free;
        double explore_z_down_limit;
        double explore_z_up_limit;
        double waypoints_raster;
        double octree_res;
        bool start_planning;


        //robot model

        /** \brief Standard construktor.
         *
         */
        iri_miriamm_path_planning(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~iri_miriamm_path_planning(); //always virtual

        //initialization function
        void init();
        //data calback functions
        void pose_update(const geometry_msgs::PoseStamped pose);
        //set robot goal
        void set_robot_goal(const geometry_msgs::PoseStamped pose);
        //octomap callback
        //void octomap_callback(const octomap_msgs::OctomapConstPtr& msg);
        void call_octomap();
        //global path calc main
        void calculate_global_path();
        //read robot model
        void get_robot_model();
        //robot colision checking
        void robot_colision_checking_pub(const geometry_msgs::PoseStamped pose);
        //check for colisions at position
        void check_position_colision(const geometry_msgs::PoseStamped pose);
        //publish box marker
        void pub_box_marker(const geometry_msgs::PoseStamped pose, const geometry_msgs::PoseStamped pose1);
        //find longest axis for serach dir
        int get_longest_axis();
        //check colison on two given waypoints
        geometry_msgs::PoseStamped find_free_path_between_two_waypoints();
        //check for colision between two points
        bool check_for_colision_between_two_points();
        //add new waypoint to global path
        void add_new_waypoint(const geometry_msgs::PoseStamped calc_pose, const int ip);
        //publish path and add aditional points if necesary
        void publish_global_path();
    };
}

#endif
