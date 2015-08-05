/** \brief
 *  \file iri_miriamm_exploration.cpp
 *  \author Peter Lepej
 *  \date 12.12.2012
 *  \version 1.0
 */

#ifndef iri_miriamm_exploration_hpp___
#define iri_miriamm_exploration_hpp___ //header is included only once

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

//#include <OGRE/OgreSceneNode.h>
//#include <OGRE/OgreSceneManager.h>
//#include <message_filters/subscriber.h>


//path_planning
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
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

//opencv
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/opencv.hpp>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CompressedImage.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;
//using octomap_msgs::GetOctomap;
using namespace octomap;
//using namespace octomath;

#define PI 3.14159265

namespace iri_miriamm_exploration  //split up code more classes whit the same name
{
    class iri_miriamm_exploration
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        iri_miriamm_exploration(const iri_miriamm_exploration &src);


        protected:
        ros::NodeHandle nh_;

        //subscribers
        ros::Subscriber pose_update_sub;
        ros::Subscriber octomap_sub;
        ros::Subscriber goal_pose_sub;
        ros::Subscriber map_2d_sub;
        //publishers
        ros::Publisher marker_pub;
        ros::Publisher octree_marker_pub1;
        ros::Publisher colision_marker_pub;
        ros::Publisher box_marker_pub;
        ros::Publisher new_goal_pub;

        public:

        //global arguments
        std::string pose_update_topic;
        std::string octomap_topic;
        std::string global_frame;
        std::string set_goal_topic;
        std::string map_2d_topic;
        std::string exploration_strategy;

        //exploration
        bool start_exploration;
        double explore_z_up_limit;
        double explore_z_down_limit;
        double explore_max_range;
        double explore_min_range;
        double explore_an_shift;
        double global_x, global_y, global_z;
        double rob_col_x, rob_col_y, rob_col_z;
        double colision_check_per_rob_size;
        double yaw, pitch, roll;
        double curr_pose_x, curr_pose_y, curr_pose_z;
        bool show_markers;
        bool pose_occupied;
        double max_x_world_size;
        double max_y_world_size;
        int explore_rate;
        int explore_node_co;
        double goal_offset;

        double xmax,ymax,zmax;
        double xmin,ymin,zmin;

        double exploration_poses_history[100][10][3];
        double way_choosed[100][1];

        //octomap
        octomap::OcTree* octree;
        OcTreeNode* oc_node;
        double octree_res;
        int co_octomap;

        /** \brief Standard construktor.
         *
         */
        iri_miriamm_exploration(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~iri_miriamm_exploration(); //always virtual

        //initialization function
        void init();
        //data calback functions
        void pose_update(const geometry_msgs::PoseStamped pose);
        //octomap callback
        void call_octomap();
        //exploration
        void get_new_area_exploration_goal();
        //check position colision
        void check_position_colision(const geometry_msgs::PoseStamped pose);
        //publish marker on colision check
        void robot_colision_checking_pub(const geometry_msgs::PoseStamped pose);
        //set new goal
        void set_robot_goal(const geometry_msgs::PoseStamped pose);
        //do auto explore
        void explore_auto();
        //map callback
        void map_callback(const nav_msgs::OccupancyGrid map);
        //octomap save
        void save_octomap();
        void get_new_frontear_goal();
    };
}

#endif
