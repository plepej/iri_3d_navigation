/** \brief
 *  \file iri_miriamm_path_follower.hpp
 *  \author Peter Lepej
 *  \date 5.5.2012
 *  \version 1.0
 */

#ifndef iri_miriamm_path_follower_hpp___
#define iri_miriamm_path_follower_hpp___ //header is included only once

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
#include <tf/transform_datatypes.h>

//path_planning
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

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

// [service client headers]
#include <roscpp/Empty.h>

// [action server client headers]
#include <kinton_wp_ctrl/taskreqstAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kinton_wp_ctrl/waypointsAction.h>

using namespace std;
//using octomap_msgs::GetOctomap;
using namespace octomap;
//using namespace octomath;

#define PI 3.14159265

namespace iri_miriamm_path_follower  //split up code more classes whit the same name
{

    //kinton states
    enum PlanStates{
        STOPPED,
        RUNNING,
        STOPPING};

    enum TaskStates{
        LANDED,
        TAKEOFF_REQUEST,
        TAKINGOFF,
        NAVIGATION_REQUEST,
        NAVIGATING,
        LAND_REQUEST,
        LANDING,
        HOVERING};

    class iri_miriamm_path_follower
    {
        private:

        /** \brief Copyconstruction is not allowed.
         */
        iri_miriamm_path_follower(const iri_miriamm_path_follower &src);

        // takoff action
        actionlib::SimpleActionClient<kinton_wp_ctrl::taskreqstAction> take_off_client_;
        kinton_wp_ctrl::taskreqstGoal take_off_goal_;
        bool take_offMakeActionRequest();
        void take_offDone(const actionlib::SimpleClientGoalState& state,  const kinton_wp_ctrl::taskreqstResultConstPtr& result);
        void take_offActive();
        void take_offFeedback(const kinton_wp_ctrl::taskreqstFeedbackConstPtr& feedback);
        bool take_off_action_active_;
        bool take_off_action_succeeded_;

        //waypoint following
        actionlib::SimpleActionClient<kinton_wp_ctrl::waypointsAction> waypoints_client_;
        kinton_wp_ctrl::waypointsGoal waypoints_goal_;
        bool waypointsMakeActionRequest();
        void waypointsDone(const actionlib::SimpleClientGoalState& state,  const kinton_wp_ctrl::waypointsResultConstPtr& result);
        void waypointsActive();
        void waypointsFeedback(const kinton_wp_ctrl::waypointsFeedbackConstPtr& feedback);
        bool waypoints_action_active_;
        bool waypoints_action_succeeded_;

        // land action
        actionlib::SimpleActionClient<kinton_wp_ctrl::taskreqstAction> land_client_;
        kinton_wp_ctrl::taskreqstGoal land_goal_;
        bool landMakeActionRequest();
        void landDone(const actionlib::SimpleClientGoalState& state,  const kinton_wp_ctrl::taskreqstResultConstPtr& result);
        void landActive();
        void landFeedback(const kinton_wp_ctrl::taskreqstFeedbackConstPtr& feedback);
        bool land_action_active_;
        bool land_action_succeeded_;
        bool land_action_finished_;

        PlanStates plan_state_; // Plan states
        TaskStates task_state_; // Task state.
        bool plan_active_; // Running experiment.


        protected:
        ros::NodeHandle nh_;

        //subscribers
        ros::Subscriber pose_update_sub;
        ros::Subscriber octomap_sub;
        ros::Subscriber path_sub;
        //publishers
        ros::Publisher marker_pub;
        ros::Publisher colision_marker_pub;
        ros::Publisher cmd_vel_pub;
        ros::Publisher planned_path_pub;
        ros::Publisher colision_start_pub;
        ros::Publisher colision_end_pub;
        ros::Publisher raycasting_collision_marker_pub;
        ros::Publisher local_path_pub;
        ros::Publisher planned_path_reactive_pub;


        public:

        //global arguments
        std::string pose_update_topic;
        std::string octomap_topic;
        std::string global_frame;
        std::string exploration_path_topic;
        std::string cmd_vel_topic;
        std::string local_path_topic;

        nav_msgs::Path planned_path_full;
        nav_msgs::Path planned_path_reactive;
        geometry_msgs::Twist cmd_vel;
        nav_msgs::Path local_path;

        //path following
        double curr_pose_x, curr_pose_y, curr_pose_z;
        double global_x, global_y, global_z;
        double rob_col_x, rob_col_y, rob_col_z;
        double colision_check_per_rob_size;
        double yaw, pitch, roll;
        bool pose_occupied;
        bool start_path_following;
        nav_msgs::Path global_path;
        double max_lin_velocity;
        double goal_offset;
        bool start_following;
        geometry_msgs::PoseStamped closest_point;
        double octree_res;
        double old_yaw_diff;
        double max_rot_velocity;
        double liftoff_altitude_limit;
        double lookahead_distance;
        int closest_point_pos;
        double lookahead_local;
        bool hit;
        double update_waypoints_freq;
        int update_cmd_vel_freq;

        double avgpf_force_x;
        double avgpf_force_y;
        double avgpf_force_z;
        double avgpf_force;

        bool alignment_mode;
        int co_al;
        double goal_dist;
        double global_desired_an_z;
        double global_desired_an_y;
        double global_desired_an_x;

        bool use_waypoint_commands;
        bool use_vel_commands;
        bool initiate_landing;
        int co_cin;

        double raycasting_net_raster;
        int count_sphere_marker;
        visualization_msgs::MarkerArray sphere_marker;

        double pf_force_vel_x;
        double pf_force_vel_y;
        double pf_force_vel_z;

        double avg_force_x;
        double avg_force_y;
        double avg_force_z;
        double avg_hit_distance;
        double closest_distance;
        double reactive_const_way_pf;

        double lock_x_vel;
        double lock_y_vel;
        double lock_z_vel;

        bool show_markers;
        bool use_stop_before_action;

        double reactive_const_pf;
        int co_octomap;
        OcTreeNode* oc_node;

        //octomap
        octomap::OcTree* octree;

        std::string plan_state;
        std::string tmp_state;

        /** \brief Standard construktor.
         *
         */
        iri_miriamm_path_follower(ros::NodeHandle nh);

        /** \brief Destructor.
         *
         */
        virtual ~iri_miriamm_path_follower(); //always virtual

        //initialization function
        void init();
        //data calback functions
        void pose_update(const geometry_msgs::PoseStamped pose);
        //octomap callback
        void call_octomap();
        //check position colision
        void check_position_colision(const geometry_msgs::PoseStamped pose);
        //publish marker on colision check
        void robot_colision_checking_pub(const geometry_msgs::PoseStamped pose);
        //path callback
        void path_callback(const nav_msgs::Path path);
        //follow path
        geometry_msgs::PoseStamped closest_point_to_curr_pos();
        //colision checkin
        void calculate_potential_field_forces();
        //marker publisher for raycasting
        void raycasting_marker(const geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, double size, double size_z);
        //raycasting colison marker
        void raycasting_collision_marker(const geometry_msgs::PoseStamped pose);
        //calculate pf forces
        void potential_field_calc();
        //octomap callback
        void octomap_callback(const octomap_msgs::Octomap octo);
        //calculate pf
        void calculate_pf();
        //use vel command controlls
        void vel_command();
        //use waypoints controll
        void use_waypoints();
        //smi
        void state_machine_interface();
    };
}

#endif
