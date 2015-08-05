/** \brief
 *
 *  \file iri_miriamm_path_planning.cpp
 *  \author Peter Lepej
 *  \date 21.04.2015
 *  \version 1.0
 */

#include </home/peter/ros/catkin_ws/src/iri_miriamm_path_planning/include/iri_miriamm_path_planning/path_planning.hpp>

namespace iri_miriamm_path_planning
{
    /****************************************************************
     * Here we can set up the parameters
     */
    iri_miriamm_path_planning::iri_miriamm_path_planning(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh)
    {

        ros::NodeHandle private_nh("~");

        //general
        private_nh.param<std::string>("global_frame", global_frame, string("/world"));
        private_nh.param<std::string>("pose_update_topic", pose_update_topic, string("/odom_fake"));
        private_nh.param<std::string>("set_goal_topic", set_goal_topic, string("/new_exploration_goal"));
        private_nh.param<std::string>("octomap_topic", octomap_topic, string("/octomap_full"));
        private_nh.param<std::string>("path_topic", path_topic, string("/global_path"));
        //path planning
        private_nh.param<double>("desired_altitude", desired_altitute, 0.4);
        private_nh.param<bool>("show_markers", show_markers, true);
        private_nh.param<double>("expand_path_search", expand_path_search, 0.5);
        private_nh.param<double>("colision_check_per_rob_size", colision_check_per_rob_size, 4.0);
        private_nh.param<double>("explore_z_down_limit", explore_z_down_limit, 0.25);
        private_nh.param<double>("explore_z_up_limit", explore_z_up_limit, 1.0);
        private_nh.param<double>("waypoints_raster", waypoints_raster, 0.3);
        //robot
        private_nh.param<double>("robot_colision_x", rob_col_x, 1.0);
        private_nh.param<double>("robot_colision_y", rob_col_y, 1.0);
        private_nh.param<double>("robot_colision_z", rob_col_z, 0.45);

    }
    /****************************************************************
     *
     */
    iri_miriamm_path_planning::~iri_miriamm_path_planning()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void iri_miriamm_path_planning::init()
    {
        //robot pose subscriber
        pose_update_sub = nh_.subscribe<geometry_msgs::PoseStamped>(pose_update_topic ,10 , &iri_miriamm_path_planning::pose_update, this);

        //robot goal pose topic
        goal_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(set_goal_topic ,10 , &iri_miriamm_path_planning::set_robot_goal, this);

        //publish generated global path
        path_pub = nh_.advertise<nav_msgs::Path>(path_topic, 1);

        path_marker_pub = nh_.advertise<nav_msgs::Path>("/global_path_marker", 1);

        //publish robot goal marker
        marker_pub = nh_.advertise<visualization_msgs::Marker>("robot_goal_marker", 1);

        //publish robot colision marker
        colision_marker_pub = nh_.advertise<visualization_msgs::Marker>("obstacle_box_marker", 1);

        //publish box marker
        box_marker_pub = nh_.advertise<visualization_msgs::Marker>("robot_colision_marker", 1);

        //octree marker publiser
        octree_marker_pub1 = nh_.advertise<visualization_msgs::MarkerArray>("octomap_pub1", 1);

        co_pl = 0;
        second_part_free = true;
        start_planning = false;

        //set global goal default position
        global_x = 0.0;
        global_y = 0.0;
        global_z = 0.0;
    }
    /*****************************************************************************************************************
     * Pose update
     */
    void iri_miriamm_path_planning::pose_update(const geometry_msgs::PoseStamped pose)
    {
        //convert quaternionst to roll pitch yaw
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.pose.orientation, q);
        tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

        curr_pose_x = pose.pose.position.x;
        curr_pose_y = pose.pose.position.y;
        curr_pose_z = pose.pose.position.z;

        //ROS_INFO("Robot Position: X: %f / Y: %f / Z: %f Orientation: Z: %f / Y: %f / X: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw, pitch, roll);

    }
    /*****************************************************************************************************************
     * Set robot goal
     */
    void iri_miriamm_path_planning::set_robot_goal(const geometry_msgs::PoseStamped pose)
    {

        global_x = pose.pose.position.x;
        global_y = pose.pose.position.y;
        global_z = pose.pose.position.z;

        ROS_INFO("Robot GOAL Position: X: %f / Y: %f / Z: %f", global_x, global_y, global_z);

        if((global_x == 0.0)&&(global_y == 0.0)&&(global_z == 0.5))
        {
            //publish MESH marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = global_frame;
            marker.header.stamp = ros::Time();
            marker.ns = "robot_pose";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            //marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = curr_pose_x;
            marker.pose.position.y = curr_pose_y;
            marker.pose.position.z = 0.25;
            marker.pose.orientation.x = pose.pose.orientation.x;
            marker.pose.orientation.y = pose.pose.orientation.y;
            marker.pose.orientation.z = pose.pose.orientation.z;
            marker.pose.orientation.w = pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            //only if using a MESH_RESOURCE marker type:
            marker.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
            marker_pub.publish( marker );

            ROS_INFO("Robot pose Marker Published!");

        }else
        {
            //publish MESH marker
            visualization_msgs::Marker marker;
            marker.header.frame_id = global_frame;
            marker.header.stamp = ros::Time();
            marker.ns = "robot_pose";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            //marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = global_x;
            marker.pose.position.y = global_y;
            marker.pose.position.z = global_z;
            marker.pose.orientation.x = pose.pose.orientation.x;
            marker.pose.orientation.y = pose.pose.orientation.y;
            marker.pose.orientation.z = pose.pose.orientation.z;
            marker.pose.orientation.w = pose.pose.orientation.w;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            //only if using a MESH_RESOURCE marker type:
            marker.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
            marker_pub.publish( marker );

            ROS_INFO("Robot pose Marker Published!");
        }

        //check if exploration has finished
        if((global_x == 0.0)&&(global_y == 0.0)&&(global_z == -100.0))
        {
            ROS_ERROR("Path Planning: STOP!!");

            global_x = 0.0;
            global_y = 0.0;
            global_z = 0.5;

            start_planning = false;

            //send goal path to stop robot!
            ROS_INFO("Path Planner: Published new global path with current pose to STOP the robot!");
            nav_msgs::Path new_path;
            new_path.poses.resize(1);
            new_path.header.frame_id = global_frame;

            //add goal
            new_path.poses[0].pose.position.x = curr_pose_x;
            new_path.poses[0].pose.position.y = curr_pose_y;
            new_path.poses[0].pose.position.z = curr_pose_z;

            path_pub.publish(new_path);
        }
        else
        {
            start_planning = true;
            //calculate global path
            calculate_global_path();
        }
    }
    /*****************************************************************************************************************
     * Publish marker on position
     */
    void iri_miriamm_path_planning::robot_colision_checking_pub(const geometry_msgs::PoseStamped pose)
    {
        double pos_x, pos_y, pos_z;
        pos_x = pose.pose.position.x;
        pos_y = pose.pose.position.y;
        pos_z = pose.pose.position.z;

        //publish MESH marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "robot_colision";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos_x;
        marker.pose.position.y = pos_y;
        marker.pose.position.z = pos_z;
        marker.pose.orientation.x = pose.pose.orientation.x;
        marker.pose.orientation.y = pose.pose.orientation.y;
        marker.pose.orientation.z = pose.pose.orientation.z;
        marker.pose.orientation.w = pose.pose.orientation.w;
        marker.scale.x = rob_col_x;
        marker.scale.y = rob_col_y;
        marker.scale.z = rob_col_z;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.2;
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
        colision_marker_pub.publish( marker );

        //Check for collisions at current position!

    }
    /*****************************************************************************************************************
     * Publish colision box marker
     */
    void iri_miriamm_path_planning::pub_box_marker(const geometry_msgs::PoseStamped pose, const geometry_msgs::PoseStamped pose1)
    {
        //calculate center
        double pos_x, pos_y, pos_z;
        pos_x = (pose.pose.position.x + pose1.pose.position.x)/2;
        pos_y = (pose.pose.position.y + pose1.pose.position.y)/2;
        pos_z = (pose.pose.position.z + pose1.pose.position.z)/2;

        double diff_x, diff_y, diff_z;
        diff_x = abs(pose.pose.position.x - pose1.pose.position.x)+rob_col_x/2;
        diff_y = abs(pose.pose.position.y - pose1.pose.position.y)+rob_col_y/2;
        diff_z = abs(pose.pose.position.z - pose1.pose.position.z)+rob_col_z/2;

        ROS_INFO("Collision Box Publisher: Box center - x: %f, y: %f z: %f", pos_x, pos_y, pos_z);
        ROS_INFO("Collision Box Publisher: Box size - x: %f, y: %f z: %f", diff_x, diff_y, diff_z);

        //publish MESH marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "robot_box_colision";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos_x;
        marker.pose.position.y = pos_y;
        marker.pose.position.z = pos_z;
        marker.pose.orientation.x = pose.pose.orientation.x;
        marker.pose.orientation.y = pose.pose.orientation.y;
        marker.pose.orientation.z = pose.pose.orientation.z;
        marker.pose.orientation.w = pose.pose.orientation.w;
        marker.scale.x = diff_x/2;
        marker.scale.y = diff_y/2; //?????
        marker.scale.z = diff_z/2;
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.4;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        //marker.mesh_resource = "package://hector_quadrotor_description/meshes/quadrotor/quadrotor_base.dae";
        colision_marker_pub.publish( marker );

        //Check for collisions at current position!

    }
    /*****************************************************************************************************************
     * Octomap callback
     */
    void iri_miriamm_path_planning::call_octomap()
    {
        ROS_INFO("Path Planning: octomap callback");
        std::string servname = octomap_topic;
        octomap_msgs::GetOctomap::Request req;
        octomap_msgs::GetOctomap::Response resp;

        while(nh_.ok() && !ros::service::call(servname, req, resp))
        {
          ROS_WARN("Request to %s failed; trying again...", nh_.resolveName(servname).c_str());
          usleep(1000000);
        }

        if (nh_.ok())
        { // skip when CTRL-C

            AbstractOcTree* tree = octomap_msgs::msgToMap(resp.map);
            octree = dynamic_cast<octomap::OcTree*>(tree);

            octree_res = octree->getResolution();

            //ROS_INFO("Tree resolution: %f Octree size: %i", octree->getResolution(), octree->size());
            double x,y,z;
            octree->getMetricSize(x, y, z);
            //ROS_INFO("Tree metric size: x: %f y: %f z: %f", x,y,z);

        }
    }
    /*****************************************************************************************************************
     * Publish Global Path
     */
    void iri_miriamm_path_planning::get_robot_model()
    {
        ROS_INFO("Build robot model for colision checking!");

    }
    /*****************************************************************************************************************
     * Find longest axis for free space search
     */
    int iri_miriamm_path_planning::get_longest_axis()
    {
        //ROS_INFO("Longest Axis Search started!");

        int bigest_dist = 0;

        //find the longest axis
        double dist_axis[3];
        dist_axis[0] = abs(occuped_area_x[start_pose_co] - occuped_area_x[end_pose_co-1]);
        dist_axis[1] = abs(occuped_area_y[start_pose_co] - occuped_area_y[end_pose_co-1]);
        dist_axis[2] = abs(occuped_area_z[start_pose_co] - occuped_area_z[end_pose_co-1]);

        double tmp_dist = 0;
        for(int i=0; i < 3; i++)
        {
            if(dist_axis[i] > tmp_dist)
            {
                tmp_dist = dist_axis[i];
                bigest_dist = i;
            }
        }

        return bigest_dist;
    }
    /*****************************************************************************************************************
     * Check if position is covering octomap
     */
    void iri_miriamm_path_planning::check_position_colision(const geometry_msgs::PoseStamped pose)
    {
        //define node of octree - global
        OcTreeNode* oc_node;

        //defina area search
        int zd0, zd1;//, yd0, yd1, xd0, xd1;

        zd0 = (pose.pose.position.z - rob_col_z/2)*(1/octree_res);
        zd1 = (pose.pose.position.z + rob_col_z/2)*(1/octree_res);

        pose_occupied = false;

        double x, y;
        //double octree_res = octree->getResolution();
        for(int z=zd0; z < zd1; z++)
        {

            //ROS_INFO("Z value: %i", z);

            for (int an=0; an<361; an++)
            {
                for(int rad=0; rad < round(rob_col_x*(1/octree_res)/2); rad++)
                {
                    x = rad*octree_res * cos(an*0.0174532925) + pose.pose.position.x ;
                    y = rad*octree_res * sin(an*0.0174532925) + pose.pose.position.y ;

                    //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x, (double)y, (double)z/10);

                    oc_node = octree->search((double)x, (double)y, (double)z*octree_res);

                    if(oc_node)
                    {
                        if(oc_node->getValue() > 0)
                        {
                            //oc_node->setValue(-2.0);
                            //oc_node->setColor(0, 255, 0);
                            //ROS_INFO("Node is OCCUPIED!");
                            pose_occupied = true;
                        }
                    }
                }   
            }
        }
    }
    /*****************************************************************************************************************
     * Check colision on two waypoints
     */
    geometry_msgs::PoseStamped iri_miriamm_path_planning::find_free_path_between_two_waypoints()
    {

//        ROS_INFO("Path Planner: Path to given Global Goal is NOT SAFE!");

//        for(int i=0; i < co_occ; i++)
//        {
//            ROS_INFO("Occp point: %i, pose: X: %f Y: %f Z: %f", i, occuped_area_x[i], occuped_area_y[i], occuped_area_z[i]);
//        }

//        ROS_INFO("Path Planner: Start Pose: %i End Pose: %i", start_pose_co, end_pose_co);

        if(show_markers == true)
        {
            //publish marker to see colision checking
            geometry_msgs::PoseStamped start_pose;
            geometry_msgs::PoseStamped end_pose;

            start_pose.pose.position.x = occuped_area_x[start_pose_co];
            start_pose.pose.position.y = occuped_area_y[start_pose_co];
            start_pose.pose.position.z = occuped_area_z[start_pose_co];

            end_pose.pose.position.x = occuped_area_x[end_pose_co-1];
            end_pose.pose.position.y = occuped_area_y[end_pose_co-1];
            end_pose.pose.position.z = occuped_area_z[end_pose_co-1];

            pub_box_marker(start_pose, end_pose);
        }

        //find longest axis
        int bigest_dist = get_longest_axis();
        //BASED ON LARGEST AXES DISTANCE WE SEARCH FOR FREE SPACE

        //calculate search parameters for free box space search
        OcTreeNode* oc_node;

        //defina area search
        int zd0, zd1, zd2, zd3, yd0, yd1, yd2, yd3, xd0, xd1, xd2, xd3;

        //Search Based on X AXIS!
        if(bigest_dist == 0)
        {
            //===X===
            ROS_INFO("Path Planner: Free SPACE search in X axis!");
            double middle_x = (occuped_area_x[start_pose_co] + occuped_area_x[end_pose_co-1])/2;
            xd2 = round((middle_x - expand_path_search/2)*(1/octree_res));
            xd3 = round((middle_x + expand_path_search/2)*(1/octree_res));
            xd0 = 0.0;
            xd1 = 0.0;

            //===Y===
            yd0 = round((occuped_area_y[start_pose_co])*(1/octree_res)-expand_path_search*(1/octree_res));
            yd1 = round((occuped_area_y[end_pose_co-1])*(1/octree_res)+expand_path_search*(1/octree_res));
            if(yd0 > yd1)
            {
                //change values
                int tmp = yd1;
                yd1 = yd0;
                yd0 = tmp;
            }
            //add boundaries of search
            yd2 = yd0 - (int)(expand_path_search*(1/octree_res));
            yd3 = yd1 + (int)(expand_path_search*(1/octree_res));

            //===Z===
            zd0 = round((occuped_area_z[start_pose_co])*(1/octree_res)-expand_path_search*(1/octree_res));
            zd1 = round((occuped_area_z[end_pose_co-1])*(1/octree_res)+expand_path_search*(1/octree_res));
            if(zd0 > zd1)
            {
                //change values
                int tmp = zd1;
                zd1 = zd0;
                zd0 = tmp;
            }
            zd2 = zd0 - (int)(expand_path_search*(1/octree_res));
            zd3 = zd1 + (int)(expand_path_search*(1/octree_res));
        }
        //Search Based on Y axis!
        else if(bigest_dist == 1)
        {
            ROS_INFO("Path Planner: Free SPACE search in Y axis!");
            //===X===
            xd0 = round((occuped_area_x[start_pose_co])*(1/octree_res));
            xd1 = round((occuped_area_x[end_pose_co-1])*(1/octree_res));
            if(xd0 > xd1)
            {
                //change values
                int tmp = xd1;
                xd1 = xd0;
                xd0 = tmp;
            }
            //add boundaries of search
            xd2 = xd0 - (int)(expand_path_search*(1/octree_res));
            xd3 = xd1 + (int)(expand_path_search*(1/octree_res));

            //===Y===
            double middle_y = (occuped_area_y[start_pose_co] + occuped_area_y[end_pose_co-1])/2;

            //ROS_INFO("Middle point: %f / Dist1: %f, Dist2: %f", middle_y, occuped_area_y[start_pose_co], occuped_area_y[end_pose_co-1]);
            yd2 = round((middle_y - expand_path_search/2)*(1/octree_res));
            yd3 = round((middle_y + expand_path_search/2)*(1/octree_res));
            yd0 = 0.0;
            yd1 = 0.0;

            //===Z===
            zd0 = round((occuped_area_z[start_pose_co])*(1/octree_res));
            zd1 = round((occuped_area_z[end_pose_co-1])*(1/octree_res));
            if(zd0 > zd1)
            {
                //change values
                int tmp = zd1;
                zd1 = zd0;
                zd0 = tmp;
            }
            zd2 = zd0 - (int)(expand_path_search*(1/octree_res));
            zd3 = zd1 + (int)(expand_path_search*(1/octree_res));

        }
        //Search Baserd on Z axis!
        else if(bigest_dist == 2)
        {
            ROS_INFO("Path Planner: Free SPACE search in Z axis!");
            //===X===
            xd0 = round((occuped_area_x[start_pose_co])*(1/octree_res)-expand_path_search*(1/octree_res));
            xd1 = round((occuped_area_x[end_pose_co-1])*(1/octree_res)+expand_path_search*(1/octree_res));
            if(xd0 > xd1)
            {
                //change values
                int tmp = xd1;
                xd1 = xd0;
                xd0 = tmp;
            }
            //add boundaries of search
            xd2 = xd0 - (int)(expand_path_search*(1/octree_res));
            xd3 = xd1 + (int)(expand_path_search*(1/octree_res));

            //===Y===
            yd0 = round((occuped_area_y[start_pose_co])*(1/octree_res)-expand_path_search*(1/octree_res));
            yd1 = round((occuped_area_y[end_pose_co-1])*(1/octree_res)+expand_path_search*(1/octree_res));
            if(yd0 > yd1)
            {
                //change values
                int tmp = yd1;
                yd1 = yd0;
                yd0 = tmp;
            }
            yd2 = yd0 - (int)(expand_path_search*(1/octree_res));
            yd3 = yd1 + (int)(expand_path_search*(1/octree_res));


            //===Z===
            double middle_z = (occuped_area_z[start_pose_co] + occuped_area_z[end_pose_co-1])/2;
            zd2 = round((middle_z - expand_path_search/2)*(1/octree_res));
            zd3 = round((middle_z + expand_path_search/2)*(1/octree_res));
            zd0 = 0.0;
            zd1 = 0.0;
        }

        //limit z search
        if(zd2 < (int)(explore_z_down_limit*(1/octree_res)))
        {
                zd2 = explore_z_down_limit*(1/octree_res);
        }

        if(zd3 > explore_z_up_limit*(1/octree_res))
        {
            zd3 = explore_z_up_limit*(1/octree_res);
        }

        //ROS_INFO("Boundaries SET!");
//        ROS_INFO("Z boundaries: zd0 %i zd1 %i zd2 %i zd3 %i", zd0, zd1, zd2, zd3);
//        ROS_INFO("Y boundaries: yd0 %i yd1 %i yd2 %i yd3 %i", yd0, yd1, yd2, yd3);
//        ROS_INFO("X boundaries: xd0 %i xd1 %i xd2 %i xd3 %i", xd0, xd1, xd2, xd3);

        //performe search in box shaped are defined by colision points
        double free_pps_x[250];
        double free_pps_y[250];
        double free_pps_z[250];
        int co_free_p = 0;

        //start search
        //double octree_res = octree->getResolution();
        for(int z=zd2; z < zd3; z++)
        {
            z = z + (rob_col_z/colision_check_per_rob_size)*(1/octree_res);
            //skip if in the box
            //if((z < zd0)||(z > zd1))
            //{
                //ROS_INFO("Search Z: %f", (double)z*octree_res);
                for(int y=yd2; y < yd3; y++)
                {
                    y = y + (rob_col_y/colision_check_per_rob_size)*(1/octree_res);
                    //skip if in the box
                    if((y < yd0)||(y > yd1))
                    {
                         //ROS_INFO("Search Y: %f", (double)y*octree_res);
                        for(int x=xd2; x < xd3; x++)
                        {
                            x = x + (rob_col_x/colision_check_per_rob_size)*(1/octree_res);
                            //skip if in the box
                            if((x < xd0)||(x > xd1))
                            {

                                oc_node = octree->search((double)x*octree_res, (double)y*octree_res, (double)z*octree_res);
                                //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x*octree_res, (double)y*octree_res, (double)z*octree_res);

                                if(oc_node)
                                {
                                    if(oc_node->getValue() < 0)
                                    {
                                        //ROS_INFO("Node is FREE!");
                                        geometry_msgs::PoseStamped col_pose;
                                        col_pose.pose.position.x = x*octree_res;
                                        col_pose.pose.position.y = y*octree_res;
                                        col_pose.pose.position.z = z*octree_res;

                                        check_position_colision(col_pose);          //check if there are colisions
                                        if(show_markers == true)
                                        {
                                            robot_colision_checking_pub(col_pose);      //show robot in a map
                                            usleep(100000);
                                        }

                                        //is pose free
                                        if (pose_occupied == false)
                                        {
                                            //add pose to database
                                            free_pps_x[co_free_p] = x*octree_res;
                                            free_pps_y[co_free_p] = y*octree_res;
                                            free_pps_z[co_free_p] = z*octree_res;
                                            co_free_p++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            //}
        }

        //FIX!!!!!!
        //choose free pose to follow based on cost function

        //remove REMOVE POINTS with 0 0 0 coordinate
        double free_pps_x_new[co_free_p];
        double free_pps_y_new[co_free_p];
        double free_pps_z_new[co_free_p];
        int co_free_new = -1;

        //copy to tmp
        for(int i=0; i < co_free_p; i++)
        {
             free_pps_x_new[i] = free_pps_x[i];
             free_pps_y_new[i] = free_pps_y[i];
             free_pps_z_new[i] = free_pps_z[i];
        }

        for(int i=0; i < co_free_p; i++)
        {
            //ROS_INFO("Free Points = X: %f / Y: %f / Z: %f", free_pps_x[i], free_pps_y[i], free_pps_z[i]);
            if((free_pps_x[i] == 0.0)&&(free_pps_y[i] == 0.0 )&&(free_pps_z[i] == 0.0))
            {
                //remove point from list
                ROS_INFO("Path Planner: Removed FREE point with 0 0 0 coordinate!");
            }
            else
            {

                co_free_new++;
                free_pps_x[co_free_new] = free_pps_x_new[i];
                free_pps_y[co_free_new] = free_pps_y_new[i];
                free_pps_z[co_free_new] = free_pps_z_new[i];
            }
        }

        co_free_p = co_free_new;

        for(int i=0; i < co_free_p; i++)
        {
            ROS_INFO("Free Points = X: %f / Y: %f / Z: %f", free_pps_x[i], free_pps_y[i], free_pps_z[i]);
        }

        //if no free path found
        if(co_free_p == 0)
        {
            ROS_ERROR("Path Planner: NO Free points Found, Goal NOT Reachable!");
            start_planning = false;
        }

        ROS_INFO("Path Planner: Choose BEST Free Point!");

        int res_closest = 0;
        if(bigest_dist == 0)
        {
            //ROS_INFO("Path Planner: Search in X axes!");

            //new aproach
            //1. find way where are more free spaces
            //2. filter only the side with more free spaces
            //3. between those free spaces choose the middle one!!!

            //middle value is
            double middle_x = ((double)xd2*octree_res+(double)xd3*octree_res)/2;
            double middle_x_l = (middle_x+(double)xd3*octree_res)/2;
            double middle_x_r = ((double)xd2*octree_res+middle_x)/2;

            ROS_INFO("Path Planning: Middle value: %f, left: %f right: %f", middle_x, middle_x_l, middle_x_r);

            int co_m_l = 0;
            int co_m_r = 0;
            double close_l = 100;
            double close_r = 100;
            int res_close_l = 0;
            int res_close_r = 0;
            for(int j=0; j < co_free_p;j++)
            {
              if(middle_x < free_pps_x[j])
              {
                  co_m_l++;
                  double closest = fabs(free_pps_x[j] - middle_x_l);
                  if(closest < close_l)
                  {
                    close_l = closest;
                    res_close_l = j;
                  }

              }
              if(middle_x > free_pps_x[j])
              {
                  co_m_r++;
                  double closest = fabs(free_pps_x[j] - middle_x_r);
                  if(closest < close_r)
                  {
                    close_r = closest;
                    res_close_r = j;
                  }

              }
            }

            if(co_m_r > co_m_l)
            {
                res_closest = res_close_r;
            }
            else
            {
                res_closest = res_close_l;
            }

            /*
            //find max min values
            double min=100;
            double max=-100;
            int minindex = 0;
            int maxindex = 0;

            //find max and mins
            for(int j=0; j<co_free_p;j++)
            {
              if(max < free_pps_x[j])
              {
                max = free_pps_x[j];
                maxindex = j;
              }
              if(min > free_pps_x[j])
              {
                min=free_pps_x[j];
                minindex = j;
              }
            }

            //ROS_INFO("Max index: %i Min index: %i", maxindex, minindex);
            //ROS_INFO("Max: %f Min: %f", occuped_area_x[maxindex], occuped_area_x[minindex]);

            double mid_x = ((double)free_pps_x[maxindex] + (double)free_pps_x[minindex])/2;
            double diff_tmp = 10;

            //ROS_INFO("Mid point: %f", mid_x);

            //find closest point to zero
            for(int i=0; i < co_free_p; i++)
            {
                double diff_res = fabs(mid_x - fabs(free_pps_x[i]));
                //ROS_INFO("Difference: %f", diff_res);

                if(diff_res < diff_tmp)
                {

                    diff_tmp = diff_res;
                    res_closest = i;
                }
            }

            */
        }else if(bigest_dist == 1)
        {
            ROS_INFO("Path Planner: Search in Y axes!");
            //search in Y axes
            //find max min values


            double middle_y = ((double)yd2*octree_res+(double)yd3*octree_res)/2;
            double middle_y_l = (middle_y+(double)yd3*octree_res)/2;
            double middle_y_r = ((double)yd2*octree_res+middle_y)/2;

            ROS_INFO("Path Planning: Middle value: %f, left: %f right: %f", middle_y, middle_y_l, middle_y_r);

            int co_m_l = 0;
            int co_m_r = 0;
            double close_l = 100;
            double close_r = 100;
            int res_close_l = 0;
            int res_close_r = 0;
            for(int j=0; j < co_free_p;j++)
            {
              if(middle_y < free_pps_y[j])
              {
                  co_m_l++;
                  double closest = fabs(free_pps_y[j] - middle_y_l);
                  if(closest < close_l)
                  {
                    close_l = closest;
                    res_close_l = j;
                  }

              }
              if(middle_y > free_pps_y[j])
              {
                  co_m_r++;
                  double closest = fabs(free_pps_y[j] - middle_y_r);
                  if(closest < close_r)
                  {
                    close_r = closest;
                    res_close_r = j;
                  }

              }
            }

            if(co_m_r > co_m_l)
            {
                res_closest = res_close_r;
            }
            else
            {
                res_closest = res_close_l;
            }


            /*
            double min=100;
            double max=-100;
            int minindex = 0;
            int maxindex = 0;

            //find max and mins
            for(int j=0; j<co_free_p;j++)
            {
              if(max < free_pps_y[j])
              {
                max = free_pps_y[j];
                maxindex = j;
              }
              if(min > free_pps_y[j])
              {
                min=free_pps_y[j];
                minindex = j;
              }
            }

            double mid_y = ((double)free_pps_y[maxindex] + (double)free_pps_y[minindex])/2;
            double diff_tmp = 10;

            for(int i=0; i < co_free_p; i++)
            {
                double diff_res = fabs(mid_y - fabs(free_pps_y[i]));

                if(diff_res < diff_tmp)
                {
                    diff_tmp = diff_res;
                    res_closest = i;
                }
            }

            */

        }else if(bigest_dist == 2)
        {
            ROS_INFO("Path Planner: Search in Z axes!");
            //search in Z axes
            //find max min values


            double middle_z = ((double)zd2*octree_res+(double)zd3*octree_res)/2;
            double middle_z_l = (middle_z+(double)zd3*octree_res)/2;
            double middle_z_r = ((double)zd2*octree_res+middle_z)/2;

            ROS_INFO("Path Planning: Middle value: %f, left: %f right: %f", middle_z, middle_z_l, middle_z_r);

            int co_m_l = 0;
            int co_m_r = 0;
            double close_l = 100;
            double close_r = 100;
            int res_close_l = 0;
            int res_close_r = 0;
            for(int j=0; j < co_free_p;j++)
            {
              if(middle_z < free_pps_z[j])
              {
                  co_m_l++;
                  double closest = fabs(free_pps_z[j] - middle_z_l);
                  if(closest < close_l)
                  {
                    close_l = closest;
                    res_close_l = j;
                  }

              }
              if(middle_z > free_pps_z[j])
              {
                  co_m_r++;
                  double closest = fabs(free_pps_z[j] - middle_z_r);
                  if(closest < close_r)
                  {
                    close_r = closest;
                    res_close_r = j;
                  }
              }
            }

            if(co_m_r > co_m_l)
            {
                res_closest = res_close_r;
            }
            else
            {
                res_closest = res_close_l;
            }


            /*
            double min=100;
            double max=-100;
            int minindex = 0;
            int maxindex = 0;

            //find max and mins
            for(int j=0; j<co_free_p;j++)
            {
              if(max < free_pps_z[j])
              {
                max = free_pps_z[j];
                maxindex = j;
              }
              if(min > free_pps_z[j])
              {
                min=free_pps_z[j];
                minindex = j;
              }
            }

            double mid_z = ((double)free_pps_z[maxindex] + (double)free_pps_z[minindex])/2;
            double diff_tmp = 10;

            for(int i=0; i < co_free_p; i++)
            {
                double diff_res = fabs(mid_z - fabs(free_pps_z[i]));

                if(diff_res < diff_tmp)
                {
                    diff_tmp = diff_res;
                    res_closest = i;
                }
            }
            */
        }

        //check if closest is not ok
        bool check_zero = true;
        while(check_zero == true)
        {
            if((free_pps_x[res_closest] == 0)&&(free_pps_y[res_closest] == 0)&&(free_pps_z[res_closest] == 0))
            {
                ROS_ERROR("Path Planner: Found Zero POSITION for waypoint, change it, res_closest is: %i", res_closest);

                res_closest++;

                if(res_closest > co_free_p)
                {
                    res_closest = res_closest - co_free_p;
                }
            }
            else
            {
                check_zero = false;
                ROS_INFO("Path Planner: Exit ZERO check loop at: %i", res_closest);
            }
        }

        //show results
        ROS_INFO("Path Planner: Selected point: %i // X: %f / Y: %f / Z: %f", res_closest, free_pps_x[res_closest], free_pps_y[res_closest], free_pps_z[res_closest]);

        geometry_msgs::PoseStamped col_pose_fin;
        col_pose_fin.pose.position.x = free_pps_x[res_closest];
        col_pose_fin.pose.position.y = free_pps_y[res_closest];
        col_pose_fin.pose.position.z = free_pps_z[res_closest];

        if(show_markers == true)
        {
            robot_colision_checking_pub(col_pose_fin);      //show robot in a map
            usleep(1000000);
        }

        if((free_pps_x[res_closest]==0)&&(free_pps_y[res_closest]==0)&&(free_pps_z[res_closest]==0))
        {
            ROS_ERROR("Path Planner: There was alternative waypoints found!");

            //chose other point
        }

        return col_pose_fin;
    }
    /*****************************************************************************************************************
     * Insert new point to Global Path
     */
    void iri_miriamm_path_planning::add_new_waypoint(const geometry_msgs::PoseStamped calc_pose, const int ip)
    {
        //ROS_INFO("Path Planner: Add new calculated pose to global path. Insertation point is: %i", ip);

        //publish_path.poses.resize(publish_path.poses.size() +1);
        nav_msgs::Path new_path;
        new_path.poses.resize(publish_path.poses.size() +1);
        new_path.header.frame_id = global_frame;

        //ROS_INFO("Path Planner: New path size: %i", new_path.poses.size());  //insert new pose

        for(int i=0; i < new_path.poses.size(); i++)
        {
            if(i == (ip +1))
            {
                //add new pose
                new_path.poses[i].pose.position.x = calc_pose.pose.position.x;
                new_path.poses[i].pose.position.y = calc_pose.pose.position.y;
                new_path.poses[i].pose.position.z = calc_pose.pose.position.z;
            }
            else if(i < (ip +1))
            {
                new_path.poses[i].pose.position.x = publish_path.poses[i].pose.position.x;
                new_path.poses[i].pose.position.y = publish_path.poses[i].pose.position.y;
                new_path.poses[i].pose.position.z = publish_path.poses[i].pose.position.z;
            }
            else if(i > (ip +1))
            {
                new_path.poses[i].pose.position.x = publish_path.poses[i-1].pose.position.x;
                new_path.poses[i].pose.position.y = publish_path.poses[i-1].pose.position.y;
                new_path.poses[i].pose.position.z = publish_path.poses[i-1].pose.position.z;
            }
        }

        //copy path to publihs path
        publish_path.poses.resize(new_path.poses.size());
        publish_path=new_path;

        if(show_markers == true)
        {
            //publish path to see if it is add
            path_marker_pub.publish(publish_path);
        }
        ROS_INFO("Path Planner: New pose added SUCCESSFULY to path!");
    }
    /*****************************************************************************************************************
     * Calculate Globabal Path
     */
    bool iri_miriamm_path_planning::check_for_colision_between_two_points()
    {

        //colision checking on shortest path ----------------------------------------------
        double distance = sqrt(pow(start_pose_x-global_x,2.0)+pow(start_pose_y-global_y,2.0)+pow(start_pose_z-global_z,2.0));
        ROS_INFO("Shortest distance is: %f // Checks done in one robot SIZE: %f", distance, colision_check_per_rob_size);

        int num_col_ch = round((colision_check_per_rob_size*distance/rob_col_x));
        double step_x = (global_x - start_pose_x)/num_col_ch;
        double step_y = (global_y - start_pose_y)/num_col_ch;
        double step_z = (global_z - start_pose_z)/num_col_ch;

        bool path_free = true;
        occuped_area_x[num_col_ch];
        occuped_area_y[num_col_ch];
        occuped_area_z[num_col_ch];

        geometry_msgs::PoseStamped col_pose;

        //set to zero
        start_pose_co = 0;
        end_pose_co = 0;
        int spc = 0;

        co_occ = 0;
        for(int p=1; p < num_col_ch+1 ; p++)
        {
            col_pose.pose.position.x = p*step_x + start_pose_x;
            col_pose.pose.position.y = p*step_y + start_pose_y;
            col_pose.pose.position.z = p*step_z + start_pose_z; //+rob_col_z/2;

            check_position_colision(col_pose);          //check if there are colisions

            if (pose_occupied == true)
            {
                //ROS_INFO("Colision on given PATH!");
                path_free = false;

                //mark occupied area
                //ROS_INFO("Oc Area: X: %f, Y: %f, Z: %f", p*step_x + start_pose_x, p*step_y + start_pose_y, p*step_z + start_pose_z);
                occuped_area_x[co_occ] = (double)(p*step_x + start_pose_x);
                occuped_area_y[co_occ] = (double)(p*step_y + start_pose_y);
                occuped_area_z[co_occ] = (double)(p*step_z + start_pose_z);

                if(spc == 0)
                {
                    start_pose_co = co_occ;
                    spc = spc + 1;
                }
                co_occ = co_occ +1;
            }else
            {
                if(spc == 1)
                {
                    end_pose_co = co_occ;
                    spc = spc + 1;
                }
            }

            if(show_markers == true)
            {
                robot_colision_checking_pub(col_pose);      //show robot in a map
                usleep(100000);
            }
        }

        return path_free;
    }    /*****************************************************************************************************************
     * Calculate Globabal Path
     */
    void iri_miriamm_path_planning::publish_global_path()
    {
        ROS_INFO("Path Planner: Published new global path!");
        nav_msgs::Path new_path;
        new_path.poses.resize(200);
        new_path.header.frame_id = global_frame;

        int co_new = 0;
        for(int i=0; i<publish_path.poses.size()-1; i++)
        {
            double dist = sqrt(pow(publish_path.poses[i].pose.position.x-publish_path.poses[i+1].pose.position.x,2.0)+pow(publish_path.poses[i].pose.position.y-publish_path.poses[i+1].pose.position.y,2.0)+pow(publish_path.poses[i].pose.position.z-publish_path.poses[i+1].pose.position.z,2.0));

            int steps = round(dist/waypoints_raster);

            double step_x = (publish_path.poses[i+1].pose.position.x - publish_path.poses[i].pose.position.x)/steps;
            double step_y = (publish_path.poses[i+1].pose.position.y - publish_path.poses[i].pose.position.y)/steps;
            double step_z = (publish_path.poses[i+1].pose.position.z - publish_path.poses[i].pose.position.z)/steps;

            new_path.poses[co_new].pose.position.x = publish_path.poses[i].pose.position.x;
            new_path.poses[co_new].pose.position.y = publish_path.poses[i].pose.position.y;
            new_path.poses[co_new].pose.position.z = publish_path.poses[i].pose.position.z;
            co_new++;

            if(steps > 0)
            {
                for(int j=1; j < steps; j++)
                {
                    //add point
                    new_path.poses[co_new].pose.position.x = publish_path.poses[i].pose.position.x + step_x*j;
                    new_path.poses[co_new].pose.position.y = publish_path.poses[i].pose.position.y + step_y*j;
                    new_path.poses[co_new].pose.position.z = publish_path.poses[i].pose.position.z + step_z*j;
                    co_new++;
                }
            }

        }

        //add goal
        int last_co = publish_path.poses.size()-1;
        new_path.poses[co_new].pose.position.x = publish_path.poses[last_co].pose.position.x;
        new_path.poses[co_new].pose.position.y = publish_path.poses[last_co].pose.position.y;
        new_path.poses[co_new].pose.position.z = publish_path.poses[last_co].pose.position.z;
        co_new++;

        new_path.poses.resize(co_new);

        path_pub.publish(new_path);
    }
    /*****************************************************************************************************************
     * Calculate Globabal Path
     */
    void iri_miriamm_path_planning::calculate_global_path()
    {
        ROS_INFO("Path Planner: Calculate Global Path");

        //1. Check robot GOAL position =============================================================
        if(((float)global_x == (float)0.0)&&((float)global_y == (float)0.0)&&((float)global_z == (float)0.0))  // if new goal position received
        {
            ROS_ERROR("Path Planner: Global GOAL not DEFINED!");
            start_planning = false;
        }
        else
        {
            co_pl = co_pl +1;
            //ROS_INFO("Robot GOAL Position: X: %f / Y: %f / Z: %f", global_x, global_y, global_z);

            //2. get octomap =======================================================================
            call_octomap();  // global octree structure!

            //3. Check goal position collison ======================================================

            geometry_msgs::PoseStamped col_pose;
            col_pose.pose.position.x = global_x;
            col_pose.pose.position.y = global_y;
            col_pose.pose.position.z = global_z;

            check_position_colision(col_pose);          //check if there are colisions

            if(show_markers == true)
            {
                robot_colision_checking_pub(col_pose);      //show robot in a map
                usleep(100000);
            }

            if(pose_occupied == false)
            {
                ROS_INFO("Robot Goal Destination is FREE for NOW!");
            }else
            {
                ROS_INFO("Robot Goal Destination is OCCUPIED, please choose another position!");
                start_planning = false;
            }

            //4. Draw straght path ================================================================

            //check if robot is at ground
            if(curr_pose_z < 0.25)
            {
                ROS_INFO("Path Planner: Robot is not HOVERING, adding start point to trajectory!");

                publish_path.poses.resize(3);
                publish_path.header.frame_id = global_frame;

                for(int d=0; d < publish_path.poses.size(); d++)
                {

                    if(d == 0) //at start add one point
                    {
                        publish_path.poses[d].pose.position.x = curr_pose_x;
                        publish_path.poses[d].pose.position.y = curr_pose_y;
                        publish_path.poses[d].pose.position.z = curr_pose_z;
                    }
                    if(d == 1) //at start add one point
                    {
                        publish_path.poses[d].pose.position.x = curr_pose_x;
                        publish_path.poses[d].pose.position.y = curr_pose_y;
                        //height set to preferable hovering altiture
                        publish_path.poses[d].pose.position.z = desired_altitute;
                    }
                    if(d > 1) //at start add one point
                    {
                        publish_path.poses[d].pose.position.x = global_x;
                        publish_path.poses[d].pose.position.y = global_y;
                        publish_path.poses[d].pose.position.z = global_z;
                    }
                }

                //lifted robot
                curr_pose_z = desired_altitute;
                //start point shifted to 1 because we have liftoff
                start_path_co = 1;
            }
            else
            {
                //leave path as it was!
                publish_path.poses.resize(2);
                publish_path.header.frame_id = global_frame;
                start_path_co = 0;

                //start point
                publish_path.poses[0].pose.position.x = curr_pose_x;
                publish_path.poses[0].pose.position.y = curr_pose_y;
                publish_path.poses[0].pose.position.z = curr_pose_z;
                //end point
                publish_path.poses[1].pose.position.x = global_x;
                publish_path.poses[1].pose.position.y = global_y;
                publish_path.poses[1].pose.position.z = global_z;
            }

            if(show_markers == true)
            {
                //publlish given path
                path_marker_pub.publish(publish_path);
            }
            //set start point
            start_pose_x = curr_pose_x;
            start_pose_y = curr_pose_y;
            start_pose_z = curr_pose_z;
            //copy global goal for calculating middle point cllision cheking
            global_x_orig = global_x;
            global_y_orig = global_y;
            global_z_orig = global_z;

            //check given path
            bool path_free;

            //do loop and create path
            int go = 1;
            int co_planner = 0;
            geometry_msgs::PoseStamped calc_pose;
            while (go == 1)
            {
                if(start_planning == true)
                {
                    ROS_INFO("Path Planner: Starting planning Loop!");

                    //check whole path
                    bool path_is_free = false;
                    int co_free = 0;
                    for(int i = 0; i < publish_path.poses.size()-1; i++)
                    {
                        start_pose_x = publish_path.poses[i].pose.position.x;
                        start_pose_y = publish_path.poses[i].pose.position.y;
                        start_pose_z = publish_path.poses[i].pose.position.z;

                        global_x = publish_path.poses[i+1].pose.position.x;
                        global_y = publish_path.poses[i+1].pose.position.y;
                        global_z = publish_path.poses[i+1].pose.position.z;

                        path_free = check_for_colision_between_two_points();

                        if(path_free == false)
                        {
                            ROS_INFO("Path Planner: Path between two points is NOT FREE!");

                            calc_pose = find_free_path_between_two_waypoints();

                            if((calc_pose.pose.position.x == 0.0)&&(calc_pose.pose.position.y == 0.0)&&(calc_pose.pose.position.z == 0.0))
                            {
                                ROS_ERROR("Path Planning: Free pose between two path has been set to 0 0 0!");
                                calc_pose.pose.position.z = 0.6;
                            }

                            start_path_co = i;
                            add_new_waypoint(calc_pose, start_path_co);

                            //ROS_INFO("Path Planner: Start path counter: %i", start_path_co);

                            co_planner++;
                        }
                        else
                        {
                            path_is_free = true;
                            co_free++;
                            //ROS_INFO("Path Planner: Path between two points is FREE!");
                        }
                    }

                    //ROS_INFO("Path Planner: Free count: %i, Path Size: %i", co_free, publish_path.poses.size()-1);
                    //if(co_free >= publish_path.poses.size()-2) //pazi za celovito preverjanje morajo biit vsi ok torej -1
                    if(co_free >= publish_path.poses.size()-1) //pazi za celovito preverjanje morajo biit vsi ok torej -1
                    {
                        ROS_INFO("Path Planner: Path to given Global Goal is FREE!");
                        //path is free publish path
                        //path_pub.publish(publish_path);
                        publish_global_path(); //if needed more waypoints on the way
                        //path_pub.publish(publish_path);
                        go = 0;
                        return;
                    }

                    if(co_planner > 4)
                    {
                        ROS_WARN("Path Planner: Path planning has been completed due to timeout!");
                        //path is free publish path
                        //path_pub.publish(publish_path);
                        publish_global_path(); //if needed more waypoints on the way
                        //path_pub.publish(publish_path);
                        go = 0;
                        return;
                    }
                }else
                {
                    ROS_ERROR("Path Planning: Path Planning has FAILED!");
                    return;
                }
            }
        }
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "iri_miriamm_path_planning");
    ros::NodeHandle nh;

    //for registering the node
    iri_miriamm_path_planning::iri_miriamm_path_planning iri_miriamm_path_planning_handler(nh);

    iri_miriamm_path_planning_handler.init();                     //initize

    //iri_miriamm_path_planning_handler.get_robot_model();          //build robot model for colision checking

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
      //iri_miriamm_path_planning_handler.calculate_global_path();

      ros::spinOnce();

      loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
