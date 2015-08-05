/** \brief
 *
 *  \file iri_miriamm_exploration.cpp
 *  \author Peter Lepej
 *  \date 5.05.2015
 *  \version 1.0
 */

#include </home/peter/ros/catkin_ws/src/iri_miriamm_exploration/include/iri_miriamm_exploration/exploration.hpp>

namespace iri_miriamm_exploration
{
    /****************************************************************
     * Here we can set up the parameters
     */
    iri_miriamm_exploration::iri_miriamm_exploration(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh)
    {

        ros::NodeHandle private_nh("~");

        //general
        private_nh.param<std::string>("global_frame", global_frame, string("/world"));
        private_nh.param<std::string>("pose_update_topic", pose_update_topic, string("/odom_fake"));
        private_nh.param<std::string>("octomap_topic", octomap_topic, string("/octomap_full"));

        //robot
        private_nh.param<double>("robot_colision_x", rob_col_x, 1.0);
        private_nh.param<double>("robot_colision_y", rob_col_y, 1.0);
        private_nh.param<double>("robot_colision_z", rob_col_z, 0.45);

        //exploration
        private_nh.param<double>("colision_check_per_rob_size", colision_check_per_rob_size, 2.0);
        private_nh.param<double>("explore_z_up_limit", explore_z_up_limit, 0.7);
        private_nh.param<double>("explore_z_down_limit", explore_z_down_limit, 0.25);
        private_nh.param<double>("explore_max_range", explore_max_range, 20.0);
        private_nh.param<double>("explore_min_range", explore_min_range, 4.5);
        private_nh.param<double>("explore_an_shift", explore_an_shift, 20.0);
        private_nh.param<std::string>("set_goal_topic", set_goal_topic, string("/set_exploration_goal"));
        private_nh.param<double>("max_x_world_size", max_x_world_size, 10.0);
        private_nh.param<double>("max_y_world_size", max_y_world_size, 10.0);
        private_nh.param<int>("explore_rate", explore_rate, 2.0);
        //private_nh.param<std::string>("map_2d_topic", map_2d_topic, string("/projected_map"));
        private_nh.param<bool>("show_markers", show_markers, true);
        private_nh.param<std::string>("exploration_strategy", exploration_strategy, string("closest_goal"));

        private_nh.param<double>("goal_offset", goal_offset, 0.10);
    }
    /****************************************************************
     *
     */
    iri_miriamm_exploration::~iri_miriamm_exploration()
    {
    }
    /*****************************************************************************************************************
     * Safe octomap
     */
    void iri_miriamm_exploration::save_octomap()
    {

    }

    /*****************************************************************************************************************
     * Initialization
     */
    void iri_miriamm_exploration::init()
    {
        //robot pose subscriber
        pose_update_sub = nh_.subscribe<geometry_msgs::PoseStamped>(pose_update_topic ,10 , &iri_miriamm_exploration::pose_update, this);

        //publish robot goal marker
        marker_pub = nh_.advertise<visualization_msgs::Marker>("robot_goal_marker", 1);

        //publish box marker
        box_marker_pub = nh_.advertise<visualization_msgs::Marker>("robot_colision_marker", 1);

        //advertise new topic with new goal
        new_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/new_exploration_goal",1);

        //publish robot colision marker
        colision_marker_pub = nh_.advertise<visualization_msgs::Marker>("obstacle_box_marker", 1);

        //robot goal pose topic
        goal_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(set_goal_topic ,10 , &iri_miriamm_exploration::set_robot_goal, this);

        //2d map subscriber
        //map_2d_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(map_2d_topic, 1 , &iri_miriamm_exploration::map_callback, this);

        //set global goal default position
        global_x = 0.0;
        global_y = 0.0;
        global_z = 0.0;

        start_exploration = false;
        explore_node_co = 0;
        co_octomap = 0;

    }
    void iri_miriamm_exploration::map_callback(const nav_msgs::OccupancyGrid map)
    {
        ROS_INFO("Exploration: Map callback");

        //get image
//        int width = map.info.width;
//        int height = map.info.height;
//        int resolution_ = map.info.resolution;

//        //ROS_INFO("Resolution: %f", resolution_);

//        cv::Mat free_space( width, height, CV_8UC1);

//        // build image from occupancy grid
//        for(uint32_t row = 0; row < height; row++)
//        {
//            for(uint32_t col = 0; col <  width; col++)
//            {
//                int8_t calcIndex = (height-1-row)*width+col;
//                int8_t map_value = map.data[calcIndex];


//                if( map_value == 0 )  // free space is set to white
//                {
//                    ((uchar*)(free_space.data + free_space.step*row))[col] = 255;
//                }
//                else // occupated space or unknown is set to black
//                {
//                    ((uchar*)(free_space.data + free_space.step*row))[col] = 0;
//                }

//            }
//        }

//        cv::imshow("fres space map", free_space);
//        cv::waitKey(3);

    }
    /*****************************************************************************************************************
     * Set robot goal
     */
    void iri_miriamm_exploration::set_robot_goal(const geometry_msgs::PoseStamped pose)
    {
        geometry_msgs::PoseStamped new_goal;

        global_x = pose.pose.position.x;
        global_y = pose.pose.position.y;
        global_z = pose.pose.position.z;

        ROS_INFO("Exploration: Robot GOAL Position: X: %f / Y: %f / Z: %f", global_x, global_y, global_z);

        start_exploration = false;

        //if global goal is set to 0 0 0, do automatic exploration
        if((global_x == 0.0)&&(global_y == 0.0)&&(global_z == 0.0))
        {
            ROS_INFO("Exploration: Start Autonomous Exploration Strategy");

            start_exploration = true;

            global_x = curr_pose_x;
            global_y = curr_pose_y;
            global_z = curr_pose_z;

            //get_new_area_exploration_goal();
            get_new_frontear_goal();
            ROS_INFO("Exploration: New goal has been choosed!");

            if(start_exploration == true)
            {
                //publish new goal
                new_goal.pose.position.x = global_x;
                new_goal.pose.position.y = global_y;
                new_goal.pose.position.z = global_z;
                new_goal_pub.publish(new_goal);
                ROS_INFO("Exploration: Robot pose Marker Published!");
            }else
            {
                ROS_ERROR("Exploration: Something gone wrong, exploration STOPED!");
            }

        }
        else
        {
            //publish new goal
            new_goal.pose.position.x = global_x;
            new_goal.pose.position.y = global_y;
            new_goal.pose.position.z = global_z;
            new_goal_pub.publish(new_goal);
            ROS_INFO("Exploration: Robot pose Marker Published!");
        }

        if((global_x == 0.0)&&(global_y == 0.0)&&(global_z == -100))
        {
            //stop exploration
            start_exploration = false;
            ROS_INFO("Exploration: Have been stoped!");

            //publish
            //publish new goal
            new_goal.pose.position.x = global_x;
            new_goal.pose.position.y = global_y;
            new_goal.pose.position.z = global_z;
            new_goal_pub.publish(new_goal);
            ROS_INFO("Exploration: Robot pose Marker Published!");
        }
    }
    /*****************************************************************************************************************
     * Calculate new exploration goal
     */
    void iri_miriamm_exploration::get_new_frontear_goal()
    {
        //call new octomap
        call_octomap();
        ROS_INFO("Exploration: Updated Octomap Received!");

        //set borders
        int zd0 = explore_z_down_limit*(1/octree_res);
        int zd1 = explore_z_up_limit*(1/octree_res);

        int rd1 = round(explore_max_range*(1/octree_res));
        int rd0 = round(explore_min_range*(1/octree_res));

        //explore init
        double x,y;
        double tmp_ex_goals[3];
        int co_unknown = 0;
        double explore_pos[3*300];
        int co_ex_poses = 0;
        geometry_msgs::PoseStamped col_pose;
        bool skip_pose = false;

        //find the frontears
        for(int z=zd0; z < zd1; z++)
        {
            z = z + (rob_col_z/2)*(1/octree_res);
            for (int an=0; an<361; an++)
            {
                an = an + (int)explore_an_shift;   //search angle
                skip_pose = false;
                for(int r=rd0; r < rd1; r++)
                {
                    //increase distance
                    r = r + rob_col_x/colision_check_per_rob_size*(1/octree_res);

                    x = r*octree_res * cos(an*0.0174532925)+curr_pose_x;
                    y = r*octree_res * sin(an*0.0174532925)+curr_pose_y;

                    //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x, (double)y, (double)z*octree_res);
                    oc_node = octree->search((double)x, (double)y, (double)z*octree_res);

                    if(oc_node)
                    {
                        if(oc_node->getValue() > 0)
                        {
                            //oc_node->setValue(-2.0);
                            //oc_node->setColor(0, 255, 0);
                            //ROS_INFO("Node is OCCUPIED!");
                        }
                        else
                        {
                            //ROS_INFO("Node is FREE!");
                            col_pose.pose.position.x = x;
                            col_pose.pose.position.y = y;
                            col_pose.pose.position.z = z*octree_res;

                            check_position_colision(col_pose);          //check if there are colisions

                            if(pose_occupied == false)
                            {
                                tmp_ex_goals[0] = x;
                                tmp_ex_goals[1] = y;
                                tmp_ex_goals[2] = z*octree_res;
                                //ROS_INFO("Potential goal: %f %f %f", x, y, z*octree_res);
                            }else
                            {
                                skip_pose = true;
                            }
                        }
                        co_unknown = 0;
                    }else
                    {
                        //this cell is unknown, store old one
                        co_unknown++;

                        if(co_unknown > 6)
                        {
                            //ROS_INFO("Exploration: Found unknown area, define as end of RAY search!");
                            co_unknown = 0;
                            break;
                        }
                    }
                }

                if(skip_pose == false)
                {
                    //check if zero
                    if((tmp_ex_goals[0] != 0.0)&&(tmp_ex_goals[1] != 0.0)&&(tmp_ex_goals[2] != 0.0))
                    {
                        explore_pos[co_ex_poses*3 +0] = tmp_ex_goals[0];
                        explore_pos[co_ex_poses*3 +1] = tmp_ex_goals[1];
                        explore_pos[co_ex_poses*3 +2] = tmp_ex_goals[2];
                        co_ex_poses++;

                        //show result
                        if(show_markers == true)
                        {
                            col_pose.pose.position.x = tmp_ex_goals[0];
                            col_pose.pose.position.y = tmp_ex_goals[1];
                            col_pose.pose.position.z = tmp_ex_goals[2];

                            robot_colision_checking_pub(col_pose);      //show robot in a map
                            usleep(30000);
                        }
                    }
                }

            }
        }

        //FIND BEST POINT
        double smallest_dis = 1000;
        int mark_point = 0;
        bool found_point = false;
        for(int i = 0; i < co_ex_poses; i++)
        {
            global_x = explore_pos[i*3 +0];
            global_y = explore_pos[i*3 +1];
            global_z = explore_pos[i*3 +2];

            ROS_INFO("Exploration: Current considered pose: X: %f, Y: %f Z: %f", global_x, global_y, global_z);

            double rob_to_free_pose = sqrt(pow(global_x - curr_pose_x,2.0)+pow(global_y - curr_pose_y,2.0)+pow(global_z- curr_pose_z,2.0));

            //ROS_INFO("Exploration: Distance calculation for goal choosing: %f at %i pose %f %f %f", rob_to_free_pose, i, explore_pos[i*3 +0], explore_pos[i*3 +1],explore_pos[i*3 +2]);

            //check if goal is in the given world
            if((fabs(global_x) < max_x_world_size)&&(fabs(global_y) < max_y_world_size))
            {
                if(global_y < 1.2) //pazi !!!!!!!!!
                {

                    if((rob_to_free_pose < smallest_dis)&&(rob_to_free_pose > explore_min_range/2))
                    {
                        smallest_dis = rob_to_free_pose;
                        mark_point = i;

                        found_point = true;
                    }
                }
            }
        }

        if(found_point == true)
        {
            global_x = explore_pos[mark_point*3 +0];
            global_y = explore_pos[mark_point*3 +1];
            global_z = explore_pos[mark_point*3 +2];

            ROS_INFO("Exploration: AUTO selected GOAL point is: X: %f, Y: %f Z: %f", global_x, global_y, global_z);
        }else
        {
            start_exploration = false;
            ROS_WARN("Exploration: Consider your exploration parameters, because no free point was found!");
        }
    }

    void iri_miriamm_exploration::get_new_area_exploration_goal()
    {
        ROS_INFO("Exploration: get_new_area_exploration_goal START!");
        double explore_pos[3*500];
        int co_ex_poses = 0;
        start_exploration = true;

        //call new octomap
        call_octomap();
        ROS_INFO("Exploration: Updated Octomap Received!");

        //fist check goals reachable in Z
        int zn0 = explore_z_down_limit*(1/octree_res);
        int zn1 = (explore_z_up_limit*(1/octree_res))/2;

        //fail flags
        bool liftup_danger = false;
        bool no_free_pose = true;

        for(int zn=zn0; zn < zn1; zn++)
        {
            zn = zn + rob_col_z*(1/octree_res)/2; //not test every position

            //check for colisions
            geometry_msgs::PoseStamped col_pose;
            col_pose.pose.position.x = curr_pose_x;
            col_pose.pose.position.y = curr_pose_y;
            col_pose.pose.position.z = zn*octree_res;

            check_position_colision(col_pose);          //check if there are colisions

            if(pose_occupied == false)
            {
                //ROS_INFO("Robot Goal Destination is FREE for NOW!");

                //explore_pos[1] = curr_pose_x;
                //explore_pos[2] = curr_pose_y;
                //explore_pos[3] = zn*octree_res;

            }else
            {
                //ROS_INFO("Robot Goal Destination is OCCUPIED, please choose another position!");
                liftup_danger = true;
            }

            if(show_markers == true)
            {
                robot_colision_checking_pub(col_pose);      //show robot in a map
                usleep(100000);
            }
        }

        //limit search in Z
        octree->getMetricMax(xmax,ymax,zmax);
        octree->getMetricMin(xmin,ymin,zmin);

        ROS_INFO("Exploration: Octree MAX: %f %f %f --- MIN: %f %f %f", xmax,ymax,zmax, xmin,ymin,zmin);

        //limit z
        int zd0 = explore_z_down_limit*(1/octree_res);
        int zd1 = explore_z_up_limit*(1/octree_res);

        //limit radius
        double rad_max[4];
        rad_max[0] = sqrt(pow(xmax,2)+pow(ymax,2));
        rad_max[1] = sqrt(pow(xmin,2)+pow(ymin,2));
        rad_max[2] = sqrt(pow(xmax,2)+pow(ymin,2));
        rad_max[3] = sqrt(pow(xmin,2)+pow(ymax,2));

        //find largest radious
        int rad = 0;
        double rad_max_tmp = 0;
        for(int i = 0; i < 4; i++)
        {
            if(rad_max[i] > rad_max_tmp)
            {
                rad_max_tmp = rad_max[i];
                rad = rad_max[i];
            }
        }

        //check if is in the max range limit
        if(rad > explore_max_range)
        {
            rad = explore_max_range;
        }

//        double explore_range_min = rad - explore_min_range;

//        if(explore_range_min < explore_min_range)
//        {
//            explore_range_min = explore_min_range;
//        }

        //calculate robot current distce from base frame
        //int rob_dist = (sqrt(pow(curr_pose_x,2.0 + pow(curr_pose_y,2.0)))*(1/octree_res));

        int rd1 = round(rad*(1/octree_res));
        int rd0 = round(explore_min_range*(1/octree_res));

        ROS_INFO("Exploration: Maxsimum radius is: %i and min rad is: %i", rd1, rd0);

        double x, y;
        double tmp_ex_goals[3];
        int co_unknown = 0;
        bool skip_pose = false;
        geometry_msgs::PoseStamped col_pose;
        for(int z=zd0; z < zd1; z++)
        {
            z = z + rob_col_z*(1/octree_res)/2; //not test every position

            for (int an=0; an<361; an++)
            {

                an = an + (int)explore_an_shift;   //search angle
                skip_pose = false;

                //clear explore goals
                for(int r=rd0; r < rd1; r++)
                {
                    r = r + rob_col_x/2*colision_check_per_rob_size*(1/octree_res);

                    x = r*octree_res * cos(an*0.0174532925)+curr_pose_x;
                    y = r*octree_res * sin(an*0.0174532925)+curr_pose_y;

                    //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x, (double)y, (double)z*octree_res);

                    oc_node = octree->search((double)x, (double)y, (double)z*octree_res);

                    if(oc_node)
                    {
                        if(oc_node->getValue() > 0)
                        {
                            //oc_node->setValue(-2.0);
                            //oc_node->setColor(0, 255, 0);
                            //ROS_INFO("Node is OCCUPIED!");
                        }
                        else
                        {
                            //ROS_INFO("Node is FREE!");
                            col_pose.pose.position.x = x;
                            col_pose.pose.position.y = y;
                            col_pose.pose.position.z = z*octree_res;

                            check_position_colision(col_pose);          //check if there are colisions

                            if(pose_occupied == false)
                            {
                                ROS_INFO("Robot Goal Destination is FREE for NOW!");

                                tmp_ex_goals[0] = x;
                                tmp_ex_goals[1] = y;
                                tmp_ex_goals[2] = z*octree_res;

                                no_free_pose = false; 

                                ROS_INFO("Potential goal: %f %f %f", x, y, z*octree_res);

                            }else
                            {
                                //skip_pose = true;
                                //break;
                            }
                        }
                        co_unknown = 0;
                    }else
                    {
                        //this cell is unknown, store old one
                        co_unknown++;

                        if(co_unknown > 4)
                        {
                            //ROS_INFO("Exploration: Found unknown area, define as end of RAY search!");
                            co_unknown = 0;
                            //break;
                            //skip_pose = true;
                            break;
                        }
                    }
                }

                if(skip_pose == false)
                {
                    //mark the far away goal
                    explore_pos[co_ex_poses*3 +0] = tmp_ex_goals[0];
                    explore_pos[co_ex_poses*3 +1] = tmp_ex_goals[1];
                    explore_pos[co_ex_poses*3 +2] = tmp_ex_goals[2];
                    co_ex_poses++;

                    //show result
                    if(show_markers == true)
                    {
                        col_pose.pose.position.x = tmp_ex_goals[0];
                        col_pose.pose.position.y = tmp_ex_goals[1];
                        col_pose.pose.position.z = tmp_ex_goals[2];

                        robot_colision_checking_pub(col_pose);      //show robot in a map
                        usleep(30000);
                    }
                }
            }
        }

         ROS_INFO("Exploration: %i potential free canditetes founded!", co_ex_poses);

        //CHOOSE THE BEST GOAL POINT

        /*int go=1;
        int co_explore_poses = 0;
        while (go == 1)
        {
            //ROS_INFO("Exploration: Try to choose the best global goal!");
            double smallest_dis = 1000;
            int mark_point = 0;

            for(int i = 0; i < co_ex_poses; i++)
            {
                double rob_to_free_pose = sqrt(pow(explore_pos[i*3 +0] - curr_pose_x,2.0)+pow(explore_pos[i*3 +1] - curr_pose_y,2.0)+pow(explore_pos[i*3 +2]- curr_pose_z,2.0));

                //ROS_INFO("Exploration: Distance calculation for goal choosing: %f at %i pose %f %f %f", rob_to_free_pose, i, explore_pos[i*3 +0], explore_pos[i*3 +1],explore_pos[i*3 +2]);

                //find the closest free pose in space
                if(rob_to_free_pose > explore_min_range/2)
                {
                    if((rob_to_free_pose < smallest_dis)&&(rob_to_free_pose > explore_min_range))
                    {
                        smallest_dis = rob_to_free_pose;
                        mark_point = i;

                        way_choosed[explore_node_co][0] = co_explore_poses;
                    }

                    //make list of exploration history
                    exploration_poses_history[explore_node_co][co_explore_poses][0] = explore_pos[i*3 + 0];
                    exploration_poses_history[explore_node_co][co_explore_poses][1] = explore_pos[i*3 + 1];
                    exploration_poses_history[explore_node_co][co_explore_poses][2] = explore_pos[i*3 + 3];
                    co_explore_poses++;
                }
            }

            //make beter point choosing algorithem
            global_x = explore_pos[mark_point*3 +0];
            global_y = explore_pos[mark_point*3 +1];
            global_z = explore_pos[mark_point*3 +2];
            
            //make copy
            //double explore_pos_cp = explore_pos;
            double explore_pos_cp[3][500];
            memcpy(explore_pos_cp,explore_pos, sizeof(double)*3*500);

            ROS_INFO("Exploration: Selected exploration point is: %f %f %f choosed point is: %i Number of discovered points is: %i", global_x, global_y, global_z, mark_point, co_ex_poses);

            //check if the selected goal is in the defined world
            if((fabs(global_x) != 0.0)&&(fabs(global_z) != 0.0)&&(fabs(global_z) != 0.0))
            {
                if((fabs(global_x) < max_x_world_size)&&(fabs(global_y) < max_y_world_size))
                {
                    go = 0;
                    ROS_INFO("Exploration: Selected goal is in the given AREA!");
                    return;
                }
                else
                {
                    //copy to tmp
                    double explore_pos_tmp[3*co_ex_poses];
                    for(int i=0; i < co_ex_poses; i++)
                    {
                         explore_pos_tmp[i*3 +0] = explore_pos[i*3 +0];
                         explore_pos_tmp[i*3 +1] = explore_pos[i*3 +1];
                         explore_pos_tmp[i*3 +2] = explore_pos[i*3 +2];
                    }
                    //remove point from the list
                    int co_ex_new = -1;
                    for(int i=0; i < co_ex_poses; i++)
                    {
                        if(i == mark_point)
                        {
                            //remove point
                            ROS_INFO("Exploration: Removing POINT: %i", i);
                        }else
                        {
                            co_ex_new++;
                            explore_pos[co_ex_new*3 +0] = explore_pos_tmp[i*3 +0];
                            explore_pos[co_ex_new*3 +1] = explore_pos_tmp[i*3 +1];
                            explore_pos[co_ex_new*3 +2] = explore_pos_tmp[i*3 +2];
                        }
                    }

                    co_ex_poses = co_ex_new;

                    if(co_ex_poses < 3)
                    {
                        ROS_INFO("Exploration: Exploration has finished on the area of X: %f to %f / Y: %f to %f", max_x_world_size, -max_x_world_size, max_y_world_size, -max_y_world_size);
                        go = 0;
                        start_exploration = false;
                        return;
                    }

                    //choose random point
                    ROS_INFO("Exploration: Interested point has been removed from list, list size: %i", co_ex_poses);
                    usleep(1000);
                }
            }else
            {
                ROS_INFO("Exploration: Selected goal is at 0 0 0!");

                start_exploration = false;
                break;
            }
        }
        */
        //add node to history
        explore_node_co++;

        double smallest_dis = 1000;
        int mark_point = 0;

        for(int i = 0; i < co_ex_poses; i++)
        {
            global_x = explore_pos[i*3 +0];
            global_y = explore_pos[i*3 +1];
            global_z = explore_pos[i*3 +2];

            ROS_INFO("X: %f, Y: %f Z: %f", global_x, global_y, global_z);

            double rob_to_free_pose = sqrt(pow(explore_pos[i*3 +0] - curr_pose_x,2.0)+pow(explore_pos[i*3 +1] - curr_pose_y,2.0)+pow(explore_pos[i*3 +2]- curr_pose_z,2.0));

            //ROS_INFO("Exploration: Distance calculation for goal choosing: %f at %i pose %f %f %f", rob_to_free_pose, i, explore_pos[i*3 +0], explore_pos[i*3 +1],explore_pos[i*3 +2]);

            if((fabs(explore_pos[i*3 +0]) < max_x_world_size)&&(fabs(explore_pos[i*3 +1]) < max_y_world_size))
            {

                //find the closest free pose in space
                if((global_x != 0.0)&&(global_y != 0.0)&&(global_z != 0.0))
                {

                    if((rob_to_free_pose < smallest_dis)&&(rob_to_free_pose > explore_min_range/2))
                    {
                        smallest_dis = rob_to_free_pose;
                        mark_point = i;

                        //way_choosed[explore_node_co][0] = co_explore_poses;
                    }
                }

                    //make list of exploration history
//                    exploration_poses_history[explore_node_co][co_explore_poses][0] = explore_pos[i*3 + 0];
//                    exploration_poses_history[explore_node_co][co_explore_poses][1] = explore_pos[i*3 + 1];
//                    exploration_poses_history[explore_node_co][co_explore_poses][2] = explore_pos[i*3 + 3];
                    //co_explore_poses++;

            }
        }

        //make beter point choosing algorithem
        global_x = explore_pos[mark_point*3 +0];
        global_y = explore_pos[mark_point*3 +1];
        global_z = explore_pos[mark_point*3 +2];


        ROS_INFO("Exploration: Robot AUTO GOAL Position: X: %f / Y: %f / Z: %f", global_x, global_y, global_z);

        if ((liftup_danger == true)&&(no_free_pose == true))
        {
            //stop exploration
            start_exploration = false;
            ROS_INFO("Exploration: Robot is LANDED and there is no FREE POSE FOUND!");
        }

        //if robot get stucked use
        // -- way_choosed
        // -- exploration_poses_history
    }
    /*****************************************************************************************************************
     * Pose update
     */
    void iri_miriamm_exploration::pose_update(const geometry_msgs::PoseStamped pose)
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
     * Autonomous explore
     */
    void iri_miriamm_exploration::explore_auto()
    {
        geometry_msgs::PoseStamped new_goal;

        if(start_exploration == true)
        {
            //check if reached a goal
            double goal_x = fabs(curr_pose_x - global_x);
            double goal_y = fabs(curr_pose_y - global_y);
            double goal_z = fabs(curr_pose_z - global_z);

            //ROS_INFO("Exploration: Exploration true! diff val: %f %f %f", goal_x, goal_y, goal_z);

            if((goal_x < goal_offset)&&(goal_y < goal_offset)&&(goal_z < goal_offset))
            {
                ROS_INFO("Exploration: New goal call has been made!");

                //call for new goal
                //if(exploration_strategy == string("closest_goal"))
                //{
                    //get_new_area_exploration_goal();
                    sleep(2);
                    get_new_frontear_goal();
                    ROS_INFO("Exploration: New goal has been choosed!");

                //}

                if(start_exploration == false)
                {
                    ROS_ERROR("Exploration: HAS BEEN STOPED DUE TO LIFTOFF DANGER OR NO FREE POSE AVALIABLE");
                    //publish new goal
                    geometry_msgs::PoseStamped new_goal;
                    new_goal.pose.position.x = 0;
                    new_goal.pose.position.y = 0;
                    new_goal.pose.position.z = -100;
                    new_goal_pub.publish(new_goal);

                }else
                {
                    //publish new goal
                    new_goal.pose.position.x = global_x;
                    new_goal.pose.position.y = global_y;
                    new_goal.pose.position.z = global_z;
                    new_goal_pub.publish(new_goal);
                    ROS_INFO("Exploration: Robot pose Marker Published!");

                }
            }
        }
    }
    /*****************************************************************************************************************
     * Publish marker on position
     */
    void iri_miriamm_exploration::robot_colision_checking_pub(const geometry_msgs::PoseStamped pose)
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
     * Octomap callback
     */
    void iri_miriamm_exploration::call_octomap()
    {
        //ROS_INFO("Exploration: octomap callback");
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


            co_octomap++;
            if(co_octomap == 1)
            {
                octree_res = octree->getResolution();
                //ROS_INFO("Tree resolution: %f Octree size: %i", octree_res, octree->size());

                double x,y,z;
                octree->getMetricSize(x, y, z);
                //ROS_INFO("Tree metric size: x: %f y: %f z: %f", x,y,z);
            }
        }
    }
    /*****************************************************************************************************************
     * Check if position is covering octomap
     */
    void iri_miriamm_exploration::check_position_colision(const geometry_msgs::PoseStamped pose)
    {
        //defina area search
        double octree_res = octree->getResolution();
        int zd0 = (pose.pose.position.z - rob_col_z/2)*(1/octree_res);
        int zd1 = (pose.pose.position.z + rob_col_z/2)*(1/octree_res);
        pose_occupied = false;

        double x, y;
        for(int z=zd0; z < zd1; z++)
        {
            //ROS_INFO("Z value: %i", z);
            for (int an=0; an<361; an++)
            {
                for(int rad=1; rad < round(rob_col_x*(1/octree_res)); rad++)
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
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "iri_miriamm_exploration");
    ros::NodeHandle nh;

    //for registering the node
    iri_miriamm_exploration::iri_miriamm_exploration iri_miriamm_exploration_handler(nh);

    iri_miriamm_exploration_handler.init();                     //initize

    ros::Rate loop_rate(iri_miriamm_exploration_handler.explore_rate);
    while (ros::ok())
    {
        iri_miriamm_exploration_handler.explore_auto();

        ros::spinOnce();

        loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
