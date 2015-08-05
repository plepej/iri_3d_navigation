/** \brief
 *
 *  \file iri_miriamm_path_follower.cpp
 *  \author Peter Lepej
 *  \date 5.05.2015
 *  \version 1.0
 */

#include </home/peter/ros/catkin_ws/src/iri_miriamm_path_follower/include/iri_miriamm_path_follower/iri_miriamm_path_follower.hpp>

namespace iri_miriamm_path_follower
{
    /****************************************************************
     * Here we can set up the parameters
     */
    iri_miriamm_path_follower::iri_miriamm_path_follower(ros::NodeHandle nh)    //constructor for the class
    : nh_(nh),
      take_off_client_("take_off", true),
      waypoints_client_("waypoints", true),
      land_client_("land", true)
    {

        ros::NodeHandle private_nh("~");

        //general
        private_nh.param<std::string>("global_frame", global_frame, string("/world"));
        private_nh.param<std::string>("pose_update_topic", pose_update_topic, string("/odom_fake"));
        private_nh.param<std::string>("octomap_topic", octomap_topic, string("/octomap_full"));
        private_nh.param<bool>("show_markers", show_markers, true);

        private_nh.param<bool>("use_vel_commands", use_vel_commands, true);
        private_nh.param<bool>("use_waypoint_commands", use_waypoint_commands, false);
        private_nh.param<bool>("use_stop_before_action", use_stop_before_action, false);

        //path follower
        private_nh.param<std::string>("exploration_path_topic", exploration_path_topic, string("/global_path"));
        private_nh.param<std::string>("local_path_topic", local_path_topic, string("/local_path"));
        private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, string("/cmd_vel"));
        private_nh.param<double>("max_lin_velocity", max_lin_velocity, 0.3);
        private_nh.param<double>("max_rot_velocity", max_rot_velocity, 0.2);
        private_nh.param<double>("goal_offset", goal_offset, 0.15);
        private_nh.param<double>("desired_altitude", liftoff_altitude_limit, 0.25);

        //collision avoidance
        private_nh.param<double>("lookahead_distance", lookahead_distance, 0.3);
        private_nh.param<double>("raycasting_net_raster", raycasting_net_raster, 0.20);
        private_nh.param<double>("reactive_const_potential_field", reactive_const_pf, 0.5);
        private_nh.param<double>("reactive_const_way_potential_field", reactive_const_way_pf, 0.5);

        private_nh.param<int>("update_cmd_vel_freq", update_cmd_vel_freq, 100);
        private_nh.param<double>("update_waypoints_freq", update_waypoints_freq, 1);

        //robot
        private_nh.param<double>("robot_colision_x", rob_col_x, 1.0);
        private_nh.param<double>("robot_colision_y", rob_col_y, 1.0);
        private_nh.param<double>("robot_colision_z", rob_col_z, 0.5);

    }
    /****************************************************************
     *
     */
    iri_miriamm_path_follower::~iri_miriamm_path_follower()
    {
    }
    /*****************************************************************************************************************
     * Initialization
     */
    void iri_miriamm_path_follower::init()
    {
        //robot pose subscriber
                                                                                    //pose update rate!!!
        pose_update_sub = nh_.subscribe<geometry_msgs::PoseStamped>(pose_update_topic ,1 , &iri_miriamm_path_follower::pose_update, this);

        //subscrite to octomap
        octomap_sub = nh_.subscribe<octomap_msgs::Octomap>(octomap_topic, 1, &iri_miriamm_path_follower::octomap_callback, this);

        if(show_markers)
        {
            //publish robot colision marker
            colision_marker_pub = nh_.advertise<visualization_msgs::Marker>("obstacle_box_marker", 1);

            //collison checkin marker
            colision_start_pub = nh_.advertise<visualization_msgs::Marker>("raycasting_start_marker", 1);
            colision_end_pub = nh_.advertise<visualization_msgs::Marker>("raycasting_end_marker", 1);

            //raycasting collison publiser
            raycasting_collision_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("raycasting_colision_marker", 1);
        }

        if(use_waypoint_commands == true)
        {
            local_path_pub = nh_.advertise<nav_msgs::Path>(local_path_topic, 1);
        }

        //subscribe to robot new path
        path_sub = nh_.subscribe<nav_msgs::Path>(exploration_path_topic, 1, &iri_miriamm_path_follower::path_callback, this);

        //cmd vel publisher
        cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

        //planned path history
        planned_path_pub = nh_.advertise<nav_msgs::Path>("/planned_path_full", 1);

        //reactive planned path history
        planned_path_reactive_pub = nh_.advertise<nav_msgs::Path>("/planned_path_full_reactive", 1);

        //set global goal default position
        global_x = 0.0;
        global_y = 0.0;
        global_z = 0.0;

        start_path_following = false;
        start_following = false;

        old_yaw_diff = -100;

        alignment_mode = false;

        co_al = 0;

        goal_dist = 0;

        co_cin = 0;

        count_sphere_marker = 0;

        pf_force_vel_x = 0;
        pf_force_vel_y = 0;
        pf_force_vel_z = 0;

        lock_x_vel = false;
        lock_y_vel = false;
        lock_z_vel = false;
        co_octomap = 0;

        //state machine start params
        plan_state_ = STOPPED;
        task_state_ = LANDED;

        plan_state = "LANDED";

        //set initial lookahead distance
        lookahead_local = lookahead_distance + lookahead_distance*max_lin_velocity;

        //init path for local
         planned_path_reactive.header.frame_id = global_frame;
         planned_path_reactive.poses.clear();
    }
    /*****************************************************************************************************************
     * octomap callback
     */
    void iri_miriamm_path_follower::octomap_callback(const octomap_msgs::Octomap octo)
    {
        //call octomap update
        //call_octomap();

        if (start_path_following == true)
        {

            //ROS_INFO("Path Following: Octomap received");

            AbstractOcTree* tree = octomap_msgs::msgToMap(octo);
            octree = dynamic_cast<octomap::OcTree*>(tree);

            octree_res = octree->getResolution();
            //ROS_INFO("Path Follower::  Octomap callback - Tree resolution: %f Octree size: %i", octree_res, octree->size());
            double x,y,z;
            octree->getMetricSize(x, y, z);
            //ROS_INFO("Tree metric size: x: %f y: %f z: %f", x,y,z);

            calculate_pf();
        }
    }

    /*****************************************************************************************************************
     * Calculate potential field repulsive-attractive
     */
    void iri_miriamm_path_follower::calculate_pf()
    {

        //ROS_INFO("Path Follower: Potential field calc started!");
        /// 1. Based on robot position check area based on lookahead distance
        /// 2. Get robot position potintial field forces in XYZ and relative to distance
        /// 3. Calculate PF based on my speed
        /// 4. Add vel to /cmd_vel

//        int yd0 = (curr_pose_y - lookahead_distance)*(1/octree_res);
//        int yd1 = (curr_pose_y + lookahead_distance)*(1/octree_res);
//        int xd0 = (curr_pose_x - lookahead_distance)*(1/octree_res);
//        int xd1 = (curr_pose_x + lookahead_distance)*(1/octree_res);

        /*
        for(int z=zd0; z < zd1; z++)
        {
            for(int y=yd0; y < yd1; y++)
            {
                 //ROS_INFO("Search Y: %f", (double)y*octree_res);
                for(int x=xd0; x < xd1; x++)
                {
                    oc_node = octree->search((double)x*octree_res, (double)y*octree_res, (double)z*octree_res);
                    //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x*octree_res, (double)y*octree_res, (double)z*octree_res);

                    if(oc_node)
                    {
                        if(oc_node->getValue() > 0)
                        {
                            //ROS_INFO("Node is occupied!");
                            co_occ++;
                            sum_pf_x = sum_pf_x + (curr_pose_x - (double)x*octree_res);
                            sum_pf_y = sum_pf_y + (curr_pose_y - (double)y*octree_res);
                            sum_pf_z = sum_pf_z + (curr_pose_z - (double)z*octree_res);

                            hit_distance = sqrt(pow((double)x*octree_res - curr_pose_x,2.0)+pow((double)y*octree_res - curr_pose_y,2.0)+pow((double)z*octree_res - curr_pose_z,2.0));
                            sum_pf_avg = sum_pf_avg + hit_distance;

                        }
                    }
                }
            }
        }

        */

        //do search based on lookahead distance and robot pose
        int zd0 = (curr_pose_z - rob_col_z/2)*(1/octree_res);
        int zd1 = (curr_pose_z + rob_col_z/2)*(1/octree_res);
        if(zd0 < 1)
        {
            zd0 = 1;
        }

        //set radius
        double rad = lookahead_distance;
        int rd1 = round(rad*(1/octree_res));
        int rd0 = round(rob_col_x/2*(1/octree_res));
        double x, y;

        //ROS_INFO("Path Follower: Z: %i %i, R: %i %i", zd0, zd1, rd0, rd1);

        //init pf variables
        double sum_pf_x = 0;
        double sum_pf_y = 0;
        double sum_pf_z = 0;
        double sum_pf_avg = 0;
        int co_occ = 0;
        double hit_distance = 0;

        for(int z=zd0; z < zd1; z++)
        {
            for (int an=0; an<361; an++)
            {
                an++; // 2 deg search
                for(int r=rd0; r < rd1; r++)
                {
                    x = r*octree_res * cos(an*0.0174532925)+curr_pose_x;
                    y = r*octree_res * sin(an*0.0174532925)+curr_pose_y;

                    oc_node = octree->search(x, y, (double)z*octree_res);
                    //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x*octree_res, (double)y*octree_res, (double)z*octree_res);

                    if(oc_node)
                    {
                        if(oc_node->getValue() > 0)
                        {
                            //ROS_INFO("Node is occupied!");
                            co_occ++;
                            sum_pf_x = sum_pf_x + (curr_pose_x - x);
                            sum_pf_y = sum_pf_y + (curr_pose_y - y);
                            sum_pf_z = sum_pf_z + (curr_pose_z - (double)z*octree_res);

                            hit_distance = sqrt(pow((double)x*octree_res - curr_pose_x,2.0)+pow((double)y*octree_res - curr_pose_y,2.0)+pow((double)z*octree_res - curr_pose_z,2.0));
                            sum_pf_avg = sum_pf_avg + hit_distance;

                        }
                    }
                }
            }
        }

        if(co_occ > 2)
        {
            //calculate avg forces
            avgpf_force_x = sum_pf_x/co_occ;
            avgpf_force_y = sum_pf_y/co_occ;
            avgpf_force_z = sum_pf_z/co_occ;
            avgpf_force = sum_pf_avg/co_occ;
            
            if(use_vel_commands == true)
            {
                //determine vel signs
                double close_diff_x = avg_force_x - curr_pose_x;
                double close_diff_y = avg_force_y - curr_pose_y;

                double sign_vel_x, sign_vel_y, sign_vel_z;

                //define signs
                sign_vel_z = avgpf_force_z/fabs(avgpf_force_z);

                if((close_diff_x < 0)&&(close_diff_y < 0))
                {
                    sign_vel_x = avgpf_force_x/fabs(avgpf_force_x);
                    sign_vel_y = avgpf_force_y/fabs(avgpf_force_y);

                    //ROS_INFO("FORCES DIRECTIONS ARE _+I+_");
                }
                if((close_diff_x < 0)&&(close_diff_y > 0))
                {
                    sign_vel_x = avgpf_force_x/fabs(avgpf_force_x);
                    sign_vel_y = avgpf_force_y/fabs(avgpf_force_y);

                    //ROS_INFO("FORCES DIRECTIONS ARE _+I-_");
                }
                if((close_diff_x > 0)&&(close_diff_y > 0))
                {
                    sign_vel_x = -avgpf_force_x/fabs(avgpf_force_x);
                    sign_vel_y = -avgpf_force_y/fabs(avgpf_force_y);

                    //ROS_INFO("FORCES DIRECTIONS ARE _-I-_");
                }
                if((close_diff_x > 0)&&(close_diff_y < 0))
                {
                    sign_vel_x = -avgpf_force_x/fabs(avgpf_force_x);
                    sign_vel_y = -avgpf_force_y/fabs(avgpf_force_y);

                    //ROS_INFO("FORCES DIRECTIONS ARE _-I+_");
                }

                if(avg_hit_distance > lookahead_local)
                {
                    avg_hit_distance = lookahead_local;
                }

                pf_force_vel_x = reactive_const_pf*sign_vel_x*(max_lin_velocity/(lookahead_local/(lookahead_local - fabs(avgpf_force) + rob_col_x/2)));
                pf_force_vel_y = reactive_const_pf*sign_vel_y*(max_lin_velocity/(lookahead_local/(lookahead_local - fabs(avgpf_force) + rob_col_y/2)));
                pf_force_vel_z = reactive_const_pf*sign_vel_z*(max_lin_velocity/(lookahead_local/(lookahead_local - fabs(avgpf_force) + rob_col_z/2)));

                //ROS_INFO("Path Follower: Reactive FORCES: x: %f, y: %f, : %f, AVG: %f hits: %i", avgpf_force_x, avgpf_force_y, avgpf_force_z, avgpf_force, co_occ);
            }

            if(use_waypoint_commands == true)
            {
                if(fabs(avgpf_force_x) > 0.1)
                {
                    avgpf_force_x = lookahead_distance/(avgpf_force_x*10);
                }else
                {
                    avgpf_force_x = 0;
                }

                if(fabs(avgpf_force_y) > 0.1)
                {
                    avgpf_force_y = lookahead_distance/(avgpf_force_y*10);
                }else
                {
                    avgpf_force_y = 0;
                }

                if(fabs(avgpf_force_z) > 0.1)
                {
                    avgpf_force_z = lookahead_distance/(avgpf_force_z*10);
                }else
                {
                    avgpf_force_z = 0;
                }
            }

        }else
        {
            //ROS_INFO("Path Follower: No repulsive forces are in lookahead area!");
            avgpf_force_x = 0;
            avgpf_force_y = 0;
            avgpf_force_z = 0;
            avgpf_force = 0;
        }


    }
    /*****************************************************************************************************************
     * Path callback
     */
    void iri_miriamm_path_follower::path_callback(const nav_msgs::Path path)
    {
        ROS_INFO("Path Follower: New path received!");

//        for(int i=0; i < path.poses.size(); i++)
//        {
//            ROS_INFO("Path Follower: Path pose no %i at x: %f y: %f z: %f:", i, path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
//        }

        if(use_vel_commands == true)
        {
            //stop robot
            geometry_msgs::Twist cmd_vel;

            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;

            //publish
            cmd_vel_pub.publish(cmd_vel);
        }

        //start following
        start_path_following = true;
        start_following = true;

        global_path = path;

        //reset conuter for stopping the actions
        co_cin = 0;

    }
    /*****************************************************************************************************************
     * Vel command controll
     */
    void iri_miriamm_path_follower::vel_command()
    {
        //find closest point to current position
        closest_point = closest_point_to_curr_pos();

        //get differences in metric -- chose correct vel directions
        double diff_x, diff_y, diff_z;
        diff_x = closest_point.pose.position.x - curr_pose_x;
        diff_y = closest_point.pose.position.y - curr_pose_y;
        diff_z = closest_point.pose.position.z - curr_pose_z;

        //ROS_INFO("Out: yaw_x: %f, yaw y: %f", diff_x, diff_y);

        double vel_sign_z;
        if(diff_z > 0)
        {
            vel_sign_z = 1;
        }else
        {
            vel_sign_z = -1;
        }

        //bigest difference is
        double biggest;
        double big;
        if(fabs(diff_x) > fabs(diff_y))
        {
            biggest = fabs(diff_x);
            big = diff_x;
        }else
        {
            biggest = fabs(diff_y);
            big = diff_y;
        }

        if(fabs(diff_z) > biggest)
        {
            biggest = fabs(diff_z);
            big = diff_z;
        }

        biggest = fabs(big);

        //calculate heading angles
        double h_an_x = atan2(diff_z, diff_y);
        double h_an_y = atan2(diff_z, diff_x);
        double h_an_z = atan2(diff_y, diff_x)-yaw;

        //limit angles
        if(h_an_x > PI)
        {
            h_an_x = h_an_x - 2*PI;
        }

        if(h_an_x < -PI)
        {
            h_an_x = h_an_x + 2*PI;
        }

        if(h_an_y > PI)
        {
            h_an_y = h_an_y - 2*PI;
        }

        if(h_an_y < -PI)
        {
            h_an_y = h_an_y + 2*PI;
        }

        if(h_an_z > PI)
        {
            h_an_z = h_an_z - 2*PI;
        }

        if(h_an_z < -PI)
        {
            h_an_z = h_an_z + 2*PI;
        }

        global_desired_an_z = h_an_z;
        global_desired_an_y = h_an_y;
        global_desired_an_x = h_an_x;

        //calculate heading for goal
        double diff_gx = global_path.poses[global_path.poses.size()-1].pose.position.x - curr_pose_x;
        double diff_gy = global_path.poses[global_path.poses.size()-1].pose.position.y - curr_pose_y;
        double diff_gz = global_path.poses[global_path.poses.size()-1].pose.position.z - curr_pose_z;

        //calculate heading angles
        double h_an_gz = atan2(diff_gy, diff_gx) - yaw;
        double h_an_gz_sec = yaw  - atan2(diff_gy, diff_gx);

        //find the shortest path to align
        if(fabs(h_an_gz) > fabs(h_an_gz_sec))
        {
            h_an_gz = h_an_gz_sec;
        }
        //calculate goal proximity and reduce speed when aproaching
        double goal_prox = 1;
        double goal_diff= lookahead_distance - sqrt(pow(diff_gx,2.0)+pow(diff_gy,2.0)+pow(diff_gz,2.0));

        if(goal_diff > 0)
        {
            goal_prox = fabs((lookahead_distance - goal_diff)/lookahead_distance);
            ROS_INFO("Goal prox: %f", goal_prox);
        }

        //ROS_INFO("Path Follower: Global goal heading angle: %f", h_an_gz*57.2957795);

        //ROS_INFO("Calculated HEADING ANGLES: x: %f, y: %f, z: %f", h_an_x*57.2957795, h_an_y*57.2957795, h_an_z*57.2957795);

        //cmd_vel.linear.x = max_lin_velocity*(cos(h_an_x)*cos(h_an_y) - sin(h_an_x)*cos(h_an_z) + cos(h_an_x)*sin(h_an_y)*sin(h_an_z) + sin(h_an_x)*sin(h_an_z) + cos(h_an_x)*sin(h_an_y)*cos(h_an_z));
        cmd_vel.linear.x = goal_prox*(max_lin_velocity*(cos(h_an_z)) + pf_force_vel_x);
        //cmd_vel.linear.x = max_lin_velocity*(cos(h_an_x)*cos(h_an_y) - sin(h_an_x)*sin(h_an_z)*sin(h_an_y) -cos(h_an_x)*sin(h_an_z) + cos(h_an_z)*sin(h_an_y) + cos(h_an_y)*sin(h_an_x)*sin(h_an_z));
        //cmd_vel.linear.y = max_lin_velocity*(sin(h_an_x)*cos(h_an_y) + cos(h_an_x)*cos(h_an_z) + sin(h_an_x)*sin(h_an_y)*sin(h_an_z) - cos(h_an_x)*sin(h_an_z) + sin(h_an_x)*sin(h_an_y)*cos(h_an_z));
        cmd_vel.linear.y = goal_prox*(max_lin_velocity*(sin(h_an_z)) + pf_force_vel_y);
        //cmd_vel.linear.y = max_lin_velocity*(cos(h_an_y)*sin(h_an_z) + cos(h_an_z)*sin(h_an_x)*sin(h_an_y) + cos(h_an_x)*cos(h_an_z) + sin(h_an_z)*sin(h_an_y)-cos(h_an_z)*cos(h_an_y)*sin(h_an_x));
        //cmd_vel.linear.z = max_lin_velocity*(-sin(h_an_y) + cos(h_an_y)*sin(h_an_z) + cos(h_an_y)*cos(h_an_z));
        //cmd_vel.linear.z = vel_sign_z*(fabs(diff_z)*max_lin_velocity)/biggest;
        cmd_vel.linear.z = goal_prox*(vel_sign_z*(fabs(diff_z)*max_lin_velocity)/biggest + pf_force_vel_z); //sin(h_an_y);

        //cmd_vel.linear.z = max_lin_velocity*(-cos(h_an_x)*sin(h_an_y) + sin(h_an_x) + cos(h_an_x)*cos(h_an_y));

        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = max_rot_velocity*(h_an_gz); //max_rot_velocity*(sin(h_an_z)/cos(h_an_y) + cos(h_an_z)/cos(h_an_y));

        //check if robot is goig to hit the ground
        double alt_diff = curr_pose_z - liftoff_altitude_limit;
        bool active_ground = false;
        if(curr_pose_z > liftoff_altitude_limit)
        {
            active_ground = true;
        }

        if((alt_diff < 0)&&(active_ground == true))
        {
            //start reducing Z vel
            cmd_vel.linear.z = 0;
            ROS_ERROR("Path Follower: Reducing vel to: %f", pf_force_vel_z);

            //lock this axes
        }

//            //ckeck if any vel axes is locked
//            if (lock_x_vel == true)
//            {
//                ROS_ERROR("Path Follower: Locked X vel AXES!");
//                cmd_vel.linear.x = 0;
//            }
//            if (lock_y_vel == true)
//            {
//                ROS_ERROR("Path Follower: Locked Y vel AXES!");
//                cmd_vel.linear.y = 0;
//            }
//            if (lock_z_vel == true)
//            {
//                ROS_ERROR("Path Follower: Locked Z vel AXES!");
//                cmd_vel.linear.z = 0;
//            }

//            //check how low is robot
//            if(curr_pose_z < liftoff_altitude_limit)
//            {
//                ROS_ERROR("Path Follower: Robot is CLOSE to GROUND, disable Z vel!");
//                cmd_vel.linear.z = cmd_vel.linear.z+max_lin_velocity*0.2;
//            }

        ROS_INFO("Calculated VELOCITIES: x: %f, y: %f, z: %f -- angular_vel: %f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, cmd_vel.angular.z);

        //check if it is close to goal
        int max_path_size = global_path.poses.size()-1;
        double diff_gxe = curr_pose_x - global_path.poses[max_path_size].pose.position.x;
        double diff_gye = curr_pose_y - global_path.poses[max_path_size].pose.position.y;
        double diff_gze = curr_pose_z - global_path.poses[max_path_size].pose.position.z;

        if((fabs(diff_gxe) < goal_offset)&&(fabs(diff_gye) < goal_offset)&&(fabs(diff_gze) < goal_offset))
        {
            ROS_INFO("Path Follower: Stop the robot, he has reached Global Goal!");
            start_path_following = false;

            //save full path and publish
            planned_path_full.header.frame_id = global_frame;
            int old_ful_path_size = planned_path_full.poses.size();
            nav_msgs::Path old_full_path = planned_path_full;
            planned_path_full.poses.resize(global_path.poses.size() + planned_path_full.poses.size());
            ROS_INFO("Path Follower: Resized full path size: %i", planned_path_full.poses.size());

            for(int i=0; i < old_ful_path_size; i++)
            {
                    //add new pose
                planned_path_full.poses[i].pose.position.x = old_full_path.poses[i].pose.position.x;
                planned_path_full.poses[i].pose.position.y = old_full_path.poses[i].pose.position.y;
                planned_path_full.poses[i].pose.position.z = old_full_path.poses[i].pose.position.z;
            }

            int co_new = 0;
            for(int i=old_ful_path_size; i < old_ful_path_size + global_path.poses.size(); i++)
            {
                    //add new pose
                planned_path_full.poses[i].pose.position.x = global_path.poses[co_new].pose.position.x;
                planned_path_full.poses[i].pose.position.y = global_path.poses[co_new].pose.position.y;
                planned_path_full.poses[i].pose.position.z = global_path.poses[co_new].pose.position.z;
                co_new++;
            }

            ROS_INFO("Path Follower: Path successfull added to log!");

            planned_path_pub.publish(planned_path_full);
        }

        if(start_path_following == false)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = 0;
        }

        if((lock_x_vel == true)&&(lock_y_vel == true)&&(lock_z_vel == true))
        {
            ROS_ERROR("Path Follower: ALL VEL AXES HAVE BEEN LOCKED - ROBOT IS STUCKED, CAN NOT FOLLOW DESIGNATED PATH!!!!");
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.z = 0;

            start_path_following = false;
        }

        //publish
        cmd_vel_pub.publish(cmd_vel);
    }
    /*****************************************************************************************************************
     * Use waypoints controll
     */
    void iri_miriamm_path_follower::use_waypoints()
    {

        //ROS_INFO("Path Follower: Calculate Waypoints for SM!");
        local_path.header.frame_id = global_path.header.frame_id;
        local_path.poses.resize(global_path.poses.size());

        //find closest point to current position
        closest_point = closest_point_to_curr_pos();
        //closest point posision closest_point_pos

        int local_path_size = local_path.poses.size();

        if(local_path_size > 1)
        {

            local_path.poses[0].pose.position.x = curr_pose_x;
            local_path.poses[0].pose.position.y = curr_pose_y;
            local_path.poses[0].pose.position.z = curr_pose_z;

            //ROS_INFO("Path Follower: Closest Point: %i", closest_point_pos);

            int co_new_pose = 1;
            for(int i = closest_point_pos+1; i < global_path.poses.size(); i++)
            {
                //check if point is in lookahead distane
                double to_point = sqrt(pow(global_path.poses[i].pose.position.x - curr_pose_x,2.0)+pow(global_path.poses[i].pose.position.y - curr_pose_y,2.0)+pow(global_path.poses[i].pose.position.z - curr_pose_z,2.0));

                //the potiential field force must be applied
                if(to_point < lookahead_distance)
                {
                    //ROS_INFO("Path Follower: current changed pose: %i", co_new_pose);
                    local_path.poses[co_new_pose].pose.position.x = global_path.poses[i].pose.position.x + avgpf_force_x*reactive_const_way_pf;
                    local_path.poses[co_new_pose].pose.position.y = global_path.poses[i].pose.position.y + avgpf_force_y*reactive_const_way_pf;
                    local_path.poses[co_new_pose].pose.position.z = global_path.poses[i].pose.position.z + avgpf_force_z*reactive_const_way_pf;
                }
                else
                {
                    local_path.poses[co_new_pose].pose.position.x = global_path.poses[i].pose.position.x;
                    local_path.poses[co_new_pose].pose.position.y = global_path.poses[i].pose.position.y;
                    local_path.poses[co_new_pose].pose.position.z = global_path.poses[i].pose.position.z;

                }

                if(local_path.poses[co_new_pose].pose.position.z < liftoff_altitude_limit)
                {
                    local_path.poses[co_new_pose].pose.position.z = liftoff_altitude_limit;
                }

                co_new_pose++;

                //ROS_INFO("Path Followe: Current pose: %f %f %f", global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y, global_path.poses[i].pose.position.z);
            }

            //add last point
            int last_p = global_path.poses.size() -1;
            local_path.poses[co_new_pose].pose.position.x = global_path.poses[last_p].pose.position.x;
            local_path.poses[co_new_pose].pose.position.y = global_path.poses[last_p].pose.position.y;
            local_path.poses[co_new_pose].pose.position.z = global_path.poses[last_p].pose.position.z;
            co_new_pose++;

            //check if robot wants to land!!
            global_x = global_path.poses[last_p].pose.position.x;
            global_y = global_path.poses[last_p].pose.position.y;
            global_z = global_path.poses[last_p].pose.position.z;

            //check if we need to start landing procedure
            if((global_x == 0.0)&&(global_y == 0.0)&&(global_z == 0.5))
            {
                ROS_ERROR("Path Follower: START LANDING PROCEDURE FLAG SET UP!");
                initiate_landing = true;
                start_path_following = false;
            }
            else
            {
                initiate_landing = false;
            }

            global_x = 0;
            global_y = 0;
            global_z = 0;

            //ROS_INFO("Path Follower: End of adding pf");
            //int new_size = global_path.poses.size() - closest_point_pos + 1;
            local_path.poses.resize(co_new_pose);

        }
        else
        {
            local_path.poses.resize(2);

            //fist point
            local_path.poses[0].pose.position.x = curr_pose_x;
            local_path.poses[0].pose.position.y = curr_pose_y;
            local_path.poses[0].pose.position.z = curr_pose_z;

            local_path.poses[1].pose.position.x = curr_pose_x;
            local_path.poses[1].pose.position.y = curr_pose_y;
            local_path.poses[1].pose.position.z = curr_pose_z;

            //set two same points fist one is negleated and second one is the current goal pose
            ROS_INFO("Path Follower; The goal pose CURRENT position has been set to STOP THE ROBOT!");

        }

        //check if it is close to goal
        int max_path_size = global_path.poses.size()-1;
        double diff_gxe = curr_pose_x - global_path.poses[max_path_size].pose.position.x;
        double diff_gye = curr_pose_y - global_path.poses[max_path_size].pose.position.y;
        double diff_gze = curr_pose_z - global_path.poses[max_path_size].pose.position.z;

        if((fabs(diff_gxe) < goal_offset)&&(fabs(diff_gye) < goal_offset)&&(fabs(diff_gze) < goal_offset))
        {
            ROS_INFO("Path Follower: Stop the robot, he has reached Global Goal!");
            start_path_following = false;

            //save full path and publish
            planned_path_full.header.frame_id = global_frame;
            int old_ful_path_size = planned_path_full.poses.size();
            nav_msgs::Path old_full_path = planned_path_full;
            planned_path_full.poses.resize(global_path.poses.size() + planned_path_full.poses.size());
            ROS_INFO("Path Follower: Resized full path size: %i", planned_path_full.poses.size());

            for(int i=0; i < old_ful_path_size; i++)
            {
                    //add new pose
                planned_path_full.poses[i].pose.position.x = old_full_path.poses[i].pose.position.x;
                planned_path_full.poses[i].pose.position.y = old_full_path.poses[i].pose.position.y;
                planned_path_full.poses[i].pose.position.z = old_full_path.poses[i].pose.position.z;
            }

            int co_new = 0;
            for(int i=old_ful_path_size; i < old_ful_path_size + global_path.poses.size(); i++)
            {
                    //add new pose
                planned_path_full.poses[i].pose.position.x = global_path.poses[co_new].pose.position.x;
                planned_path_full.poses[i].pose.position.y = global_path.poses[co_new].pose.position.y;
                planned_path_full.poses[i].pose.position.z = global_path.poses[co_new].pose.position.z;
                co_new++;
            }

            ROS_INFO("Path Follower: Path successfull added to log!");

            planned_path_pub.publish(planned_path_full);
        }

        //publish local path for visualization
        local_path_pub.publish(local_path);

        //take last local pose and publish it
        geometry_msgs::PoseStamped tmp;
        //tmp.header.frame_id = local_path.header.frame_id;
        tmp.pose.position.x = local_path.poses[1].pose.position.x;
        tmp.pose.position.y = local_path.poses[1].pose.position.y;
        tmp.pose.position.z = local_path.poses[1].pose.position.z;

        planned_path_reactive.poses.push_back(tmp);

        planned_path_reactive_pub.publish(planned_path_reactive);

        //apply interface to STATE MACHINE
        state_machine_interface();

    }
    /*****************************************************************************************************************
     * State Machnie Interface
     */
    void iri_miriamm_path_follower::state_machine_interface()
    {

        //check if robot is on the ground

        //ROS_INFO("Path Follower: State Machine Controller Started!");
        if((curr_pose_z < liftoff_altitude_limit)&&(plan_state == "LANDED"))
        {
            if(use_stop_before_action == true)
            {
                //wait for takeoff
                char inc0;
                ROS_INFO("Path Follower: Wait for key to continue ... TAKEOFF");
                std::cin >> inc0;
            }

            ROS_INFO("[peter_wp_plan]: Take-off request.");
            take_offMakeActionRequest();
            task_state_ = TAKINGOFF;
            plan_state = "TAKEOFF";
        }
        else if(plan_state == "TAKEOFF")
        {
            if (take_off_action_succeeded_ == true)
            {
              //this->instr_num_ = this->instr_num_+1;
              this->task_state_ = HOVERING;
              plan_state = "HOVERING";
            }else
            {
                //ROS_INFO("Path Follower: Waiting for respond from SM to finish TAKEOFF!");
            }
        }
        else if((plan_state == "HOVERING")||(plan_state == "WAYFOLLOWING"))
        {
            if(use_stop_before_action == true)
            {
                //wait for new plan
                if (co_cin == 0)
                {
                    char inc1;
                    ROS_INFO("Path Follower: Wait for key to continue ... WAYPOINT FOLLOWING");
                    std::cin >> inc1;
                    co_cin = 2;
                }
            }

            //co_cin = 0;

            plan_state = "WAYFOLLOWING";

            //set wait time update_waypoints_freq
            usleep(1/update_waypoints_freq*1000000);

            //send waypoints
            waypointsMakeActionRequest();
            this->task_state_ = NAVIGATING;
        }

        if((plan_state == "WAYFOLLOWING")&&(start_path_following == false)&&(initiate_landing == true))
        {
            if(use_stop_before_action == true)
            {
                //land robot
                char inc2;
                ROS_INFO("Path Follower: Wait for key to continue ... LANDING ");
                std::cin >> inc2;
            }

            ROS_INFO("[peter_wp_plan]: Land request.");
            landMakeActionRequest();
            this->task_state_ = LANDING;
            plan_state = "LANDING";
        }
        else if(plan_state == "LANDING")
        {
            if(land_action_succeeded_ == true)
            {
                plan_state = "LANDED";
                ROS_INFO("Path Follower: Robot Has Landed Succesfully");
            }
        }

    }
    bool iri_miriamm_path_follower::landMakeActionRequest()
    {
      //ROS_INFO("[peter_wp_plan]: landMakeActionRequest: Starting New Request!");

      //this->alg_.lock();

      bool reqst_ok = false;
      if(land_client_.isServerConnected())
      {
        //send a goal to the action server
        land_goal_.activate = true;
        land_client_.sendGoal(land_goal_,
                    boost::bind(&iri_miriamm_path_follower::landDone,     this, _1, _2),
                    boost::bind(&iri_miriamm_path_follower::landActive,   this),
                    boost::bind(&iri_miriamm_path_follower::landFeedback, this, _1));
        ROS_INFO("[peter_wp_plan]: landMakeActionRequest: Goal Sent.");
        reqst_ok = true;
      }
      else
        ROS_INFO("[peter_wp_plan]: landMakeActionRequest: Action server is not connected");

      this->land_action_active_ = true;
      this->land_action_succeeded_ = false;

      //this->alg_.unlock();

      return reqst_ok;
    }
    void iri_miriamm_path_follower::landDone(const actionlib::SimpleClientGoalState& state,  const kinton_wp_ctrl::taskreqstResultConstPtr& result)
    {
      //this->alg_.lock();
      if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
      {
        this->land_action_succeeded_ = true;
        ROS_INFO("[peter_wp_plan]: landDone: Goal Achieved!");
      }
      else
      {
        this->land_action_succeeded_ = false;
        this->plan_active_ = false;
        ROS_INFO("[peter_wp_plan]: landDone: %s", state.toString().c_str());
      }

      this->land_action_active_ = false;

      //this->alg_.unlock();
    }
    void iri_miriamm_path_follower::landActive()
    {
      //this->alg_.lock();
      ROS_INFO("peterWpPlanAlgNode::landActive: Goal just went active!");
      //this->alg_.unlock();
    }
    void iri_miriamm_path_follower::landFeedback(const kinton_wp_ctrl::taskreqstFeedbackConstPtr& feedback)
    {
      //this->alg_.lock();
      //ROS_INFO("peterWpPlanAlgNode::landFeedback: Got Feedback!");

      bool feedback_is_ok = true;

      //analyze feedback
      //my_var = feedback->var;

      //if feedback is not what expected, cancel requested goal
      if( !feedback_is_ok )
      {
        land_client_.cancelGoal();
        ROS_INFO("peterWpPlanAlgNode::landFeedback: Cancelling Action!");
      }
      //this->alg_.unlock();
    }
    bool iri_miriamm_path_follower::waypointsMakeActionRequest()
    {
      ROS_INFO("[peter_wp_plan]: waypointsMakeActionRequest: Starting New Request!");

      //this->alg_.lock();

      bool reqst_ok = false;
      if(waypoints_client_.isServerConnected())
      {

        //Eigen::MatrixXd waypoints = this->waypoints_;
        int num_wp = local_path.poses.size();
        waypoints_goal_.waypoints.clear();

        //ROS_INFO("Num of waypoints: %i", num_wp);

        //fill in the waypoints
        for (int wp = 1; wp < num_wp; ++wp)
        {
            kinton_wp_ctrl::Waypoint k_wp;

            k_wp.pose.position.x = local_path.poses[wp].pose.position.x;
            k_wp.pose.position.y = local_path.poses[wp].pose.position.y;
            k_wp.pose.position.z = local_path.poses[wp].pose.position.z;

            k_wp.pose.orientation.x = 0.0;
            k_wp.pose.orientation.y = 0.0;
            k_wp.pose.orientation.z = 0.0;
            k_wp.pose.orientation.w = 1.0;

            //tf::Quaternion qt;
            //qt.setRPY(0.0,0.0,waypoints(3,wp));
            //geometry_msgs::Quaternion qt_msg;
            //tf::quaternionTFToMsg(qt,qt_msg);
            //waypoints_goal_.waypoints[wp].pose.orientation = qt_msg;

            k_wp.cruise = max_lin_velocity;
            k_wp.max_confidence_error[0] = goal_offset;
            k_wp.max_confidence_error[1] = goal_offset;
            k_wp.wait = 1;

            //add waypoints to msg
            waypoints_goal_.waypoints.push_back(k_wp);
        }

        //waypoints_goal_.data = my_desired_goal;
        waypoints_client_.sendGoal(waypoints_goal_,
                    boost::bind(&iri_miriamm_path_follower::waypointsDone,     this, _1, _2),
                    boost::bind(&iri_miriamm_path_follower::waypointsActive,   this),
                    boost::bind(&iri_miriamm_path_follower::waypointsFeedback, this, _1));
        //ROS_INFO("[peter_wp_plan]: waypointsMakeActionRequest: Goal Sent.");
        reqst_ok = true;
      }
      else
        ROS_INFO("[peter_wp_plan]: waypointsMakeActionRequest: Action server is not connected");

      this->waypoints_action_active_ = true;
      this->waypoints_action_succeeded_ = false;

      //this->alg_.unlock();

      return reqst_ok;
    }
    void iri_miriamm_path_follower::waypointsFeedback(const kinton_wp_ctrl::waypointsFeedbackConstPtr& feedback)
    {
      //this->alg_.lock();
      //ROS_INFO("WAYPOITN_FOLLOW_peterWpPlanAlgNode::waypointsFeedback: Got Feedback!");

      bool feedback_is_ok = true;

      //analyze feedback
      //my_var = feedback->var;

      //if feedback is not what expected, cancel requested goal
      if( !feedback_is_ok )
      {
        waypoints_client_.cancelGoal();
        ROS_INFO("WAYPOITN_FOLLOW_peterWpPlanAlgNode::waypointsFeedback: Cancelling Action!");
      }
      //this->alg_.unlock();
    }
    void iri_miriamm_path_follower::waypointsDone(const actionlib::SimpleClientGoalState& state,  const kinton_wp_ctrl::waypointsResultConstPtr& result)
    {
      //this->alg_.lock();
      if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
      {
        this->waypoints_action_succeeded_ = true;
        ROS_INFO("[peter_wp_plan]: waypointsDone: Goal Achieved!");
      }
      else
      {
        this->waypoints_action_succeeded_ = false;
        this->plan_active_ = false;
        ROS_INFO("[peter_wp_plan]: waypointsDone: %s", state.toString().c_str());
      }

      this->waypoints_action_active_ = false;

      //copy & work with requested result
      //this->alg_.unlock();
    }
    void iri_miriamm_path_follower::waypointsActive()
    {
      //this->alg_.lock();
      //ROS_INFO("peterWpPlanAlgNode::waypointsActive: Goal just went active!");
      //this->alg_.unlock();
    }
    bool iri_miriamm_path_follower::take_offMakeActionRequest()
    {
      //ROS_INFO("[peter_wp_plan]: take_offMakeActionRequest: Starting New Request!");

      //this->alg_.lock();

       bool reqst_ok = false;
      if(take_off_client_.isServerConnected())
      {
        //send a goal to the action server
        take_off_goal_.activate = true;
        take_off_client_.sendGoal(take_off_goal_,
                    boost::bind(&iri_miriamm_path_follower::take_offDone,     this, _1, _2),
                    boost::bind(&iri_miriamm_path_follower::take_offActive,   this),
                    boost::bind(&iri_miriamm_path_follower::take_offFeedback, this, _1));
        //ROS_INFO("[peter_wp_plan]: take_offMakeActionRequest: Goal Sent.");
        reqst_ok = true;
      }
      else
        ROS_INFO("[peter_wp_plan]: take_offMakeActionRequest: Action server is not connected");

      this->take_off_action_active_ = true;
      this->take_off_action_succeeded_ = false;

      //this->alg_.unlock();

      return reqst_ok;
    }
    void iri_miriamm_path_follower::take_offDone(const actionlib::SimpleClientGoalState& state,  const kinton_wp_ctrl::taskreqstResultConstPtr& result)
    {
      //this->alg_.lock();
      if( state == actionlib::SimpleClientGoalState::SUCCEEDED )
      {
        this->take_off_action_succeeded_ = true;
        ROS_INFO("[peter_wp_plan]: take_offDone: Goal Achieved!");
      }
      else
      {
        this->take_off_action_succeeded_ = false;
        this->plan_active_ = false;
        ROS_INFO("[peter_wp_plan]: take_offDone: %s", state.toString().c_str());
      }

      this->take_off_action_active_ = false;

      //this->alg_.unlock();
    }
    void iri_miriamm_path_follower::take_offActive()
    {
      //this->alg_.lock();
      ROS_INFO("peterWpPlanAlgNode::take_offActive: Goal just went active!");
      //this->alg_.unlock();
    }
    void iri_miriamm_path_follower::take_offFeedback(const kinton_wp_ctrl::taskreqstFeedbackConstPtr& feedback)
    {
      //this->alg_.lock();
      //ROS_INFO("peternWpPlanAlgNode::take_offFeedback: Got Feedback!");

      bool feedback_is_ok = true;

      //analyze feedback
      //my_var = feedback->var;

      //if feedback is not what expected, cancel requested goal
      if( !feedback_is_ok )
      {
        take_off_client_.cancelGoal();
        ROS_INFO("peterWpPlanAlgNode::take_offFeedback: Cancelling Action!");
      }
      //this->alg_.unlock();
    }
    /*****************************************************************************************************************
     * Pose update
     */
    void iri_miriamm_path_follower::pose_update(const geometry_msgs::PoseStamped pose)
    {
        //convert quaternionst to roll pitch yaw
        tf::Quaternion q;
        tf::quaternionMsgToTF(pose.pose.orientation, q);
        tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

        curr_pose_x = pose.pose.position.x;
        curr_pose_y = pose.pose.position.y;
        curr_pose_z = pose.pose.position.z;

        //ROS_INFO("Robot Position: X: %f / Y: %f / Z: %f Orientation: Z: %f / Y: %f / X: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, yaw, pitch, roll);

        //path following is based on position update
        if(start_path_following == true)
        {
            //use command vel control /cmd_vel
            if(use_vel_commands == true)
            {
                vel_command();
            }
            if(use_waypoint_commands == true)
            {
                use_waypoints();
            }

            if((use_vel_commands == true)&&(use_waypoint_commands == true))
            {
                ROS_WARN("Path Follower: You have defined both controls, be WARN that this might not be OK for your system!");
            }else if((use_vel_commands == false)&&(use_waypoint_commands == false))
            {
                ROS_ERROR("Path Follower: You have not defined any command controls!");
                return;
            }

        }
    }
    /*****************************************************************************************************************
     * Publish marker on position
     */
    void iri_miriamm_path_follower::raycasting_marker(const geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped end, double size, double size_z)
    {

        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "robot_colision";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = start.pose.position.x;
        marker.pose.position.y = start.pose.position.y;
        marker.pose.position.z = start.pose.position.z;
        marker.pose.orientation.x = start.pose.orientation.x;
        marker.pose.orientation.y = start.pose.orientation.y;
        marker.pose.orientation.z = start.pose.orientation.z;
        marker.pose.orientation.w = start.pose.orientation.w;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.1;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        colision_start_pub.publish( marker );

        marker.header.frame_id = global_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "robot_colision";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = end.pose.position.x;
        marker.pose.position.y = end.pose.position.y;
        marker.pose.position.z = end.pose.position.z;
        marker.pose.orientation.x = end.pose.orientation.x;
        marker.pose.orientation.y = end.pose.orientation.y;
        marker.pose.orientation.z = end.pose.orientation.z;
        marker.pose.orientation.w = end.pose.orientation.w;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.1;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        colision_end_pub.publish( marker );
        marker.DELETE;

    }
    /*****************************************************************************************************************
     * Publish marker on position
     */
    void iri_miriamm_path_follower::robot_colision_checking_pub(const geometry_msgs::PoseStamped pose)
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

        marker.DELETE;
    }

    /*****************************************************************************************************************
     * Publish warning colision marker
     */
    void iri_miriamm_path_follower::raycasting_collision_marker(const geometry_msgs::PoseStamped pose)
    {
        double pos_x, pos_y, pos_z;
        pos_x = pose.pose.position.x;
        pos_y = pose.pose.position.y;
        pos_z = pose.pose.position.z;

        //publish MESH marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame;
        marker.header.stamp = ros::Time();
        marker.ns = "robot_colision_marker";
        marker.id = count_sphere_marker;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos_x;
        marker.pose.position.y = pos_y;
        marker.pose.position.z = pos_z;
        marker.pose.orientation.x = pose.pose.orientation.x;
        marker.pose.orientation.y = pose.pose.orientation.y;
        marker.pose.orientation.z = pose.pose.orientation.z;
        marker.pose.orientation.w = pose.pose.orientation.w;
        marker.scale.x = raycasting_net_raster;
        marker.scale.y = raycasting_net_raster;
        marker.scale.z = raycasting_net_raster;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.4078;
        marker.color.g = 0.1333;
        marker.color.b = 0.5450;

        marker.DELETE;
        //255;28;174 104;34;139

        if(pose.pose.position.x == 100.0)
        {
            //delete marker
            sphere_marker.markers.clear();
            count_sphere_marker = 0;
        }

        sphere_marker.markers.resize(count_sphere_marker+1);
        //sphere_marker.markers.push_back(marker);
        sphere_marker.markers[count_sphere_marker] = marker;

        raycasting_collision_marker_pub.publish( sphere_marker );

        count_sphere_marker++;

        ROS_INFO("Path Follower: Colision Marker Published!, number: %i", count_sphere_marker);
    }
    /*****************************************************************************************************************
     * Octomap callback
     */
    void iri_miriamm_path_follower::call_octomap()
    {
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

            if(co_octomap == 1)
            {
                octree_res = octree->getResolution();
                ROS_INFO("Path Follower::  Octomap callback - Tree resolution: %f Octree size: %i", octree_res, octree->size());
                double x,y,z;
                octree->getMetricSize(x, y, z);
                ROS_INFO("Tree metric size: x: %f y: %f z: %f", x,y,z);
            }

            //calculate pd forces
            //calculate_potential_field_forces();
            //calculate_pf();

        }
    }
    /*****************************************************************************************************************
     * Calculate potential field repulsive-attractive
     */
    void iri_miriamm_path_follower::calculate_potential_field_forces()
    {

        if (start_path_following == true)
        {
            //search params
            point3d end;

            //find next closest point
            geometry_msgs::PoseStamped col_pose;
            col_pose = closest_point_to_curr_pos();
            int next_point = closest_point_pos;
            int count_check_pos = 0;

            //get poins in local path
            bool do_check = true;

            double force_sum_x = 0;
            double force_sum_y = 0;
            double force_sum_z = 0;
            double sum_hit_distance = 0;
            int co_hd = 0;

            if (show_markers)
            {

                //do checks SQARE BOX - erase history
    //            geometry_msgs::PoseStamped colision;
    //            colision.pose.position.x = 100;
    //            colision.pose.position.y = 100;
    //            colision.pose.position.z = 100;
    //            raycasting_collision_marker(colision);

                //Publish visualization
                geometry_msgs::PoseStamped start;
                start.pose.position.x = curr_pose_x - lookahead_local;
                start.pose.position.y = curr_pose_y - lookahead_local;
                start.pose.position.z = curr_pose_z;

                geometry_msgs::PoseStamped endp;
                endp.pose.position.x = curr_pose_x + lookahead_local;
                endp.pose.position.y = curr_pose_y + lookahead_local;
                endp.pose.position.z = curr_pose_z;

                //show raycasting marker
                double show_size = 0.4;
                raycasting_marker(start, endp, show_size, show_size);
            }

            //calculate limits
            int num_ch = ((lookahead_local)/raycasting_net_raster);
            int num_ch_z = ((rob_col_z)/raycasting_net_raster);

            //calculate points
            double xs, ys, xe, ye, zs, ze;
            closest_distance = 100;

            for(int z = -num_ch_z; z < num_ch_z+1; z++)
            {
                zs = z*raycasting_net_raster + curr_pose_z;
                ze = z*raycasting_net_raster + curr_pose_z;

                for(int r = -num_ch; r < num_ch+1; r++)
                {

                    //start point
                    xs = curr_pose_x - lookahead_local;
                    ys = r*raycasting_net_raster +  curr_pose_y;

                    //end point
                    xe = curr_pose_x + lookahead_local;
                    ye = r*raycasting_net_raster + curr_pose_y;

                    //show obstacle TO SEE WHERE THE RAYCASING is performed
//                    geometry_msgs::PoseStamped colision;
//                    colision.pose.position.x = xs;
//                    colision.pose.position.y = ys;
//                    colision.pose.position.z = zs;
//                    raycasting_collision_marker(colision);
//                    colision.pose.position.x = xe;
//                    colision.pose.position.y = ye;
//                    colision.pose.position.z = ze;
//                    raycasting_collision_marker(colision);

                    //do raycasting
                    point3d origin(xs, ys, zs);
                    point3d direction(xe, ye, ze);
                    hit = octree->castRay(origin, direction, end, true, lookahead_local);

                    //ROS_INFO("Path Follower: Start: %f, %f, %f --> End: %f %f %f // RESULT:: %f, %f, %f",origin.x(), origin.y(), origin.z(), direction.x(), direction.y(), direction.z(), end.x(), end.y(), end.z());

                    if(hit == true)
                    {
                        //distnace from orign to end
                        double hit_distance = sqrt(pow(end.x() - curr_pose_x,2.0)+pow(end.y() - curr_pose_y,2.0)+pow(end.z() - curr_pose_z,2.0));

//                        //ROS_INFO("Path Follower: Hit Distance: %f, lookahead_local: %f", hit_distance, lookahead_local);
                        ROS_ERROR("Path Follower: Ray has HIT the OBSTACLE!");

                        if(show_markers)
                        {
                            //show obstacle
                            geometry_msgs::PoseStamped colision;
                            colision.pose.position.x = end.x();
                            colision.pose.position.y = end.y();
                            colision.pose.position.z = end.z();
                            raycasting_collision_marker(colision);
                        }

                        //calculate repulsive function
                        force_sum_x = force_sum_x + (curr_pose_x - end.x());
                        force_sum_y = force_sum_y + (curr_pose_y - end.y());
                        force_sum_z = force_sum_z + (curr_pose_z - end.z());
                        sum_hit_distance = sum_hit_distance + hit_distance;
                        co_hd++;

                        if(hit_distance < (rob_col_x/2))
                        {
                            ROS_ERROR("Path Follower: Robot HAS HIT SOMETHING!!!");
                        }

                        if(hit_distance < closest_distance)
                        {
                            closest_distance = hit_distance;
                        }
                    }

                    //do raycasting - other side
                    point3d originr(xe, ye, ze);
                    point3d directionr(xs, ys, zs);
                    hit = octree->castRay(originr, directionr, end, true, lookahead_local);

                    if(hit == true)
                    {
                        //distnace from orign to end
                        double hit_distance = sqrt(pow(end.x() - curr_pose_x,2.0)+pow(end.y() - curr_pose_y,2.0)+pow(end.z() - curr_pose_z,2.0));

                        //ROS_INFO("Path Follower: Hit Distance: %f, lookahead_local: %f", hit_distance, lookahead_local);
                        ROS_ERROR("Path Follower: Ray has HIT the OBSTACLE!");

                        if(show_markers)
                        {
                            //show obstacle
                            geometry_msgs::PoseStamped colision;
                            colision.pose.position.x = end.x();
                            colision.pose.position.y = end.y();
                            colision.pose.position.z = end.z();
                            raycasting_collision_marker(colision);
                        }

                        //calculate repulsive function
                        force_sum_x = force_sum_x + (curr_pose_x - end.x());
                        force_sum_y = force_sum_y + (curr_pose_y - end.y());
                        force_sum_z = force_sum_z + (curr_pose_z - end.z());
                        sum_hit_distance = sum_hit_distance + hit_distance;
                        co_hd++;

                        if(hit_distance < (rob_col_x/2))
                        {
                            ROS_ERROR("Path Follower: Robot HAS HIT SOMETHING!!!");
                        }

                        if(hit_distance < closest_distance)
                        {
                            closest_distance = hit_distance;
                        }
                    }
                    // do raycasing in other two directions
                }
            }
            //če je neprehodno je potrebno povečati lookahead distance

            ROS_INFO("Path Follower: EXIT LOOP - PATH CHECKING FINISHED! Counted HITS: %i", co_hd);

            //average results
            if(co_hd > 0)
            {
                avg_force_x = force_sum_x/co_hd;
                avg_force_y = force_sum_y/co_hd;
                avg_force_z = force_sum_z/co_hd;
                avg_hit_distance = sum_hit_distance/co_hd;

                //calculate forces
                potential_field_calc();
            }else
            {
                pf_force_vel_x = 0;
                pf_force_vel_y = 0;
                pf_force_vel_z = 0;
            }
        }else
        {
            pf_force_vel_x = 0;
            pf_force_vel_y = 0;
            pf_force_vel_z = 0;
        }
    }
    /*****************************************************************************************************************
     * Calculate forces for potential field
     */
    void iri_miriamm_path_follower::potential_field_calc()
    {
        if(start_path_following == true)
        {
            //include reciprocal function

            //determine vel signs
            double close_diff_x = avg_force_x - curr_pose_x;
            double close_diff_y = avg_force_y - curr_pose_y;

            double sign_vel_x, sign_vel_y, sign_vel_z;

            //define signs
            sign_vel_z = avg_force_z/fabs(avg_force_z);

            if((close_diff_x < 0)&&(close_diff_y < 0))
            {
                sign_vel_x = avg_force_x/fabs(avg_force_x);
                sign_vel_y = avg_force_y/fabs(avg_force_y);

                ROS_INFO("FORCES DIRECTIONS ARE _+I+_");
            }
            if((close_diff_x < 0)&&(close_diff_y > 0))
            {
                sign_vel_x = avg_force_x/fabs(avg_force_x);
                sign_vel_y = avg_force_y/fabs(avg_force_y);

                ROS_INFO("FORCES DIRECTIONS ARE _+I-_");
            }
            if((close_diff_x > 0)&&(close_diff_y > 0))
            {
                sign_vel_x = -avg_force_x/fabs(avg_force_x);
                sign_vel_y = -avg_force_y/fabs(avg_force_y);

                ROS_INFO("FORCES DIRECTIONS ARE _-I-_");
            }
            if((close_diff_x > 0)&&(close_diff_y < 0))
            {
                sign_vel_x = -avg_force_x/fabs(avg_force_x);
                sign_vel_y = -avg_force_y/fabs(avg_force_y);

                ROS_INFO("FORCES DIRECTIONS ARE _-I+_");
            }

            if(avg_hit_distance > lookahead_local)
            {
                avg_hit_distance = lookahead_local;
            }

            pf_force_vel_x = reactive_const_pf*sign_vel_x*(max_lin_velocity/(lookahead_local/(lookahead_local - fabs(avg_force_x) + rob_col_x/2)));
            pf_force_vel_y = reactive_const_pf*sign_vel_y*(max_lin_velocity/(lookahead_local/(lookahead_local - fabs(avg_force_y) + rob_col_y/2)));
            pf_force_vel_z = reactive_const_pf*sign_vel_z*(max_lin_velocity/(lookahead_local/(lookahead_local - fabs(avg_force_z) + rob_col_z/2)));

            //lock vel on axes if closeses distance is smaller
//            double smallest_var = 0;
//            if(closest_distance < rob_col_x/2)
//            {
//                //find smallest cmd vel:
//                smallest_var = (double)std::min(std::min(fabs(pf_force_vel_x), fabs(pf_force_vel_y)), fabs(pf_force_vel_z));
//                ROS_INFO("Pathh Follower: Smallest vel: %f", smallest_var);

//                if(smallest_var == fabs(pf_force_vel_x))
//                    lock_x_vel = true;

//                if(smallest_var == fabs(pf_force_vel_y))
//                    lock_y_vel = true;

//                if(smallest_var == fabs(pf_force_vel_z))
//                    lock_z_vel = true;
//            }
//            else
//            {
//                lock_x_vel = false;
//                lock_y_vel = false;
//                lock_z_vel = false;
//            }

            //ROS_INFO("Path Follower: Robot hit distance: %f, closest distance: %f", rob_hit_distance, closest_distance);



//            //encrease search area if forces are too big
//            double sum_forces = fabs(pf_force_vel_x) + fabs(pf_force_vel_y) + fabs(pf_force_vel_z);

//            if(sum_forces > max_lin_velocity*1.5)
//            {
//                lookahead_local = lookahead_local + raycasting_net_raster;
//                ROS_ERROR("Path Follower: Forces have overreached treshold, increasing lookahead distance: %f");
//            }

//            if(fabs(pf_force_vel_x) > max_lin_velocity)
//            {
//                pf_force_vel_x = pf_force_vel_x/fabs(pf_force_vel_x)*max_lin_velocity;
//            }

//            if(fabs(pf_force_vel_y) > max_lin_velocity)
//            {
//                pf_force_vel_y = pf_force_vel_y/fabs(pf_force_vel_y)*max_lin_velocity;
//            }

//            if(fabs(pf_force_vel_z) > max_lin_velocity)
//            {
//                pf_force_vel_z = pf_force_vel_z/fabs(pf_force_vel_z)*max_lin_velocity;
//            }

            ROS_ERROR("Path Follower: Repulsive forces are: %f %f %f // CMD: %f %f %f", avg_force_x, avg_force_y, avg_force_z, pf_force_vel_x, pf_force_vel_y, pf_force_vel_z);

        }
    }
    /*****************************************************************************************************************
     * Check if position is covering octomap
     */
    void iri_miriamm_path_follower::check_position_colision(const geometry_msgs::PoseStamped pose)
    {
        //Check for collisions at current position!
        //define node of octree - global
        OcTreeNode* oc_node;

        //defina area search
        int zd0, zd1;//, yd0, yd1, xd0, xd1;

        zd0 = (pose.pose.position.z - rob_col_z/2)*(1/octree_res);
        zd1 = (pose.pose.position.z + rob_col_z/2)*(1/octree_res);

        pose_occupied = false;

        double x, y;
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
     * Find closest point to currnet position
     */
    geometry_msgs::PoseStamped iri_miriamm_path_follower::closest_point_to_curr_pos()
    {
        geometry_msgs::PoseStamped closest_point;

        double temp_dist = 10000;
        int small_i = 0;
        for(int i=0; i < global_path.poses.size(); i++)
        {
            //calculate distance between two points
            goal_dist = sqrt(pow((curr_pose_x - global_path.poses[i].pose.position.x),2) + pow((curr_pose_y - global_path.poses[i].pose.position.y),2)+pow((curr_pose_z - global_path.poses[i].pose.position.z),2));

            //distance must be larger than 0
            if(goal_dist < temp_dist)
            {
               temp_dist = goal_dist;
               small_i = i;
            }

        }

        //get nex point on the way
        int new_point;
        new_point = small_i +1;
        if(new_point > global_path.poses.size()-1)
        {
            new_point = small_i;
        }
        
        closest_point_pos = new_point;

        //ROS_INFO("Path Follower: Closest Point - x: %f y: %f z: %f", global_path.poses[new_point].pose.position.x,global_path.poses[new_point].pose.position.y,global_path.poses[new_point].pose.position.z);

        //mark points
        closest_point.pose.position.x = global_path.poses[new_point].pose.position.x;
        closest_point.pose.position.y = global_path.poses[new_point].pose.position.y;
        closest_point.pose.position.z = global_path.poses[new_point].pose.position.z;

        return closest_point;
    }
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "iri_miriamm_path_follower");
    ros::NodeHandle nh;

    //for registering the node
    iri_miriamm_path_follower::iri_miriamm_path_follower iri_miriamm_path_follower_handler(nh);

    iri_miriamm_path_follower_handler.init();                     //initize

    ros::Rate loop_rate(iri_miriamm_path_follower_handler.update_cmd_vel_freq);  //daj to v parametre
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    //ros::spin();
    return 0;
}
