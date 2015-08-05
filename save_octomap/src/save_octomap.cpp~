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

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

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

#include <octomap/octomap.h>
//#include <octomap/OcTreeLabeled.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace std;
using namespace octomap;

octomap::OcTree* octree;
OcTreeNode* oc_node;
double octree_res;

void buildCloud(OcTree tree)
{

    //how many occupied cells do we have in the tree?
    //std::list<octomap::OcTreeVolume> occupiedCells;
    //tree.getOccupied(occupiedCells);

/*
    //cloud to store the points
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize(occupiedCells.size());

    std::list<octomap::OcTreeVolume>::iterator it;
    int i=0;
    for (it = occupiedCells.begin(); it != occupiedCells.end(); ++it, i++)
    {
        //add point in point cloud
        cloud.points[i].x = it->first.x();
        cloud.points[i].y = it->first.y();
        cloud.points[i].z = it->first.z();      
    }   



    //save cloud
    pcl::io::savePCDFileASCII ("file.pcd", cloud);
*/
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "iri_save_octomap");
	ros::NodeHandle nh;


	//call octomap
	std::string servname = "octomap_full";
	octomap_msgs::GetOctomap::Request req;
	octomap_msgs::GetOctomap::Response resp;

	while(nh.ok() && !ros::service::call(servname, req, resp))
	{
		ROS_WARN("Request to %s failed; trying again...", nh.resolveName(servname).c_str());
		usleep(1000000);
	}

	if (nh.ok())
	{ // skip when CTRL-C

		AbstractOcTree* tree = octomap_msgs::msgToMap(resp.map);
		octree = dynamic_cast<octomap::OcTree*>(tree);

		ROS_INFO("Path Follower::  Octomap callback - Tree resolution: %f Octree size: %i", octree->getResolution(), octree->size());
		octree_res = octree->getResolution();
		double x,y,z;
		octree->getMetricSize(x, y, z);
		ROS_INFO("Tree metric size: x: %f y: %f z: %f", x,y,z);

		double xmax,ymax,zmax;
		double xmin,ymin,zmin;
		octree->getMetricMax(xmax,ymax,zmax);
		octree->getMetricMin(xmin,ymin,zmin);

		//convert to int
		int zmini = zmin/octree_res; 
		int zmaxi = zmax/octree_res;

		int ymini = ymin/octree_res; 
		int ymaxi = ymax/octree_res;

		int xmini = xmin/octree_res; 
		int xmaxi = xmax/octree_res;

		//octomap search
		int co_occ = 0;
		for(int z=zmini; z < zmaxi; z++)
		{
		        for(int y=ymini; y < ymaxi; y++)
		        {
		                for(int x=xmini; x < xmaxi; x++)
		                {
		                        oc_node = octree->search((double)x*octree_res, (double)y*octree_res, (double)z*octree_res);
		                        //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x*octree_res, (double)y*octree_res, (double)z*octree_res);

		                        if(oc_node)
		                        {
		                            if(oc_node->getValue() > 0)
		                            {
						//pose occupied
						co_occ++;
		                            }
					}
				}
		        }
		}

		ROS_INFO("Number of occuped nodes is: %i", co_occ);

		//cloud to store the points
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.points.resize(co_occ);

		ROS_INFO("Created and resized PCL, add points to the cloud ...");

		int co_pcl = 0;
		for(int z=zmini; z < zmaxi; z++)
		{
		        for(int y=ymini; y < ymaxi; y++)
		        {
		                for(int x=xmini; x < xmaxi; x++)
		                {
		                        oc_node = octree->search((double)x*octree_res, (double)y*octree_res, (double)z*octree_res);
		                        //ROS_INFO("search in X: %f Y: %f Z: %f", (double)x*octree_res, (double)y*octree_res, (double)z*octree_res);

		                        if(oc_node)
		                        {
		                            if(oc_node->getValue() > 0)
		                            {
						//add point in point cloud
						cloud.points[co_pcl].x = (double)x*octree_res;
						cloud.points[co_pcl].y = (double)y*octree_res;
						cloud.points[co_pcl].z = (double)z*octree_res; 
						co_pcl++;  

						//ROS_INFO("Point num: %i pos: %f %f %f", co_pcl, (double)x*octree_res, (double)y*octree_res,(double)z*octree_res);
		                            }
					}
				}
		        }
		}

		cloud.width = 1;
		cloud.height = cloud.points.size();

		ROS_INFO("Number of occuped nodes is AFTER: %i", co_pcl);

		ROS_INFO("Point cloud created ...");

    		//save cloud
    		pcl::io::savePCDFileASCII ("/home/peter/ros/catkin_ws/src/save_octomap/pcd/file.pcd", cloud);

		ROS_INFO("Point cloud saved ...");

		//pcl_voxel_grid file.pcd file_con.pcd -leaf 0.0005,0.0005,0.0005
/*		const char command_l;
		command_l = "pcl_voxel_grid /home/peter/ros/catkin_ws/src/save_octomap/pcd/file.pcd /home/peter/ros/catkin_ws/src/save_octomap/pcd/file_con.pcd -leaf ";
		command_l += octree_res;
		command_l += ", ";
		command_l += octree_res;
		command_l += ", ";
		command_l += octree_res;
		command_l += " &";
*/
		system("pcl_voxel_grid /home/peter/ros/catkin_ws/src/save_octomap/pcd/file.pcd /home/peter/ros/catkin_ws/src/save_octomap/pcd/file_con.pcd -leaf 0.001,0.001,0.001 &");

		ROS_INFO("Leafs from PCD created ...");

		//pcl_pcd2ply file_con.pcd file_con.ply
/*		const char command_con;
		command_con = "pcl_pcd2ply /home/peter/ros/catkin_ws/src/save_octomap/pcd/file_con.pcd /home/peter/ros/catkin_ws/src/save_octomap/pcd/file_con.ply ";
		command_con += " &";
*/		system("pcl_pcd2ply /home/peter/ros/catkin_ws/src/save_octomap/pcd/file_con.pcd /home/peter/ros/catkin_ws/src/save_octomap/pcd/file_con.ply &");

		ROS_INFO("Converted to PLY, you can use MESHLAB to create meshes!");
	}


	//ros::spin();
	return 0;
}
