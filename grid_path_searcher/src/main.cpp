
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "RRT_star.h"
#include "backward.hpp"
#include "visualization.h"


using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}
ros::Publisher  _grid_map_vis_pub;
ros::Publisher _RRTstar_path_vis_pub;
ros::Publisher _RRTstar_goal_vis_pub;

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d target_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ros related
ros::Subscriber _map_sub, _pts_sub;
World * _RRTstar_world = new World();

pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ> cloud_vis;
sensor_msgs::PointCloud2 map_vis;

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);


void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;
    //是否采用rviz外部传入的目标点信息
    if(1){
        target_pt << wp.poses[0].pose.position.x,
                    wp.poses[0].pose.position.y,
                    wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
    }

}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{    
    if(_has_map ) {_grid_map_vis_pub.publish(map_vis);return;}

    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];  
              

        // set obstalces into grid map for path planning
        _RRTstar_world->setObs(pt.x, pt.y, pt.z);
        // for visualize only
        Vector3d cor_round = _RRTstar_world->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    //将height设为1说明这个cloud_vis点云序列是无序点云，而width代表无序点云的数量
    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    //指定点云中的所有数据都是有限的（true），还是其中的一些点不是有限的，它们的XYZ值可能包含inf/NaN 这样的值（false）。
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    
    //define start and end positions
    Point start_pos(start_pt[0], start_pt[1], start_pt[2]);
    Point end_pos(target_pt[0], target_pt[1],target_pt[2]);
    vector<Vector3d> path_points;//visualize the generated path
    vector<Vector3d> start_goal_points;//visualize the goal
    start_goal_points.push_back(target_pt);
    start_goal_points.push_back(_start_pt);
    visRRTgoal(start_goal_points, _RRTstar_goal_vis_pub) ;
    //define the raduis for RRT* algorithm (Within a radius of r, RRT* will find all neighbour nodes of a new node).
    float rrt_radius = 1;
    //define the radius to check if the last node in the tree is close to the end position
    float end_thresh = 1;
    //instantiate RRTSTAR class
    RRTSTAR* rrtstar = new RRTSTAR(start_pos,end_pos, rrt_radius, end_thresh, _RRTstar_world);
    
    // RRT* Algorithm
    /*
     Description of RRT* algorithm: 
    1. Pick a random node "N_rand".
    2. Find the closest node "N_Nearest" from explored nodes to branch out towards "N_rand".
    3. Steer from "N_Nearest" towards "N_rand": interpolate if node is too far away. The interpolated Node is "N_new"
    4.  Check if an obstacle is between new node and nearest nod.
    5. Update cost of reaching "N_new" from "N_Nearest", treat it as "cmin". For now, "N_Nearest" acts as the parent node of "N_new".
    6. Find nearest neighbors with a given radius from "N_new", call them "N_near"
    7. In all members of "N_near", check if "N_new" can be reached from a different parent node with cost lower than Cmin, and without colliding
    with the obstacle. Select the node that results in the least cost and update the parent of "N_new".
    8. Add N_new to node list.
    9. Rewire the tree if possible. Search through nodes in "N_near" and see if changing their parent to "N_new" lowers the cost of the path. If so, rewire the tree and
    add them as the children of "N_new" and update the cost of the path.
    10. Continue until maximum number of nodes is reached or goal is hit.
    */
    std::cout << "Starting RRT* Algorithm..." << std::endl;
    //search for the first viable solution
    std::vector<Point> initial_solution =rrtstar->planner();
    if (!initial_solution.empty()) {
        std::cout << "First Viable Solution Obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->lastnode->cost << std::endl;
        std::cout << "Saving the generated plan (vector of points)" << std::endl;
    }
    std::vector<Point> optimized_solution;
    //search for the optimized paths
    while (rrtstar->getCurrentIterations() < rrtstar->getMaxIterations() && !initial_solution.empty())
    {
        std::cout << "=========================================================================" << std::endl;
        std::cout << "The algorithm continues iterating on the current plan to improve the plan" << std::endl;
        optimized_solution = rrtstar->planner();
        std::cout << "More optimal solution has obtained after " << rrtstar->getCurrentIterations() << " iterations" << std::endl;
        std::cout << "Cost is " << rrtstar->m_cost_bestpath << std::endl;



        cout<<rrtstar->getCurrentIterations()<<"   "<<rrtstar->getMaxIterations()<<endl;


        for (size_t path_idx = 0; path_idx < optimized_solution.size(); path_idx++)
        {
            /**
            *
            *
            STEP 7: Trandform the found path from path to path_points for rviz display
            *
            *
            */
            Vector3d tmp(0,0,0);
            tmp[0] = optimized_solution[path_idx].m_x;
            tmp[1] = optimized_solution[path_idx].m_y;
            tmp[2] = optimized_solution[path_idx].m_z;
            path_points.push_back(tmp);
        }
        visRRTstarPath(path_points, _RRTstar_path_vis_pub);   



    }



}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _RRTstar_path_vis_pub         = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis",5);
    _RRTstar_goal_vis_pub         = nh.advertise<visualization_msgs::Marker>("RRTstar_goal_vis",1);


    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 10.0);
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
    
    nh.param("planning/target_x",  target_pt(0),  0.0);
    nh.param("planning/target_y",  target_pt(1),  0.0);
    nh.param("planning/target_z",  target_pt(2),  0.0);
    
    _map_lower << - _x_size/2.0, - _y_size/2.0,- _z_size/2.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0,+ _z_size/2.0;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _RRTstar_world = new World();
    _RRTstar_world ->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _RRTstar_world;
    return 0;
}

