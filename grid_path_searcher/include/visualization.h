/**
 *  This file is used for visualization
 *  Author: jianzhuozhu
 *  Date: 2021-7-27
 */
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Eigen>
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

using namespace std;
using namespace Eigen;


/**
 * @brief Function for draw RRTstar Path
 */
void visRRTstarPath(vector<Vector3d> nodes ,ros::Publisher _RRTstar_path_vis_pub);
/**
 * @brief Function for draw goal point
 */
void visRRTgoal(vector<Vector3d> nodes ,ros::Publisher _RRTstar_path_vis_pub);
/**
 * @brief make frame to show pose and shape
 */
vector<Vector3d> showframe(vector<Vector3d> nodes);
/**
 * @brief calculate rotmatrix 
 */
Vector3d calculate_rotmatrix(Vector3d vectorBefore,Vector3d vectorAfter,Vector3d p_tmp);
