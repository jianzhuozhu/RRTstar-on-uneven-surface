/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 */


#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <float.h>
#include "backward.hpp"
#include "node.h"

using namespace std;
using namespace Eigen;

// ======= 3 Classes : Point, Line, World ============== //
/**
* @brief Class for storing position of a point in the space
*/
class Point {
public:
    //Data member
    float m_x;
    float m_y;
    float m_z;
    //Constructors
    Point();
    Point(float X, float Y,float Z);
};



//Operators overloading for Point class
template <typename T>
inline Point operator *(const Point& left, T right)
{
    return Point(left.m_x * right, left.m_y * right,left.m_z * right);
}


inline Point operator +(const Point& left, const Point& right)
{
    return Point(left.m_x + right.m_x, left.m_y + right.m_y,left.m_z + right.m_z);
}


inline Point operator -(const Point& left, const Point& right)
{
    return Point(left.m_x - right.m_x, left.m_y - right.m_y, left.m_z - right.m_z);
}

template <typename T>
inline Point operator /(const Point& left, T right)
{
    return Point(left.m_x / right, left.m_y / right,left.m_z / right);
}



/**
* @brief Class for creating a line segment from two Points. 
*/
class Line {
public:
    //Constructors
    Line();
    Line(float x1pos, float y1pos,float z1pos, float x2pos, float y2pos,float z2pos);
    //Methods

    /**
    * @brief Return if "this" line has an intersection with "other" line.
    * @param Line OtherLine
    * @return bool
    */
    bool LineIntersection(const Line& OtherLine);
private:
    // Data Members 
    Point m_p1;
    Point m_p2;
};



/**
* @brief class for storing obstacles and world dimension. Obstacles are stored as rectangles. Rectangle is denoted by two points; top left and bottom right.
* if the world width and world height do not set using setWorldWidth and setWorldHeight methods, the default values which are world_width=500 and world_height=500
* will be set.
*/
class World
{
private:
    // Data Members 
    std::vector<std::pair<Point, Point> > m_obstacles;
    //storing map data
    protected:
    uint8_t * data;
    GridNodePtr *** GridNodeMap;

    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
    int GLXYZ_SIZE, GLYZ_SIZE;

    double resolution, inv_resolution;
    double gl_xl, gl_yl, gl_zl;
    double gl_xu, gl_yu, gl_zu;	

    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

public:
    float world_x_range;
    float world_y_range;
    float world_z_range;
    
    //init grid map
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
    void setObs(const double coord_x, const double coord_y, const double coord_z);
    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
    float distance(const Point p, const Point q);
    bool isObsFree(const double coord_x, const double coord_y, const double coord_z);
    //Constructors
    //假设world为一个立方体，边长为world_x_range，world_y_range，world_z_range
    World();
    World(float world_x_range, float world_y_range ,float world_z_range);
    //class Methods
    
    /**
    * @brief Create obstacles. Obstacles are stored as rectangles. Rectangle is denoted by two points; top left and bottom right.
    * @param Point topLeft (i.e., position of the top left point)
    * @param Point bottomRight (i.e., position of the bottom right point)
    * @return bool
     */
    void addObstacle(Point& topLeft, Point& bottomRight);

    /**
   * @brief Check if there is any obstacle between the 2 nodes.
   * @param Point p1 (i.e., the position of the first node)
   * @param Point p2 (i.e., the position of the second node)
   * @return bool value of whether obstacle exists between nodes
   */
    bool checkObstacle(Point& p1, Point& p2);


    /**
    * @brief Save obstacles to file
    * @param std::string filename
    * @param std::string fileHeader.
    * @return void
     */
    void saveObsToFile(const std::string filename);

    /**
    * @brief set the width of the world
    * @param float w
    * @return void
    */
    void setWorldWidth(float w);

    /**
    * @brief set the height of the world
    * @param float h
    * @return void
    */
    void setWorldHeight(float h);

    /**
    * @brief get the width of the world
    * @param void
    * @return float
    */
    float getWorldWidth();

    /**
    * @brief get the height of the world
    * @param void
    * @return float
    */
    float getWorldHeight();
    
    /**
    * @brief get the low bound of the world
    * @param void
    * @return Vector3d
    */
    Vector3d getglobalmaplb();


    /**
    * @brief get the up bound of the world
    * @param void
    * @return Vector3d
    */
    Vector3d getglobalmapub();


};

