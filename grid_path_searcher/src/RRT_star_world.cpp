/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  Author: jianzhuozhu
 *  Date: 2021-7-24
 */


#include"RRT_star_world.h"
using namespace std;
using namespace Eigen;

//Point class Constructors
Point::Point()
    :m_x(0.0f), m_y(0.0f), m_z(0.0f) {}
Point::Point(float X, float Y,float Z)
    : m_x(X), m_y(Y),m_z(Z) {}


//Line class Constructors
Line::Line() {};
Line::Line(float x1pos, float y1pos,float z1pos, float x2pos, float y2pos,float z2pos)
{
    this->m_p1.m_x = x1pos;
    this->m_p1.m_y = y1pos;
    this->m_p1.m_z = z1pos;
    this->m_p2.m_x = x2pos;
    this->m_p2.m_y = y2pos;
    this->m_p2.m_z = z2pos;
}

//World class Constructors
World::World()
    : world_x_range(500), world_y_range(500) ,world_z_range(500){}


World::World(float world_x_range, float world_y_range ,float world_z_range)
    : world_x_range(world_x_range), world_y_range(world_y_range) ,world_z_range(world_z_range){}



bool World::checkObstacle(Point& p1, Point& p2) {
    Line co_line(p1.m_x, p1.m_y,p1.m_z, p2.m_x, p2.m_y,p2.m_z); //creat line between two nodes
    float co_line_length = distance(p1,p2);//两点间距
    float co_line_check_step = resolution;//使用分辨率做步长
    int co_line_check_step_num = (int)co_line_length/co_line_check_step;//计算要检测多少步
    Point norm_p = p2-p1;
    norm_p = norm_p / co_line_length;  //normalize the vector
    bool is_obs_tmp = false;//true 表示有障碍，flase 表示没有障碍
    bool is_obs = false;//true 表示有障碍，flase 表示没有障碍

    for(int i = 0;i<=co_line_check_step_num;i++){
        Point check_p = p1 + norm_p * (i*co_line_check_step);
        is_obs_tmp = ! isObsFree(check_p.m_x,check_p.m_y,check_p.m_z);
        if(is_obs_tmp == true){is_obs = true;}
    }
    is_obs = is_obs && (! isObsFree(p2.m_x, p2.m_y,p2.m_z));

    return is_obs;
}

void World::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);

    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void World::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

//判断是否占据
bool World::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    bool is_obs_free;
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    is_obs_free = (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));

    return is_obs_free;
}

Vector3d World::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i World::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d World::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

float World::distance(const Point p, const Point q) { //Find the distance between two points.
    Point dist_v = p - q;
    return sqrt(powf(dist_v.m_x, 2) + powf(dist_v.m_y, 2) + powf(dist_v.m_z, 2));
}

Vector3d World::getglobalmaplb(){
    Vector3d globalmaplb(3);
    globalmaplb << this->gl_xl,this->gl_yl,this->gl_zl;
    return globalmaplb;}

Vector3d World::getglobalmapub(){
    Vector3d globalmapub(3);
    globalmapub << this->gl_xu,this->gl_yu,this->gl_zu;
    return globalmapub;}


