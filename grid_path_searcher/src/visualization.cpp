#include "visualization.h"

//Visualize the generated path
void visRRTstarPath(vector<Vector3d> nodes ,ros::Publisher _RRTstar_path_vis_pub)
{
    visualization_msgs::Marker Points, Line ,Frame; 
    Frame.header.frame_id = Points.header.frame_id = Line.header.frame_id = "world";
    Frame.header.stamp    = Points.header.stamp    = Line.header.stamp    = ros::Time::now();
    Frame.ns              = Points.ns              = Line.ns              = "demo_node/RRTstarPath";
    Frame.action          = Points.action          = Line.action          = visualization_msgs::Marker::ADD;
    Frame.pose.orientation.w = Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id   = 1;
    Frame.id  = 2;

    Points.type = visualization_msgs::Marker::POINTS;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;
    Frame.type  = visualization_msgs::Marker::LINE_LIST;

    Frame.scale.x = 0.05;
    Points.scale.x = 0.1; 
    Points.scale.y = 0.1;
    Line.scale.x   = 0.1;

    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
    Points.color.a = 1.0;
    Line.color.b   = 1.0;
    Line.color.a   = 1.0;
    Frame.color.g  = 0.7;
    Frame.color.a  = 0.7;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
        Frame.points.push_back(pt);
    }
    //make frame
    vector<Vector3d> Frame_list;
    Frame_list = showframe(nodes);
    for(int i = 0; i < int(Frame_list.size()); i++)
    {
        Vector3d coord = Frame_list[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Frame.points.push_back(pt);
    }

    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
    _RRTstar_path_vis_pub.publish(Frame);
}

//Visualize the goal
void visRRTgoal(vector<Vector3d> nodes ,ros::Publisher _RRTstar_path_vis_pub)
{
    visualization_msgs::Marker Sphere, Line; 
    Sphere.header.frame_id = Line.header.frame_id = "world";
    Sphere.header.stamp    = Line.header.stamp    = ros::Time::now();
    Sphere.ns              = Line.ns              = "demo_node/RRTgoal";
    Sphere.action          = Line.action          = visualization_msgs::Marker::ADD;
    Sphere.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Sphere.id = 0;
    Line.id   = 1;
    Sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;

    Sphere.scale.x = 0.4; 
    Sphere.scale.y = 0.4; 
    Sphere.scale.z = 0.4; 

    Line.scale.x   = 0.1;

    //points are green and Line Strip is blue
    Sphere.color.r = 1.0f;
    Sphere.color.a = 1.0;
    Line.color.b   = 1.0;
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Sphere.points.push_back(pt);
        //Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Sphere);
    _RRTstar_path_vis_pub.publish(Line); 
}


vector<Vector3d> showframe(vector<Vector3d> nodes){
    double frame_w = 0.75;
    double frame_h = 0.36;
    Vector3d vectorBefore(1, 0, 0);

    vector<Vector3d> Frame_list;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    geometry_msgs::Point p3;
    geometry_msgs::Point p4;
    

    for(int i = 0; i < int(nodes.size()); i++)
    {
        if(i < int(nodes.size())-1){
            Vector3d coord = nodes[i];
            Vector3d p1_tmp,p2_tmp,p3_tmp,p4_tmp;
            Vector3d p1_,p2_,p3_,p4_;
            p1_tmp(0) = 0;p1_tmp(1) = +frame_w/2;p1_tmp(2) = +frame_h/2;
            p2_tmp(0) = 0;p2_tmp(1) = -frame_w/2;p2_tmp(2) = +frame_h/2;
            p3_tmp(0) = 0;p3_tmp(1) = +frame_w/2;p3_tmp(2) = -frame_h/2;
            p4_tmp(0) = 0;p4_tmp(1) = -frame_w/2;p4_tmp(2) = -frame_h/2;
            
            p1_ = calculate_rotmatrix(vectorBefore,(nodes[i+1]-nodes[i]).normalized(),p1_tmp) + nodes[i];
            p2_ = calculate_rotmatrix(vectorBefore,(nodes[i+1]-nodes[i]).normalized(),p2_tmp) + nodes[i];
            p3_ = calculate_rotmatrix(vectorBefore,(nodes[i+1]-nodes[i]).normalized(),p3_tmp) + nodes[i];
            p4_ = calculate_rotmatrix(vectorBefore,(nodes[i+1]-nodes[i]).normalized(),p4_tmp) + nodes[i];
            
            Frame_list.push_back(p1_);Frame_list.push_back(p2_);
            Frame_list.push_back(p2_);Frame_list.push_back(p4_);
            Frame_list.push_back(p4_);Frame_list.push_back(p3_);
            Frame_list.push_back(p3_);Frame_list.push_back(p1_);
        }
        
    }
    return Frame_list;
}

Vector3d calculate_rotmatrix(Vector3d vectorBefore,Vector3d vectorAfter,Vector3d p_tmp){
    Eigen::Matrix3d rotMatrix;
    rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();
    return rotMatrix * p_tmp;
}

