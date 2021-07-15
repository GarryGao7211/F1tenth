// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

#include<eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;

//typedef std::pair<int,int> edge;
//std::vector<Node> path_to_goal;



class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Publisher drive_pub;
    ros::Publisher env_pub;
    ros::Publisher static_pub;
    ros::Publisher dynamic_pub;
    ros::Publisher env_viz_pub;
    ros::Publisher static_viz_pub;
    ros::Publisher dynamic_viz_pub;
    ros::Publisher tree_viz_pub;
    ros::Publisher path_viz_pub;
    ros::Publisher viz_pub; // published global goal
    ros::Publisher local_viz_pub; // local point to travel in path 


    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub;
    ros::Subscriber env_sub;
    ros::Subscriber static_sub;
    ros::Subscriber dynamic_sub;
    ros::Subscriber exeu_sub;


    // tf stuff
    tf::TransformListener listener;

    //map params
    float map_resolution;
    int map_width,map_height;
    geometry_msgs::Pose map_origin; // cell(0,0), [m, m, rad]
    float origin_x, origin_y;
    nav_msgs::MapMetaData all_map_metadata;
    nav_msgs::OccupancyGrid env_layer_msg;

    // underlying data structures
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> env_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> static_layer;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dynamic_layer;

    // laser params
    bool LASER_INIT;
    std::vector<float> angles_vector;
    std::vector<float> current_scan;
    int SCAN_COUNT;

    // occgrid params
    const int INFLATION = 2;


    // TODO: create RRT params
    //int max_iter = 100;
    double max_coord = 51.225;
    double min_coord = -51.225;
    //double step_size = 0.02;
    //threshold distance between lastest node and goal point
    double goal_threshold = 0.10;
    double lookhead = 1.5; // lookhead distance can be tuned
    double d; //distance between waypoint and robot coord in map
    double current_x;
    double current_y; // current waypoint in the loop
    
    std::vector<Node> path;
    std::vector<double> waypoint_x;
    std::vector<double> waypoint_y;
    std::vector<double> waypoint_angle;
    std::vector<double> waypoint_speed;

    // random generator, use this
    std::mt19937 gen;
    //std::uniform_real_distribution<double> x_dist;
    //std::uniform_real_distribution<double> y_dist;
    

    // rviz shown
    void env_callback(const nav_msgs::OccupancyGrid::ConstPtr& env_layer_msg);
    void static_callback(const nav_msgs::OccupancyGrid::ConstPtr& static_layer_msg);
    void dynamic_callback(const nav_msgs::OccupancyGrid::ConstPtr& dynamic_layer_msg);

    // mapping function
    std::vector<int> ind_2_rc(int ind);
    geometry_msgs::Point cell_2_coord(int ind);
    std::vector<int> coord_2_cell_rc(double x, double y);
    bool out_of_bounds(int r, int c);

    // callbacks
    // where rrt actually happens
    void pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg);
    //void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void exeu_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void pub_layers();

    // RRT methods
    std::vector<double> sample(double robot_x,double robot_y);

    // change from int to Node type
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point, std::vector<Node> &tree);
    bool check_layers_collision(int r, int c);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);

    // visualization for tree and path node
    void visualize_tree(std::vector<Node> &tree);
    void visualize_path(std::vector<Node> &path);

    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

