#include <ros/ros.h>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <rlss_ros/Bezier.h>
#include "../third_party/rlss/third_party/json.hpp"
#include <rlss/RLSS.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <fstream>
#include <ios>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <rlss_ros/setTargetsConfig.h>

constexpr unsigned int DIM = DIMENSION;

using namespace geometry_msgs;
using namespace std;
using namespace ros;
using namespace Eigen;
using Eigen::Vector4d;

using RLSS = rlss::RLSS<double, DIM>;
using OccupancyGrid = rlss::OccupancyGrid<double, DIM>;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using Bezier = splx::Bezier<double, DIM>;
using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using VectorDIM = Bezier::VectorDIM;

// Trajectory Target
StdVectorVectorDIM goal_pose;
StdVectorVectorDIM current_pose;
int trajectory_target = 0;
double velocity = 0;
int number_of_drones = 0;

void dynamicReconfigureCallback(rlss_ros::setTargetsConfig &config, uint32_t level){
    trajectory_target = config.trajectory_target;
    velocity = config.intended_velocity;
    number_of_drones = config.number_of_drones; 
    goal_pose[0] << config.x_0, config.y_0, config.z_0;
    goal_pose[1] << config.x_1, config.y_1, config.z_1;
}

void hover0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    current_pose[0] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
}

void hover1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    current_pose[1] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_desired_trajectory_feeder");
    ros::NodeHandle nh;

    //dynamic reconfigure callback
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig> server;
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    //subscription
    ros::Subscriber hover_pub_0 = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 10, hover0Callback);
    ros::Subscriber hover_pub_1 = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, hover1Callback);
    // dso convex hull algo would be added here

    //publishing
    ros::Publisher pt = nh.advertise<rlss_ros::PiecewiseTrajectory>("Pseudo_trajectory", 1); //just added 
    ros::Publisher duration_demo = nh.advertise<std_msgs::Float64>("duration", 1); //just added
    ros::Rate rate(10);

    //control_pts setup
    int starting_pt = 0;
    StdVectorVectorDIM starting_cpt;
    std::size_t count = 0;
    std::size_t anti_count = 0;
    Vector2d duration;
    std::vector<Bezier> bez_vec;

    //rlss_ros msgs
    rlss_ros::PiecewiseTrajectory Pt_msg;
    rlss_ros::Bezier bez_msg;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();   
        switch(trajectory_target){
        
        case 0:
            starting_pt = 0;
            starting_cpt.clear();

        case 1:
            if(starting_pt < 1){
                for(unsigned int d = 0; d < number_of_drones; d++){
                    starting_cpt[d] = current_pose[d];
                    duration[d] = (goal_pose[d] - starting_cpt[d]).norm()/velocity;
                    bez_msg.dimension = DIM;
                    bez_msg.duration = duration[d]; 
                    //bez_msg.starting_cpts = starting_cpt[d]; need to fking run a double for loop to include all the values
                    //bez_msg.goal_pose = goal_pose[d];
                    Pt_msg.pieces.push_back(bez_msg);
                    bez_msg.cpts.clear();
            } 
                starting_pt += 1;
                pt.publish(Pt_msg);
            }
        }    

    }
    return 0;
}