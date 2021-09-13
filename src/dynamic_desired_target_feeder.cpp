#include <ros/ros.h>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <rlss_ros/Bezier.h>
#include <rlss_ros/dyn_params.h>
#include "../third_party/rlss/third_party/json.hpp"
#include <rlss/RLSS.hpp>
#include <rlss/OccupancyGrid.hpp>
#include <fstream>
#include <ios>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <rlss_ros/setTargetsConfig.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>

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
unsigned int number_of_drones = 0;
double reach_distance;
double obs_check_distance;
double rescaling_factor;
double intended_velocity;
unsigned int solver;

ros::Publisher dp;

bool reached_final_destination(const StdVectorVectorDIM& goal_pose, 
                            const StdVectorVectorDIM& current_pose, 
                            const double& reach_distance,
                            const double& number_of_drones){
    for (unsigned int i = 0; i < number_of_drones; i++)
    {
        double count = 0;
        if((goal_pose[i]-current_pose[i]).norm() < reach_distance)
        {
            count += 1;
       
        }
    }


}

void dynamicReconfigureCallback(rlss_ros::setTargetsConfig &config, uint32_t level){
    rlss_ros::dyn_params dyn_msg;

    trajectory_target = config.trajectory_target;
    velocity = config.intended_velocity;
    goal_pose[0] << config.x_0, config.y_0, config.z_0;
    goal_pose[1] << config.x_1, config.y_1, config.z_1;

    dyn_msg.number_of_drones = config.number_of_drones; 
    dyn_msg.reach_distance = config.reach_distance;
    reach_distance = config.reach_distance;
    dyn_msg.obs_check_distance = config.optimization_obstacle_check_distance;
    dyn_msg.rescaling_factor = config.rescaling_factor;
    dyn_msg.solver_type = config.solver_type;
    dyn_msg.intended_velocity = config.intended_velocity;

    dp.publish(dyn_msg);

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
    ros::init(argc, argv, "dynamic_desired_target_feeder");
    ros::NodeHandle nh;

    //dynamic reconfigure callback
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig> server;
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    //subscription
    ros::Subscriber hover_pub_0 = nh.subscribe("/uav0/mavros/local_position/pose", 10, hover0Callback);
    ros::Subscriber hover_pub_1 = nh.subscribe("/uav1/mavros/local_position/pose", 10, hover1Callback);
    // dso convex hull algo would be added here

    //publishing
    ros::Publisher pt = nh.advertise<rlss_ros::PiecewiseTrajectory>("Pseudo_trajectory", 10); //just added 
    dp = nh.advertise<rlss_ros::dyn_params>("dyn_params",10);
    ros::Publisher planner_activation = nh.advertise<std_msgs::Bool>("planner_activation", 1);
    ros::Rate rate(10);

    //control_pts setup
    int trigger = 0;
    StdVectorVectorDIM trigger_pose;
    StdVectorVectorDIM starting_cpt;
    std::size_t count = 0;
    std::size_t anti_count = 0;
    Vector2d duration;

    while(ros::ok()){
        ros::spinOnce(); 

        if(trigger_pose != goal_pose){
            trigger_pose = goal_pose;
            trigger = 0;
        }

        switch(trajectory_target){
        
        case 0:
    
            trigger = 0;
            trigger_pose.clear();
            std_msgs::Bool activation;
            activation.data = false;
            planner_activation.publish(activation); 
            ROS_INFO_STREAM ("Hovering");

            break;
            
        case 1:
            //pt_msg.pieces.clear();
            if (((goal_pose[0] - current_pose[0]).norm() && (goal_pose[1] - starting_cpt[1]).norm())  > reach_distance){ 
                std_msgs::bool activation;
                activation.data = true;
                planner_activation.publish(activation); 
                if(trigger < 1){
                    rlss_ros::PiecewiseTrajectory pt_msg;
                    pt_msg.start_time.data = ros::Time::now();
                    for(unsigned int d = 0; d < number_of_drones; d++){
                        rlss_ros::Bezier bez_msg;
                        starting_cpt[d] = current_pose[d];
                        duration[d] = (goal_pose[d] - starting_cpt[d]).norm()/velocity;
                        bez_msg.dimension = DIM;
                        bez_msg.duration = duration[d];
                        for (std::size_t i = 0; i < DIM; i++){
                            bez_msg.start.push_back(starting_cpt[d][i]);
                            bez_msg.end.push_back(goal_pose[d][i]);
                        } 
                        pt_msg.pieces.push_back(bez_msg);
                        //bez_msg.start.clear();
                        //bez_msg.end.clear();
                    } 
                    trigger += 1;
                    pt.publish(pt_msg);
                }
                ROS_INFO_STREAM ("Going to target");
            }
            else{
                std_msgs::bool activation;
                activation.data = false;
                planner_activation.publish(activation); 
            }
            
            break;

        }    
        rate.sleep();
    }
    return 0;
}