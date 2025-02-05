#include <ros/ros.h>
#include <rlss/RLSS.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <rlss_ros/Bezier.h>
#include <rlss_ros/dyn_params.h>
#include "../third_party/rlss/third_party/json.hpp"
#include <rlss/OccupancyGrid.hpp>
#include <fstream>
#include <ios>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Dense>
#include <rlss_ros/setTargetsConfig.h>
#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>


constexpr unsigned int DIM = DIMENSION;

//using namespace geometry_msgs;
//using namespace std;
//using namespace ros;
using namespace Eigen;
//using Eigen::Vector2d;

using RLSS = rlss::RLSS<double, DIM>;
using OccupancyGrid = rlss::OccupancyGrid<double, DIM>;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using Bezier = splx::Bezier<double, DIM>;
using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using VectorDIM = Bezier::VectorDIM;


// Trajectory Target
StdVectorVectorDIM goal_pose(DIM);
StdVectorVectorDIM current_pose(DIM);
int trajectory_target;
double velocity;
unsigned int number_of_drones;
double reach_distance;
double obs_check_distance;
double rescaling_factor;
unsigned int solver_type;
std_msgs::Int32 trigger_callback;
double buffer = 5.0;
double count = 0.0;
std::string mode;
std::string uav_1;
std::string uav_2;


ros::Publisher dp;
ros::Publisher planner_activation;
ros::Publisher tri;

bool reached_final_destination(const StdVectorVectorDIM& goal_pose, 
                            const StdVectorVectorDIM& current_pose, 
                            const double& reach_distance,
                            const double& number_of_drones){

    ROS_INFO_STREAM ("Initial Count " << count);
    
    std_msgs::Bool activation;

    if (trigger_callback.data == 0)
    {

        count = 0.0;

    };

    //ROS_INFO_STREAM (trigger_callback.data);
    if (count < number_of_drones)
    {
        ROS_INFO_STREAM ("Measurement time to final goal");
        for (unsigned int i = 0; i < number_of_drones; i++)
        {
            ROS_INFO_STREAM ("i is " << i);
            ROS_INFO_STREAM ("Norm for i is: " << (goal_pose[i]-current_pose[i]).norm());
            ROS_INFO_STREAM ("Measuring against reach distance");
            if((goal_pose[i]-current_pose[i]).norm() < reach_distance)
            {
                ROS_INFO_STREAM ("reach distance reached...");
                count += 1;
            }
        }
        if (count < number_of_drones)
        {
            count = 0.0;
        }
    }

    ROS_INFO_STREAM ("Count is " << count);
    ROS_INFO_STREAM ("Drone 1");
    ROS_INFO_STREAM ((goal_pose[0]-current_pose[0]).norm());
    ROS_INFO_STREAM (goal_pose[0]);
    ROS_INFO_STREAM ("Drone 2");
    ROS_INFO_STREAM ((goal_pose[1]-current_pose[1]).norm());
    ROS_INFO_STREAM (goal_pose[1]);

    if(count > (number_of_drones -1)) //for 2 drones, shud be (count > 1) but for this testing, I am gonna let count be only > 0 cos i am testing one drone first
        {
            ROS_INFO_STREAM ("Finished going to target");
            activation.data = false;
            planner_activation.publish(activation); 
            //trigger_callback.data = 0;
            //tri.publish(trigger_callback);
            return true;
        }
    else
        {
            ROS_INFO_STREAM ("Still going to target");
            activation.data = true;
            planner_activation.publish(activation);
            return false;
        }

    //count = 0.0; //resets the counter just in case
    
}

void triggerCallback(const std_msgs::Int32& msg)
    {
        trigger_callback = msg;
    }

void dynamicReconfigureCallback(rlss_ros::setTargetsConfig &config, uint32_t level)
{
    trajectory_target = config.trajectory_target;
    velocity = config.intended_velocity;
    reach_distance = config.reach_distance;
    number_of_drones = config.number_of_drones;
    goal_pose[0] << config.x_0, config.y_0, config.z_0;
    goal_pose[1] << config.x_1, config.y_1, config.z_1;
    obs_check_distance = config.optimization_obstacle_check_distance;
    rescaling_factor = config.rescaling_factor;
    solver_type = config.solver_type;

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_desired_target_feeder");
    ros::NodeHandle nh;
    nh.getParam("mode", mode);

    if (mode == "sim")
    {
        uav_1 = "/sim/uav1/mavros/local_position/pose";
        uav_2 = "/sim/uav2/mavros/local_position/pose";
    }
    else
    {
        uav_1 = "/uav1/mavros/local_position/pose";
        uav_2 = "/uav2/mavros/local_position/pose";
    }
    

    //dynamic reconfigure callback
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig> server;
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig>::CallbackType f;

    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);

    //subscription
    //switch(drone_env) rmb need to do this for individual drones 
    ros::Subscriber hover_pub_0 = nh.subscribe(uav_1, 10, hover0Callback);
    ros::Subscriber hover_pub_1 = nh.subscribe(uav_2, 10, hover1Callback);
    ros::Subscriber trigger_sub = nh.subscribe("/trigger", 10, triggerCallback);
    // dso convex hull algo would be added here

    //publishing
    ros::Publisher pt = nh.advertise<rlss_ros::PiecewiseTrajectory>("/Pseudo_trajectory", 10); //just added 
    dp = nh.advertise<rlss_ros::dyn_params>("/dyn_params",10);
    planner_activation = nh.advertise<std_msgs::Bool>("/planner_activation", 10);
    tri = nh.advertise<std_msgs::Int32>("/trigger",10);
    ros::Rate rate(10);

    //control_pts setup
    StdVectorVectorDIM trigger_pose(DIM);
    StdVectorVectorDIM starting_cpt(DIM);

    StdVectorVectorDIM testing_cpt(DIM);
    std::size_t count = 0;
    std::size_t anti_count = 0;
    Vector2d duration;
    rlss_ros::PiecewiseTrajectory pt_msg;

    while(ros::ok()){
        ros::spinOnce();

        //testing_cpt = current_pose;
        if (ros::Time::now().toSec() > buffer)
        { 
            rlss_ros::dyn_params dyn_msg;
            dyn_msg.number_of_drones = number_of_drones; 
            dyn_msg.reach_distance = reach_distance;
            dyn_msg.obs_check_distance = obs_check_distance;
            dyn_msg.rescaling_factor = rescaling_factor;
            dyn_msg.solver_type = solver_type;
            dyn_msg.intended_velocity = velocity;  
            dp.publish(dyn_msg);

            ROS_INFO_STREAM("current position of uav 1 is " << current_pose[0]);
            ROS_INFO_STREAM("current position of uav 2 is " << current_pose[1]);

            switch(trajectory_target){
            
            case 0:
                {
                trigger_callback.data = 0;
                //trigger_pose.clear();
                std_msgs::Bool activation;
                activation.data = false;
                planner_activation.publish(activation); 
                tri.publish(trigger_callback);
                ROS_INFO_STREAM ("Hovering");
                //use activation and trigger as the indication for planner 
                }
                break;
                
            case 1:
                {//pt_msg.pieces.clear();
                if(trigger_pose != goal_pose)
                {
                    ROS_INFO_STREAM ("Changing goal pose...");
                    trigger_pose = goal_pose;
                    trigger_callback.data = 0;
                    tri.publish(trigger_callback);
                }
                while (!reached_final_destination(goal_pose,current_pose,reach_distance,number_of_drones)){ 
                    //ROS_INFO_STREAM(trigger_callback.data);
                    if(trigger_callback.data < 1){
                        pt_msg.pieces.clear();
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
                        trigger_callback.data += 1;
                        ROS_INFO_STREAM ("Sending goal position...");

                        //tri.publish(trigger_callback);
                        //pt.publish(pt_msg);
                    }
                    else
                    {
                        break;
                    }
                    }
                //ROS_INFO_STREAM (duration[0]);
                tri.publish(trigger_callback);
                pt.publish(pt_msg);
                }
                break;
            } 
            if (trajectory_target == 0)
            {
                trigger_pose.clear();
            } 
        }      
        rate.sleep();
    }
    return 0;
}