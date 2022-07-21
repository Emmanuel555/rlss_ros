#include <ros/ros.h>
#include <rlss_ros/RobotState.h>
#include <rlss_ros/AABBCollisionShape.h>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/internal/Util.hpp>
#include <tf/transform_listener.h>
#include <rlss_ros/dyn_params.h>
#include <rlss_ros/Collision_Shape_Grp.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <stdlib.h> /*getenv*/

constexpr unsigned int DIM = DIMENSION;

using VectorDIM = rlss::internal::VectorDIM<double, DIM>;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using AABBCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using AlignedBox = rlss::internal::AlignedBox<double, DIM>;

//unsigned int self_robot_idx;
StdVectorVectorDIM state(DIM); // state now contains the position of all the drones involved
unsigned int number_of_drones;
std::string str_number_of_drones;
std::string other_agent;
VectorDIM testing(DIM);
std::shared_ptr<AABBCollisionShape> shape;
int robot_idx; 

//ros::Publisher self_state_publisher;
ros::Publisher collision_shape_grp_publisher;

void dynparamCallback(const rlss_ros::dyn_params::ConstPtr& msg){
    number_of_drones = msg->number_of_drones;
    str_number_of_drones = std::to_string(number_of_drones);
}

void hover0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) //by right for this callbacks, it shud be the other surrounding drones local position
{
    auto local_pos = *msg;
    state[robot_idx-1] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
    //state[1] << local_pos.pose.position.x , local_pos.pose.position.y, local_pos.pose.position.z;
    //state[1] << 2.0 , 0.0, local_pos.pose.position.z;
}

void hover1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    if (robot_idx == 1)
    {
        state[robot_idx] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
    }
    else
    {
        state[robot_idx-robot_idx] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
    }
}

void humanCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[2] << local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, std::string(getenv("DRONE_NAME")) + "_pose_callback_for_drones");
    ros::NodeHandle nh;

    //replanning period
    double replanning_period;
    nh.getParam("replanning_period", replanning_period);
    nh.getParam("robot_idx", robot_idx);
    auto id = std::to_string(robot_idx);

    if (robot_idx == 1)
    {
        other_agent = std::to_string(robot_idx+1);
    }
    else
    {
        other_agent = std::to_string(robot_idx-1);         
    }

    std::vector<double> colshape_min_vec, colshape_max_vec;
    nh.getParam("others_collision_shape_min", colshape_min_vec);
    nh.getParam("others_collision_shape_max", colshape_max_vec);
    VectorDIM colshape_min(DIM), colshape_max(DIM);
    for(unsigned int i = 0; i < DIM; i++) {
        colshape_min(i) = colshape_min_vec[i];
        colshape_max(i) = colshape_max_vec[i];
    }
    AlignedBox colshape_at_zero(colshape_min, colshape_max);
    shape = std::make_shared<AABBCollisionShape>(colshape_at_zero);

    //publishers
    //self_state_publisher = nh.advertise<rlss_ros::RobotState>("self_state", 1);
    collision_shape_grp_publisher = nh.advertise<rlss_ros::Collision_Shape_Grp>("/uav" + id + "/other_robot_collision_shapes", 10);

    //subscribers
    ros::Subscriber hover_pub_0 = nh.subscribe("/uav" + id + "/mavros/local_position/pose", 10, hover0Callback);
    ros::Subscriber hover_pub_1 = nh.subscribe("/uav" + other_agent + "/mavros/local_position/pose", 10, hover1Callback);
    ros::Subscriber human_pub = nh.subscribe("/HumanUWBUAV" + id + "Pose", 10, humanCallback);
    ros::Subscriber dynamicparams = nh.subscribe("/uav" + id + "/dyn_params", 10, dynparamCallback);
    ros::Rate rate(1/replanning_period);

    //rlss_ros msgs    
    rlss_ros::AABBCollisionShape cs_msg;
    rlss_ros::Collision_Shape_Grp cs_grp_msg;

    while(ros::ok()) {
        ros::spinOnce();
        for (unsigned int i = 0; i < number_of_drones; i++)
        {
            cs_msg.bbox.min.clear();
            cs_msg.bbox.max.clear();
            cs_msg.robot_idx = i;
            AlignedBox current_cs = shape->boundingBox(state[i]);
            //ROS_INFO_STREAM (current_cs.min()(0));
            //ROS_INFO_STREAM(state[i][0]);
            for(unsigned int d = 0; d < DIM; d++){
                cs_msg.bbox.min.push_back(current_cs.min()(d));
                cs_msg.bbox.max.push_back(current_cs.max()(d));
            }        
            cs_grp_msg.col_shapes.push_back(cs_msg);
        }
    
        auto between_human = (state[robot_idx-1] - state[2]).norm();
        auto between_drones = (state[1] - state[0]).norm();
	ROS_INFO_STREAM("Human Pose " << state[2]);
	ROS_INFO_STREAM("UAV Pose " << state[robot_idx-1]);        
	ROS_INFO_STREAM("between_human " << between_human);
	ROS_INFO_STREAM("between_drones " << between_drones);
        collision_shape_grp_publisher.publish(cs_grp_msg);
        cs_grp_msg.col_shapes.clear();
        
        rate.sleep();
    }

    return 0;
}
