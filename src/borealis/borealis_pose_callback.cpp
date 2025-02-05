#include <ros/ros.h>
#include <rlss_ros/RobotState.h>
#include <rlss_ros/AABBCollisionShape.h>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/internal/Util.hpp>
#include <tf/transform_listener.h>
#include <rlss_ros/dyn_params.h>
#include <rlss_ros/Collision_Shape_Grp.h>
#include <geometry_msgs/PoseStamped.h>

constexpr unsigned int DIM = DIMENSION;

using VectorDIM = rlss::internal::VectorDIM<double, DIM>;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using AABBCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using AlignedBox = rlss::internal::AlignedBox<double, DIM>;

//unsigned int self_robot_idx;
StdVectorVectorDIM state(DIM); // state now contains the position of all the drones involved
unsigned int number_of_drones;
VectorDIM testing(DIM);
std::shared_ptr<AABBCollisionShape> shape;
std::string mode;
std::string uav_1;
std::string uav_2;


//ros::Publisher self_state_publisher;
ros::Publisher collision_shape_grp_publisher;

void dynparamCallback(const rlss_ros::dyn_params::ConstPtr& msg){
    number_of_drones = msg->number_of_drones;
}

void hover0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[0] << local_pos.pose.position.x, local_pos.pose.position.y - 2.0, local_pos.pose.position.z;
}

void hover1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[1] << local_pos.pose.position.x, local_pos.pose.position.y + 2.0, local_pos.pose.position.z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_callback_for_drones");
    ros::NodeHandle nh;

    std::vector<double> colshape_min_vec, colshape_max_vec;
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

    nh.getParam("collision_shape_at_zero_min", colshape_min_vec);
    nh.getParam("collision_shape_at_zero_max", colshape_max_vec);
    VectorDIM colshape_min(DIM), colshape_max(DIM);
    for(unsigned int i = 0; i < DIM; i++) {
        colshape_min(i) = colshape_min_vec[i];
        colshape_max(i) = colshape_max_vec[i];
    }
    AlignedBox colshape_at_zero(colshape_min, colshape_max);
    shape = std::make_shared<AABBCollisionShape>(colshape_at_zero);

    //publishers
    //self_state_publisher = nh.advertise<rlss_ros::RobotState>("self_state", 1);
    collision_shape_grp_publisher = nh.advertise<rlss_ros::Collision_Shape_Grp>("/other_robot_collision_shapes", 10);

    //subscribers
    ros::Subscriber hover_pub_0 = nh.subscribe(uav_1, 10, hover0Callback);
    ros::Subscriber hover_pub_1 = nh.subscribe(uav_2, 10, hover1Callback);
    ros::Subscriber dynamicparams = nh.subscribe("/dyn_params", 10, dynparamCallback);
    ros::Rate rate(10);

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
        collision_shape_grp_publisher.publish(cs_grp_msg);
        cs_grp_msg.col_shapes.clear();
        
        rate.sleep();
    }

    return 0;
}