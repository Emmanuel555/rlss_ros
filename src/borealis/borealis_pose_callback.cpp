#include <ros/ros.h>
#include <rlss_ros/RobotState.h>
#include <rlss_ros/AABBCollisionShape.h>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <rlss/internal/Util.hpp>
#include <tf/transform_listener.h>
#include <rlss_ros/dyn_params.h>
#include <rlss_ros/Collision_Shape_Grp.h>

constexpr unsigned int DIM = DIMENSION;

using VectorDIM = rlss::internal::VectorDIM<double, DIM>;
using StdVectorVectorDIM = rlss::internal::StdVectorVectorDIM<double, DIM>;
using AABBCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using AlignedBox = rlss::internal::AlignedBox<double, DIM>;

//unsigned int self_robot_idx;
StdVectorVectorDIM state; // state now contains the position of all the drones involved
unsigned int number_of_drones;

//ros::Publisher self_state_publisher;
ros::Publisher collision_shape_grp_publisher;

void dynparamCallback(const rlss_ros::dyn_params::ConstPtr& msg){
    number_of_drones = msg->number_of_drones;
}

void hover0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[0] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
}

void hover1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[1] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pose callback for drone 1 + 2");
    ros::NodeHandle nh;

    //publishers
    //self_state_publisher = nh.advertise<rlss_ros::RobotState>("self_state", 1);
    collision_shape_grp_publisher = nh.advertise<rlss_ros::Collision_Shape_Grp>("/other_robot_collision_shapes", 10);

    //subscribers
    ros::Subscriber hover_pub_0 = nh.subscribe("/uav0/mavros/local_position/pose", 10, hover0Callback);
    ros::Subscriber hover_pub_1 = nh.subscribe("/uav1/mavros/local_position/pose", 10, hover1Callback);
    ros::Subscriber dynamicparams = nh.subscribe("dyn_params", 10, dynparamCallback);
    ros::Rate rate(10);

    std::shared_ptr<AABBCollisionShape> shape;
    
    //rlss_ros msgs    
    rlss_ros::AABBCollisionShape cs_msg;
    rlss_ros::Collision_Shape_Grp cs_grp_msg;

    while(ros::ok()) {
        ros::spinOnce();
        for (unsigned int i = 0; i < number_of_drones; i++){
            cs_msg.robot_idx = i;
            for(unsigned int d = 0; d < DIM; d++){
                cs_msg.bbox.min.push_back((shape->boundingBox(state[i])).min()(d));
                cs_msg.bbox.max.push_back((shape->boundingBox(state[i])).max()(d));
            }        
            cs_grp_msg.col_shapes.push_back(cs_msg);
            }    
        collision_shape_grp_publisher.publish(cs_grp_msg);
        cs_grp_msg.col_shapes.clear();
        
        rate.sleep();
    }

    return 0;
}