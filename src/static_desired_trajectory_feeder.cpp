#include <ros/ros.h>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <rlss_ros/PiecewiseTrajectory.h>
#include <rlss_ros/Bezier.h>
#include "../third_party/rlss/third_party/json.hpp"
#include <fstream>
#include <ios>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>

constexpr unsigned int DIM = DIMENSION;

using Bezier = splx::Bezier<double, DIM>;
using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using VectorDIM = Bezier::VectorDIM;

ros::Publisher pub;
rlss_ros::PiecewiseTrajectory msg;   //msg needs fixing 


bool setDesiredTrajectory(std_srvs::Empty::Request& req, std_srvs::Empty::Request& res) {
    pub.publish(msg);
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_desired_trajectory_feeder");
    ros::NodeHandle nh;
    ros::Publisher Bezpub = nh.advertise<rlss_ros::Bezier>("Bezier_trajectory", 1); //just added 
    ros::Publisher trajpub = nh.advertise<rlss_ros::PiecewiseTrajectory>("Pseudo_trajectory", 1); //just added 
    ros::Publisher duration_demo = nh.advertise<std_msgs::Float64>("duration", 1); //just added 
    ros::Rate rate(1);
    
    std::string robot_description_path;
    nh.getParam("robot_description_path", robot_description_path);

    std::fstream json_fs(robot_description_path, std::ios_base::in);
    nlohmann::json robot_desc = nlohmann::json::parse(json_fs);

    PiecewiseCurve traj;
        
    for(const auto& piece: robot_desc["original_trajectory"]["pieces"]) { // within original traj
        if(piece["type"] != "BEZIER") { //test with this tmr
            ROS_ERROR_STREAM("piece type should be bezier");
            return 0;
        }

        double duration = piece["duration"]; //duration
        duration_demo.publish(duration);
        //double extension = piece["duration"];
        ROS_INFO_STREAM (duration); // not looping ** need to fix this shit

        Bezier bez(duration);
        for(const auto& cpt: piece["control_points"]) {
            std::vector<double> cptv = cpt;
            if(cptv.size() != DIM) {
                ROS_ERROR_STREAM("cpt size does not match DIM");
                return 0;
            }
            VectorDIM cpt_vec;

            for(unsigned int i = 0; i < DIM; i++) {
                cpt_vec(i)= cptv[i];
            }
            bez.appendControlPoint(cpt_vec);
            //std::cout << bez.duration; // not looping ** need to fix this shit
            //std::cout << cpt_vec; 
            //ROS_INFO_STREAM (duration); // not looping ** need to fix this shit
            //std::cout << typeid(DIM).name() << std::endl;

        }

        traj.addPiece(bez); //total based on json file shud have 1 piece 
        //duration_demo.publish(duration);
        //std::cout << msg.pieces.duration; // not looping ** need to fix this shit 
        
    }

    pub = nh.advertise<rlss_ros::PiecewiseTrajectory>("desired_trajectory", 1);
    std::size_t count = 0;
    std::size_t anti_count = 0;
    while(ros::ok()){
        ros::spinOnce();
        //for(std::size_t i = 0; i < traj.numPieces(); i++) {
        
        if (count < traj.numPieces()){   
            const Bezier& bez = traj[count];

            rlss_ros::Bezier bez_msg;
            bez_msg.dimension = DIM;
            bez_msg.duration = bez.maxParameter();

            for(std::size_t j = 0; j < bez.numControlPoints(); j++) {

                for(unsigned int d = 0; d < DIM; d++) {
                    bez_msg.cpts.push_back(bez[j][d]); // inserts control point no. followed by the axis
                }
                ROS_INFO_STREAM (bez.numControlPoints()); // 2
            }
            ROS_INFO_STREAM (bez_msg.cpts.size()); // 6
            //ROS_INFO_STREAM (traj.numPieces());
            //ROS_INFO_STREAM (bez_msg.duration);
            //ROS_INFO_STREAM (bez_msg.dimension);
            //ROS_INFO_STREAM(typeid(bez_msg.cpts).name());
            //ROS_INFO_STREAM(typeid(msg.pieces).name());
            Bezpub.publish(bez_msg); // after inserting 

            //if (anti_count == count){
            msg.pieces.push_back(bez_msg);
            trajpub.publish(msg);
            //ROS_INFO_STREAM ("allahuahbak");
            //}    
            ros::ServiceServer service = nh.advertiseService("set_desired_trajectory", setDesiredTrajectory);
            anti_count++;
            //ROS_INFO_STREAM (count);
            //shud use while ros != shutdown to loop this, its only going thru one shot 
        }
    
     
        //trajpub.publish(msg);
        //ros::ServiceServer service = nh.advertiseService("set_desired_trajectory", setDesiredTrajectory);

        //ros::spin();
        rate.sleep();

    }
    return 0;
}