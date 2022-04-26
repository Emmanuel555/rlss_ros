#include "ros/ros.h"
#include <rlss_ros/OccupancyGrid.h>
#include <rlss/OccupancyGrid.hpp>
#include <fstream>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Bool.h>

namespace fs = boost::filesystem;
constexpr unsigned int DIM = DIMENSION;

using OccupancyGrid = rlss::OccupancyGrid<double, DIM>;
using OccIndex = OccupancyGrid::Index;
using OccCoordinate = OccupancyGrid::Coordinate;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using VectorDIM = OccupancyGrid::VectorDIM;
using Ellipsoid = rlss::Ellipsoid<double, DIM>;
using MatrixDIMDIM = rlss::internal::MatrixRC<double, DIM, DIM>;
sensor_msgs::PointCloud pcl; 
std_msgs::Bool activation;
unsigned int robot_idx; 

void plannerCallback(const std_msgs::Bool::ConstPtr& msg)
{
    activation = *msg;
}

void occupancyGridCallback(const sensor_msgs::PointCloud::ConstPtr &msg)// need to check if drone 2 takes off at 0,0,0 or offset based on gps
{
    //ROS_INFO_STREAM("occ grid callback up...");
    pcl = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_map_feeder_1");
    ros::NodeHandle nh;

    //replanning period
    double replanning_period;
    nh.getParam("replanning_period", replanning_period);


    //data_cap for no. of points in map
    //double data_cap;
    //nh.getParam("data_cap", data_cap);

    
    // current location to attain obstacles
    std::string obstacles_directory;
    nh.getParam("obstacles_directory", obstacles_directory);

    nh.getParam("robot_idx", robot_idx);
    auto id = std::to_string(robot_idx);


    // 
    std::vector<double> step_size;
    nh.getParam("occupancy_grid_step_size", step_size);
    if(step_size.size() != DIM) { // shud be (0.5, 0.5, 0.5)
        ROS_FATAL_STREAM("occupancy grid step size dimension" + std::to_string(step_size.size()) + " does not match dimension " + std::to_string(DIM));
        return 0;
    }
    OccCoordinate occ_step_size;
    for(std::size_t i = 0; i < DIM; i++) {
        occ_step_size(i) = step_size[i]; //{0.5, 0,5 0,5} 
    }

    ROS_INFO_STREAM(occ_step_size.transpose());
    OccupancyGrid occupancy_grid(occ_step_size);
    boost::filesystem::path p(obstacles_directory);
    //std::cout << std::endl;

    /* for(auto& p: fs::directory_iterator(obstacles_directory)) {
        ROS_INFO_STREAM(p.path().string());
        std::fstream obstacle_file(p.path().string(), std::ios_base::in);
        std::string type;
        obstacle_file >> type;
        if(type == "cvxhull") {
            StdVectorVectorDIM obstacle_pts;
            VectorDIM vec;
            while (obstacle_file >> vec(0)) {
                for(unsigned int d = 1; d < DIM; d++)
                    obstacle_file >> vec(d);
                obstacle_pts.push_back(vec);
            }
            occupancy_grid.addObstacle(obstacle_pts);
        } else if(type == "ellipsoid") {
            VectorDIM center;
            MatrixDIMDIM mtr;

            for(unsigned int i = 0; i < DIM; i++) {
                obstacle_file >> center(i);
            }

            for(unsigned int r = 0; r < DIM; r++) {
                for(unsigned int c = 0; c < DIM; c++) {
                    obstacle_file >> mtr(r, c);
                }
            }
            Ellipsoid ell(center, mtr);
            occupancy_grid.addObstacle(ell);
        }
    } */

    //Planner usage
    ros::Subscriber planner_sub = nh.subscribe("/planner_activation_" + id, 1, plannerCallback);

    //pcl subscription
    ros::Subscriber occgridsub = nh.subscribe("occupancy_map/visualize_pointcloud", 1, occupancyGridCallback);
    
    //internal occupancy grid rlss pub
    ros::Publisher pub = nh.advertise<rlss_ros::OccupancyGrid>("occupancy_grid_" + id, 1);
    

    ros::Rate rate(1/replanning_period);
    while(ros::ok()) {
        ros::spinOnce();
        rlss_ros::OccupancyGrid msg;

       /*  switch (activation.data) //trigger
        {
                
        case false:           
        { */
            //auto counter_1 = 0.0;

        for (auto&pts : pcl.points)
        {
            //if (counter_1 < data_cap){
            //ROS_INFO_STREAM("Accessing data points...");
            occupancy_grid.setOccupancy(OccCoordinate(pts.x,pts.y,pts.z));
            //}
            //counter_1 += 1.0;
        }   
        ROS_INFO_STREAM(pcl.points.size());
        ROS_INFO_STREAM("Inserting obstacles");
        for(unsigned int i = 0; i < DIM; i++) {
            msg.step_size.push_back(occ_step_size(i));
        }

        const auto& index_set = occupancy_grid.getIndexSet(); // getting the occupied indexes from the grids

        for(const auto& idx: index_set) {
            for(unsigned int i = 0; i < DIM; i++) {
                msg.occupied_indexes.push_back(idx(i));
            }
        }

        if (pcl.points.size() != 0)
        {
            pub.publish(msg);
        }
        else
        {
            ROS_WARN_STREAM("occ grid map encountered errors");
        }

        //auto counter_2 = 0.0;

        for (auto&pts : pcl.points)
        {
            //if (counter_2 < data_cap){
            occupancy_grid.removeOccupancy(OccCoordinate(pts.x, pts.y, pts.z));
            //}
            //counter_2 += 1.0;
        }

        /*     break;

        }
        */
        
        /* case true:
        {
            ROS_INFO_STREAM("Not inserting obstacles"); // rostopic msg only holds the obstacles saved during the prev collection
            break;

        }
        } */
    
        rate.sleep();
        
    }

    return 0;
}
