#include <ros/ros.h>
#include <rlss/RLSS.hpp>
#include <rlss/Borealis_Planning.hpp>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <vector>
#include <rlss_ros/AABB.h>
#include <rlss_ros/AABBCollisionShape.h>
#include <rlss_ros/RobotState.h>
#include <rlss_ros/Bezier.h>
#include <rlss_ros/OccupancyGrid.h>
#include <rlss_ros/PiecewiseTrajectory.h>
#include "../../third_party/rlss/third_party/json.hpp"
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <splx/curve/PiecewiseCurve.hpp>
#include <splx/curve/Bezier.hpp>
#include <rlss/TrajectoryOptimizers/RLSSHardOptimizer.hpp>
#include <rlss/TrajectoryOptimizers/RLSSSoftOptimizer.hpp>
#include <rlss/TrajectoryOptimizers/RLSSHardSoftOptimizer.hpp>
#include <rlss/DiscretePathSearchers/RLSSDiscretePathSearcher.hpp>
#include <rlss/ValidityCheckers/RLSSValidityChecker.hpp>
#include <rlss/GoalSelectors/RLSSGoalSelector.hpp>
#include <rlss/GoalSelectors/BorealisGoalSelector.hpp>
#include <std_msgs/Time.h>
#include <rlss_ros/setTargetsConfig.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <rlss_ros/Collision_Shape_Grp.h>
#include <std_msgs/Bool.h>
#include <rlss_ros/dyn_params.h>
#include <rlss_ros/Collision_Shape_Grp.h>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud.h>
#include <stdlib.h> /*getenv*/

namespace fs = boost::filesystem;
constexpr unsigned int DIM = DIMENSION;

using RLSS = rlss::RLSS<double, DIMENSION>;
using Borealis_Planning = rlss::Borealis_Planning<double,DIMENSION>;
using CollisionShape = rlss::CollisionShape<double, DIM>;
using AlignedBoxCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using OccupancyGrid = rlss::OccupancyGrid<double, DIM>;
using OccIndex = OccupancyGrid::Index;
using OccCoordinate = OccupancyGrid::Coordinate;
using StdVectorVectorDIM = OccupancyGrid::StdVectorVectorDIM;
using AlignedBox = OccupancyGrid::AlignedBox;
using VectorDIM = RLSS::VectorDIM;
using PiecewiseCurveQPGenerator = splx::PiecewiseCurveQPGenerator<double, DIM>;
using PiecewiseCurve = splx::PiecewiseCurve<double, DIM>;
using Bezier = splx::Bezier<double, DIM>;
using RLSSHardOptimizer = rlss::RLSSHardOptimizer<double, DIM>;
using RLSSSoftOptimizer = rlss::RLSSSoftOptimizer<double, DIM>;
using RLSSHardSoftOptimizer = rlss::RLSSHardSoftOptimizer<double, DIM>;
using RLSSDiscretePathSearcher = rlss::RLSSDiscretePathSearcher<double, DIM>;
using RLSSValidityChecker = rlss::RLSSValidityChecker<double, DIM>;
using RLSSGoalSelector = rlss::RLSSGoalSelector<double, DIM>;
using BorealisGoalSelector = rlss::BorealisGoalSelector<double, DIM>;
using TrajectoryOptimizer = rlss::TrajectoryOptimizer<double, DIM>;
using DiscretePathSearcher = rlss::DiscretePathSearcher<double, DIM>;
using ValidityChecker = rlss::ValidityChecker<double, DIM>;
using GoalSelector = rlss::GoalSelector<double, DIM>;
using Ellipsoid = rlss::Ellipsoid<double, DIM>;
using MatrixDIMDIM = rlss::internal::MatrixRC<double, DIM, DIM>;

int robot_idx; 
std::map<unsigned int, AlignedBox> other_robot_collision_shapes;  // robot_idx -> colshape
OccupancyGrid global_occupancy_grid(OccCoordinate(0.5,0.5,0.5)); // default values
//ros::Time desired_trajectory_set_time = ros::Time(0);
std_msgs::Time desired_trajectory_set_time;
PiecewiseCurve desired_trajectory;
StdVectorVectorDIM state(DIM);// this one needs to be changed
StdVectorVectorDIM current_pose(DIM);
unsigned int continuity_upto_degree;
double testing;
std_msgs::Bool activation;
std::string optimizer;
VectorDIM starting(DIM);
VectorDIM ending(DIM);
std::vector<geometry_msgs::Point32> pcl_pts;
sensor_msgs::PointCloud pcl;
VectorDIM durations(DIM);


// Dynamic Planner Params
double reach_distance;
double optimization_obstacle_check_distance;
unsigned int solver_type;
unsigned int number_of_drones;

// Duration and rescaling coeff
double rescaling_multiplier;
double intended_velocity;


void otherRobotShapeCallback(const rlss_ros::Collision_Shape_Grp::ConstPtr &msg) // each robot needs to have its own shape topic
{   
    for (const auto &c_s : msg->col_shapes){
        unsigned int robot_c_idx = c_s.robot_idx;
        VectorDIM shape_min(DIM), shape_max(DIM);
        for (unsigned int i = 0; i < DIM; i++)
        {
            shape_min(i) = c_s.bbox.min[i]; // -0.6, 0.6, 0
            shape_max(i) = c_s.bbox.max[i]; // 0.6, 0.6, 0.25
        }
        other_robot_collision_shapes[robot_c_idx] = AlignedBox(shape_min, shape_max); // if its 0,0,0, cannot work with this current setup
    }
} // other robot shapes


void hover0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[0] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
}


/*void hover1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    auto local_pos = *msg;
    state[1] << local_pos.pose.position.x, local_pos.pose.position.y, local_pos.pose.position.z;
}*/


void plannerCallback(const std_msgs::Bool::ConstPtr& msg)
{
    activation = *msg;
}

void desiredTrajectoryCallback(const rlss_ros::PiecewiseTrajectory::ConstPtr &msg)
{
    PiecewiseCurve curve;
    //std::vector<std::optional<PiecewiseCurve>> curve;
    for (std::size_t i = 0; i < msg->pieces.size(); i++)
    {
        rlss_ros::Bezier piece_msg = msg->pieces[i];
        if (piece_msg.dimension != DIM)
        {
            ROS_FATAL_STREAM("bez dimension is not correct");
            return;
        }
        else
        {
            Bezier bez(piece_msg.duration);
            durations[i] = piece_msg.duration;
            VectorDIM starting_cpt(DIM);
            VectorDIM end_cpt(DIM);
            for (unsigned int j = 0; j < DIM; j++)
            {    
                starting_cpt[j] = piece_msg.start[j];
                end_cpt[j] = piece_msg.end[j];
                //ROS_INFO_STREAM (end_cpt[j]);
                starting[j] = piece_msg.start[j];
                ending[j] = piece_msg.end[j];
            }
            bez.appendControlPoint(starting_cpt);
            bez.appendControlPoint(end_cpt);
            curve.addPiece(bez);
        }
        
    }
    desired_trajectory = curve;
    desired_trajectory_set_time = msg->start_time;
}

void occupancyGridCallback(const rlss_ros::OccupancyGrid::ConstPtr &msg)// need to check if drone 2 takes off at 0,0,0 or offset based on gps
{    
    if (msg->step_size.size() != DIM)
    {
        ROS_FATAL_STREAM("occupancy grid dimensions not correct");
    }
    else
    {
        OccCoordinate step_size(DIM);
        for (unsigned int i = 0; i < DIM; i++)
        {
            step_size(i) = msg->step_size[i];
        }

        auto occupancy_grid = OccupancyGrid(step_size);
        for (std::size_t i = 0; i < msg->occupied_indexes.size() / DIM; i++)
        {
            OccIndex index(DIM);
            for (std::size_t j = 0; j < DIM; j++)
            {
                index(j) = msg->occupied_indexes[i * DIM + j];
            }
            occupancy_grid.setOccupancy(index);
        }
        global_occupancy_grid = occupancy_grid;
    }
    
}

void dynparamCallback(const rlss_ros::dyn_params::ConstPtr& msg){
    number_of_drones = msg->number_of_drones;
    reach_distance = msg->reach_distance;
    solver_type = msg->solver_type;
    optimization_obstacle_check_distance = msg->obs_check_distance;
    rescaling_multiplier = msg->rescaling_factor;
    intended_velocity = msg->intended_velocity;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, std::string(getenv("DRONE_NAME")) + "_borealis_planner");
    ros::NodeHandle nh;

    
    //required params from launch file*********************************************
    //nh.getParam("robot_idx", self_robot_idx);
    

    //continuity upto deg
    int c_upto_d;
    nh.getParam("continuity_upto_degree", c_upto_d);
    continuity_upto_degree = c_upto_d;
    state.resize(continuity_upto_degree + 1);
    

    //replanning period
    double replanning_period;
    nh.getParam("replanning_period", replanning_period);
    //std::cout << "replanning_period: " << replanning_period << std::endl;

    double actual_timestamp;
    nh.getParam("actual_timestamp", actual_timestamp);

    bool recording;
    nh.getParam("recording", recording);

    nh.getParam("robot_idx", robot_idx);
    auto id = std::to_string(robot_idx);

    //max derivative mag
    std::vector<int> maximum_derivative_magnitude_degrees(DIM);
    std::vector<double> maximum_derivative_magnitude_magnitudes(DIM);
    nh.getParam("maximum_derivative_magnitude_degrees", maximum_derivative_magnitude_degrees);
    nh.getParam("maximum_derivative_magnitude_magnitudes", maximum_derivative_magnitude_magnitudes);
    if (maximum_derivative_magnitude_degrees.size() != maximum_derivative_magnitude_magnitudes.size())
    {
        ROS_FATAL_STREAM("maximum derivative magnitude degree magnitude mismatch");
        return 0;
    }
    std::vector<std::pair<unsigned int, double>> maximum_derivative_magnitudes(continuity_upto_degree);
    for (std::size_t i = 0; i < maximum_derivative_magnitude_degrees.size(); i++)
    {
        maximum_derivative_magnitudes[i].first = maximum_derivative_magnitude_degrees[i],
        maximum_derivative_magnitudes[i].second = maximum_derivative_magnitude_magnitudes[i];
    }


    //workspace size 
    std::vector<double> workspace_min_vec(DIM), workspace_max_vec(DIM);
    nh.getParam("workspace_min", workspace_min_vec);
    nh.getParam("workspace_max", workspace_max_vec);
    if (workspace_min_vec.size() != workspace_max_vec.size())
    {
        ROS_FATAL_STREAM("workspace min-max size mismatch");
        return 0;
    }
    if (workspace_min_vec.size() != DIM)
    {
        ROS_FATAL_STREAM("workspace vector size not equal to dimension");
        return 0;
    }
    VectorDIM workspace_min(DIM), workspace_max(DIM);
    for (unsigned int i = 0; i < DIM; i++)
    {
        workspace_min(i) = workspace_min_vec[i];
        workspace_max(i) = workspace_max_vec[i];
    }
    AlignedBox workspace(workspace_min, workspace_max);


    //UAV collision shape
    std::vector<double> colshape_min_vec(DIM), colshape_max_vec(DIM);
    nh.getParam("collision_shape_at_zero_min", colshape_min_vec);
    nh.getParam("collision_shape_at_zero_max", colshape_max_vec);
    if (colshape_min_vec.size() != colshape_max_vec.size())
    {
        ROS_FATAL_STREAM("colshape min-max size mismatch");
        return 0;
    }
    if (colshape_min_vec.size() != DIM)
    {
        ROS_FATAL_STREAM("colshape vector size not equal to dimension");
        return 0;
    }
    VectorDIM colshape_min(DIM), colshape_max(DIM);
    for (unsigned int i = 0; i < DIM; i++)
    {
        colshape_min(i) = colshape_min_vec[i];
        colshape_max(i) = colshape_max_vec[i];
    }
    AlignedBox colshape_at_zero(colshape_min, colshape_max);
    auto aabb_col_shape = std::make_shared<AlignedBoxCollisionShape>(
        colshape_at_zero);
    auto self_col_shape = std::static_pointer_cast<CollisionShape>(
        aabb_col_shape);


    //soft optimisation SVM params for r2r, r2o, ipc, cc/////////////////////
    
    const std::vector<std::string> soft_opt_param_names{
        "robot_to_robot_hyperplane_constraints",
        "robot_to_obstacle_hyperplane_constraints",
        "initial_point_constraints",
        "continuity_constraints"};
    std::unordered_map<std::string, std::pair<bool, double>>
        soft_optimization_parameters;
    for (const auto &opt_param_name : soft_opt_param_names)
    {
        bool enabled;
        double value;
        nh.getParam("soft_optimization_parameters/" + opt_param_name + "_enabled", enabled);
        nh.getParam("soft_optimization_parameters/" + opt_param_name + "_value", value);
        soft_optimization_parameters[opt_param_name] = std::make_pair(enabled, value);
    }


    //soft-hard, hard, soft
    //std::string optimizer;
    //switch (solver_type)
    //{
    //case 0:
    //    optimizer = "rlss-hard-soft";
    //    break;
    //case 1:
    //    optimizer = "rlss-soft";
    //    break;
    //case 2:
    //    optimizer = "rlss-hard";
    //    break;
   // }


    //end point cost weights
    std::vector<double> piece_endpoint_cost_weights(DIM);
    nh.getParam("piece_endpoint_cost_weights", piece_endpoint_cost_weights);


    //integrated squared derivative weights
    std::vector<int> integrated_squared_derivative_weight_degrees(DIM);
    std::vector<double> integrated_squared_derivative_weight_weights(DIM);
    nh.getParam("integrated_squared_derivative_weight_degrees", integrated_squared_derivative_weight_degrees);
    nh.getParam("integrated_squared_derivative_weight_weights", integrated_squared_derivative_weight_weights);
    if (integrated_squared_derivative_weight_weights.size() != integrated_squared_derivative_weight_degrees.size())
    {
        ROS_FATAL_STREAM("integrated squared derivative weights size mismatch");
        return 0;
    }
    std::vector<std::pair<unsigned int, double>>
        integrated_squared_derivative_weights(DIM);
    for (std::size_t i = 0; i < integrated_squared_derivative_weight_degrees.size(); i++)
    {
        integrated_squared_derivative_weights[i].first = integrated_squared_derivative_weight_degrees[i];
        integrated_squared_derivative_weights[i].second = integrated_squared_derivative_weight_weights[i];
    }

    
    //Planned for a Time horizon of 8
    double desired_time_horizon;
    nh.getParam("desired_time_horizon", desired_time_horizon);


    //how many times can the recalculation be done
    unsigned int max_rescaling_count;
    int m_r_c;
    nh.getParam("max_rescaling_count", m_r_c);
    max_rescaling_count = m_r_c;


    //duration multiplier
    //double rescaling_multiplier;
    //nh.getParam("rescaling_multiplier", rescaling_multiplier);

    //data_cap for no. of points in map
    double data_cap;
    nh.getParam("data_cap", data_cap);


    //search step for validity 
    double search_step;
    nh.getParam("search_step", search_step);


    //Time horizon divided by 4, where each piece has 8 control points
    std::vector<int> num_bezier_control_points;
    nh.getParam("num_bezier_control_points", num_bezier_control_points);


    PiecewiseCurveQPGenerator qp_generator;
    for (const auto &cpts : num_bezier_control_points)
    {
        qp_generator.addBezier(cpts, 0);
    }


    /**************Occupancy grid generator ****************/
    std::string obstacles_directory;
    nh.getParam("obstacles_directory", obstacles_directory);

    std::vector<double> step_size;
    nh.getParam("occupancy_grid_step_size", step_size);
    if(step_size.size() != DIM) {
        ROS_FATAL_STREAM("occupancy grid step size dimension" + std::to_string(step_size.size()) + " does not match dimension " + std::to_string(DIM));
        return 0;
    }
    
    OccCoordinate occ_step_size;
    for(std::size_t i = 0; i < DIM; i++) {
        occ_step_size(i) = step_size[i];
    }

    //ROS_INFO_STREAM(occ_step_size.transpose());
    //OccupancyGrid occupancy_grid(occ_step_size);
    //boost::filesystem::path p(obstacles_directory);
    //occupancy_grid.setOccupancy(OccCoordinate(2.0,4.0,1.0));

    /*for(auto& p: fs::directory_iterator(obstacles_directory)) {
        //ROS_INFO_STREAM(p.path().string());
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
    }*/


    /* Publishers and Subscribers **********/

    //collision shapes
    ros::Subscriber colshapesub = nh.subscribe("/uav" + id + "/other_robot_collision_shapes", 10, otherRobotShapeCallback);
    

    //mavros subscription
    ros::Subscriber hover_pub_0 = nh.subscribe("/uav" + id + "/mavros/local_position/pose", 10, hover0Callback);
    //ros::Subscriber hover_pub_1 = nh.subscribe("/uav1/mavros/local_position/pose", 10, hover1Callback);
    
        
    //Current drone's intended goal and starting cpt/position
    ros::Subscriber destrajsub = nh.subscribe("/uav" + id + "/Pseudo_trajectory", 1, desiredTrajectoryCallback);

    //Planner usage
    ros::Subscriber planner_sub = nh.subscribe("/uav" + id + "/planner_activation", 1, plannerCallback);

    //Dynamic Params
    ros::Subscriber dynamicparams = nh.subscribe("/uav" + id + "/dyn_params", 10, dynparamCallback);
    
    //Occupancy grid of current drone
    //ros::Subscriber occgridsub = nh.subscribe("/occupancy_map/occupancy_pointcloud", 1, occupancyGridCallback);
    ros::Subscriber occgridsub = nh.subscribe("/uav" + id + "/occupancy_grid", 1, occupancyGridCallback);
    
    //Push the position of where this drone shud go into topic 
    ros::Publisher trajpub = nh.advertise<rlss_ros::PiecewiseTrajectory>("/uav" + id + "/final_trajectory", 1);
    //ros::Publisher trajpub_1 = nh.advertise<geometry_msgs::PoseStamped>("/final_trajectory_pose_1", 1);
    ros::Publisher trajwhole_0 = nh.advertise<nav_msgs::Path>("/uav" + id + "/whole_trajectory_pose", 1);
    ros::Publisher trajpub_0 = nh.advertise<geometry_msgs::PoseStamped>("/uav" + id + "/final_trajectory_pose", 1);
    //ros::Publisher currentpub_0 = nh.advertise<geometry_msgs::PoseStamped>("/current_trajectory_pose_0", 1);


    //loops at the rate of the replanning period
    ros::Rate rate(1/replanning_period);

    //new state created for final position
    StdVectorVectorDIM new_state(DIM);
    geometry_msgs::PoseStamped traj_0;
    //geometry_msgs::PoseStamped traj_1;
    geometry_msgs::PoseStamped current_0;

    rlss_ros::PiecewiseTrajectory pt_msg;
    double counter = 0.0;
    while (ros::ok())
    {
        ros::spinOnce();
        pt_msg.pieces.clear();
        ROS_INFO_STREAM (number_of_drones);


        ROS_INFO_STREAM ("global occupancy size");
        ROS_INFO_STREAM (global_occupancy_grid.size());
        //OccupancyGrid testing_lol(OccCoordinate(0.5,0.5,0.5));
        //ROS_INFO_STREAM (typeid(testing_lol).name());
        ROS_INFO_STREAM ("Planner Commencing");
        //ROS_INFO_STREAM (intended_velocity);
        ROS_INFO_STREAM (solver_type);
        ROS_INFO_STREAM (soft_optimization_parameters["robot_to_robot_hyperplane_constraints"].second);
        /*std::vector<std::pair<unsigned int, double>> testing_derivatives(DIM);
        for (std::size_t i = 0; i < maximum_derivative_magnitude_degrees.size(); i++)
        {
            ROS_INFO_STREAM (maximum_derivative_magnitude_degrees[i]);
            ROS_INFO_STREAM (maximum_derivative_magnitude_magnitudes[i]);
            testing_derivatives[i].first = maximum_derivative_magnitude_degrees[i],
            testing_derivatives[i].second = maximum_derivative_magnitude_magnitudes[i];
        }

        ROS_INFO_STREAM (testing_derivatives[0].second);*/
        //counter += replanning_period;
        //ROS_INFO_STREAM (counter);
        
        
        
    
        //ROS_INFO_STREAM (new_curve.numPieces());
        //rlss_goal_selector->setOriginalTrajectory(new_curve); //cannot call use the index beside the std::vector here, it has to stand alone
        StdVectorVectorDIM selected_state(DIM);
        ROS_INFO_STREAM (selected_state.size());
        ROS_INFO_STREAM (continuity_upto_degree);
        VectorDIM current_position(DIM);
        std::vector<AlignedBox> selected_shapes_to_collide(DIM);
        ROS_INFO_STREAM ("Number of collision shapes before insertion is " << selected_shapes_to_collide.size());
        for (std::size_t d = 0; d < DIM; d++)
        {
            selected_state[0][d] = state[0][d];
            current_position[d] = state[0][d];
        }

        for (const auto &elem: other_robot_collision_shapes) 
        {
            /* ROS_INFO_STREAM ("elem first");
            ROS_INFO_STREAM (elem.first);
            ROS_INFO_STREAM (robot_idx-1); */
            if (elem.first != robot_idx-1)
            {
                ROS_INFO_STREAM ("Collision shape being inserted for " << elem.first);
                selected_shapes_to_collide.push_back(elem.second); // first is unsigned int
            }
        }
        ROS_INFO_STREAM ("Number of collision shapes after insertion is " << selected_shapes_to_collide.size());
        
        //AlignedBox robot_box = self_col_shape->boundingBox(selected_state[0]);

        //ROS_INFO_STREAM ("Is the occupancy grid of the robot state occupied?");
        //ROS_INFO_STREAM (occupancy_grid.isOccupied(selected_state[0]));
        //ROS_INFO_STREAM (workspace.contains(robot_box));
        //ROS_INFO_STREAM (desired_trajectory_set_time.data.toSec());
        //ROS_INFO_STREAM (new_curve.numPieces());
        //selected_shapes_to_collide[i].push_back(other_robot_collision_shapes[1-i].second);
            
        //double new_time = time_on_trajectory.toSec()
        /*ROS_INFO_STREAM (time_on_trajectory.toSec());
        ROS_INFO_STREAM (state[0][1]);
        ROS_INFO_STREAM (selected_state[0][1]);
        ROS_INFO_STREAM (selected_shapes_to_collide.size());
        ROS_INFO_STREAM (other_robot_collision_shapes.size());
        occupancy_grid_ptr.*/

        //std::vector<std::vector<AlignedBox>> other_robot_shapes;
        //other_robot_shapes[0].push_back(other_robot_collision_shapes[1]);
        //other_robot_shapes[1].push_back(other_robot_collision_shapes[0]);

        //ROS_INFO_STREAM (selected_state[0][0]);
        //ROS_INFO_STREAM (selected_state[0][1]);
        //ROS_INFO_STREAM (selected_state[0][2]);



        //**************************Planner*********************************

        // somehow values from callback are only reflected inside here 
        
        //Goal selection constructor
        desired_time_horizon = durations[0];
        auto rlss_goal_selector = std::make_shared<RLSSGoalSelector>
            (
                    desired_time_horizon,
                    desired_trajectory, //this is the motherfker
                    workspace,
                    self_col_shape,
                    search_step
            );
        //rlss_goal_selector->setOriginalTrajectory(new_curve);
        auto goal_selector
            = std::static_pointer_cast<GoalSelector>(rlss_goal_selector);


        double maximum_velocity = std::numeric_limits<double>::max();
        for (const auto &[d, v] : maximum_derivative_magnitudes)
        {
        if (d == 1)
        {
            maximum_velocity = v;
            ROS_INFO_STREAM (qp_generator.numPieces());
            break;
        }
        }

        //Path searching constructor
        auto rlss_discrete_path_searcher = std::make_shared<RLSSDiscretePathSearcher>(
            replanning_period,
            workspace,
            self_col_shape,
            intended_velocity,
            qp_generator.numPieces()); // 4

        auto discrete_path_searcher = std::static_pointer_cast<DiscretePathSearcher>(
            rlss_discrete_path_searcher);



        //Traj optimisation constructor
        std::shared_ptr<TrajectoryOptimizer> trajectory_optimizer;
        switch (solver_type)
        {
        
        case 0:
        {
            auto rlss_hard_soft_optimizer = std::make_shared<RLSSHardSoftOptimizer>(
                self_col_shape,
                qp_generator,
                workspace,
                continuity_upto_degree,
                integrated_squared_derivative_weights,
                piece_endpoint_cost_weights,
                soft_optimization_parameters,
                optimization_obstacle_check_distance);
                trajectory_optimizer = std::static_pointer_cast<TrajectoryOptimizer>(
                rlss_hard_soft_optimizer);
        }    
            break;
            
        case 1:
        {
            auto rlss_soft_optimizer = std::make_shared<RLSSSoftOptimizer>(
                self_col_shape,
                qp_generator,
                workspace,
                continuity_upto_degree,
                integrated_squared_derivative_weights,
                piece_endpoint_cost_weights,
                soft_optimization_parameters,
                optimization_obstacle_check_distance);
                trajectory_optimizer = std::static_pointer_cast<TrajectoryOptimizer>(
                rlss_soft_optimizer);
        }
            break;

        case 2:
        {
            auto rlss_hard_optimizer = std::make_shared<RLSSHardOptimizer>(
                self_col_shape,
                qp_generator,
                workspace,
                continuity_upto_degree,
                integrated_squared_derivative_weights,
                piece_endpoint_cost_weights,
                optimization_obstacle_check_distance);
                trajectory_optimizer = std::static_pointer_cast<TrajectoryOptimizer>(
                rlss_hard_optimizer);
        }    
            break;

        }

        /*std::cout << "Optimizer..." << optimizer << std::endl;
        std::shared_ptr<TrajectoryOptimizer> trajectory_optimizer;
        if (optimizer == "rlss-hard-soft")
        {
            auto rlss_hard_soft_optimizer = std::make_shared<RLSSHardSoftOptimizer>(
                self_col_shape,
                qp_generator,
                workspace,
                continuity_upto_degree,
                integrated_squared_derivative_weights,
                piece_endpoint_cost_weights,
                soft_optimization_parameters,
                optimization_obstacle_check_distance);
            trajectory_optimizer = std::static_pointer_cast<TrajectoryOptimizer>(
                rlss_hard_soft_optimizer);
        }
        else if (optimizer == "rlss-soft")
        {
            auto rlss_soft_optimizer = std::make_shared<RLSSSoftOptimizer>(
                self_col_shape,
                qp_generator,
                workspace,
                continuity_upto_degree,
                integrated_squared_derivative_weights,
                piece_endpoint_cost_weights,
                soft_optimization_parameters,
                optimization_obstacle_check_distance);
            trajectory_optimizer = std::static_pointer_cast<TrajectoryOptimizer>(
                rlss_soft_optimizer);
        }
        else if (optimizer == "rlss-hard")
        {
            auto rlss_hard_optimizer = std::make_shared<RLSSHardOptimizer>(
                self_col_shape,
                qp_generator,
                workspace,
                continuity_upto_degree,
                integrated_squared_derivative_weights,
                piece_endpoint_cost_weights,
                optimization_obstacle_check_distance);
            trajectory_optimizer = std::static_pointer_cast<TrajectoryOptimizer>(
                rlss_hard_optimizer);
        }
        else
        {
            throw std::domain_error(
                absl::StrCat(
                    "optimizer ",
                    optimizer,
                    " not recognized.")
            );
        }*/

        /*ROS_INFO_STREAM ("Testing maximum derivative mag");
        ROS_INFO_STREAM (maximum_derivative_magnitude_degrees[0]);
        ROS_INFO_STREAM (maximum_derivative_magnitude_degrees[1]);
        ROS_INFO_STREAM (maximum_derivative_magnitude_magnitudes[0]);
        ROS_INFO_STREAM (maximum_derivative_magnitudes[0].first);*/

        //validity check constructor
        auto rlss_validity_checker = std::make_shared<RLSSValidityChecker>(
            maximum_derivative_magnitudes,
            search_step);

        auto validity_checker = std::static_pointer_cast<ValidityChecker>(
            rlss_validity_checker);


        //emplace back used to append the planners vector for the drones
        RLSS planner(
            goal_selector,
            trajectory_optimizer,
            discrete_path_searcher,
            validity_checker,
            max_rescaling_count,
            rescaling_multiplier
        );

        switch (activation.data)
        {
            
        case true:
            
        {
            ROS_INFO_STREAM ("Planner activated...");

            //add destination
            PiecewiseCurve new_curve;
            new_curve.addPiece(desired_trajectory.operator[](0));
            VectorDIM goal_position = new_curve.eval(new_curve.maxParameter(), 0);
            ROS_INFO_STREAM (new_curve.maxParameter());

            //add original pw curve to goal selector
            rlss_goal_selector->setOriginalTrajectory(new_curve);

            //Start timer
            ros::Time current_time = ros::Time::now(); 
            ros::Duration time_on_trajectory = current_time - desired_trajectory_set_time.data;
            
            ROS_INFO_STREAM (time_on_trajectory.toSec()); // current time
            ROS_INFO_STREAM (current_time.toSec());
            ROS_INFO_STREAM (desired_trajectory_set_time.data.toSec());
            //ROS_INFO_STREAM (desired_time_horizon);
            //ROS_INFO_STREAM (optimization_obstacle_check_distance);
            

            //Planner
            std::optional<PiecewiseCurve> curve = planner.plan(
                time_on_trajectory.toSec(),// time
                selected_state,// where i m currently
                selected_shapes_to_collide,//where other robots r currently, atm no shapes r appended which meant the vectors inside have no collision shapes (no min,no max)
                // unlike dyn sim, this vector itself has a default dim of 3, bit tricky situation
                global_occupancy_grid 
            ); // occupancy  
            
            
            //curve
            if (curve)
            {                    
                
                ROS_INFO_STREAM ("Planner worked and Curve succeeded...");
                PiecewiseCurve traj = *curve; //* = dereferencing
                //rlss_ros::PiecewiseTrajectory traj_msg;
                //traj_msg.generation_time.data = current_time;
                //traj.maxParameter might not be duration or time horizon when it reaches the end point in case
                new_state[0] = traj.eval(std::min(actual_timestamp, traj.maxParameter()),0); // this is the position it needs to actually go to        
                // updated position after replanning period, for planning of next curve only thats why u dun see the robot json updater                             
                ROS_INFO_STREAM ("Max_Time_Param"); // the path is lined up for 5.28s but theres no tracking 
                ROS_INFO_STREAM (traj.maxParameter());
                ROS_INFO_STREAM ("Time taken to so far...");
                ROS_INFO_STREAM (time_on_trajectory.toSec()); // current time
                traj_0.header.stamp.sec = time_on_trajectory.toSec(); 
                //traj_1.header.stamp.sec = time_on_trajectory.toSec(); 
                current_0.header.stamp.sec = time_on_trajectory.toSec();
                current_0.pose.position.x = state[0][0];
                current_0.pose.position.y = state[0][1];
                current_0.pose.position.z = state[0][2];
                //ROS_INFO_STREAM (current_time.toSec());
                //ROS_INFO_STREAM (desired_trajectory_set_time.data.toSec());
                /*ROS_INFO_STREAM ("x");
                ROS_INFO_STREAM (new_state[0][0]); // even if start from 0, domain error would pop if 0 is being accessed wo processing for drone 1
                ROS_INFO_STREAM ("y");
                ROS_INFO_STREAM (new_state[0][1]);
                ROS_INFO_STREAM ("z");
                ROS_INFO_STREAM (new_state[0][2]);*/

                rlss_ros::Bezier bez_msg;
                for (std::size_t f = 0; f < DIM; f++){
                    bez_msg.end.push_back(new_state[0][f]);
                }

                pt_msg.pieces.push_back(bez_msg);

                traj_0.pose.position.x = new_state[0][0]; 
                traj_0.pose.position.y = new_state[0][1];   
                traj_0.pose.position.z = new_state[0][2];   

                if (recording)
                {
                    nav_msgs::Path whole_traj_0;
                    whole_traj_0.header.frame_id = "uav1/camera_init";
                    whole_traj_0.header.stamp.sec = time_on_trajectory.toSec();
                    for (double d = 0; d < traj.maxParameter(); d+=0.1)
                    {
                        geometry_msgs::PoseStamped setpts;
                        setpts.pose.position.x = traj.eval(d,0)[0];
                        setpts.pose.position.y = traj.eval(d,0)[1];
                        setpts.pose.position.z = traj.eval(d,0)[2];
                        whole_traj_0.poses.push_back(setpts);
                    }
                    trajwhole_0.publish(whole_traj_0); 
                }                                        
                
            }
            else
            {

                ROS_WARN_STREAM("Planner failed");
            
            }
        
            break;

        }
            
        case false:

        {
        
            //    new_state[i] = state[i];
                ROS_INFO_STREAM ("Planner deactivated...");
        
        } 

            break;
            
        }

        


        /* ROS_INFO_STREAM ("x_0");
        ROS_INFO_STREAM (new_state[0][0]);
        ROS_INFO_STREAM ("current_x_0");
        ROS_INFO_STREAM (current_0.pose.position.x);
        ROS_INFO_STREAM ("y_0");
        ROS_INFO_STREAM (new_state[0][1]);
        ROS_INFO_STREAM ("current_y_0");
        ROS_INFO_STREAM (current_0.pose.position.y);
        ROS_INFO_STREAM ("z_0");
        ROS_INFO_STREAM (new_state[0][2]);
        ROS_INFO_STREAM ("current_z_0");
        ROS_INFO_STREAM (current_0.pose.position.z); */

    
        trajpub.publish(pt_msg);   
        trajpub_0.publish(traj_0);
        //trajpub_1.publish(traj_1);
        //currentpub_0.publish(current_0);
        rate.sleep();
        
    }

    return 0;

}
