#include <ros/ros.h>
#include <rlss/RLSS.hpp>
#include <rlss/CollisionShapes/AlignedBoxCollisionShape.hpp>
#include <vector>
#include <rlss_ros/AABB.h>
#include <rlss_ros/AABBCollisionShape.h>
#include <rlss_ros/RobotState.h>
#include <rlss_ros/Bezier.h>
#include <rlss_ros/OccupancyGrid.h>
#include <rlss_ros/PiecewiseTrajectory.h>
#include "../third_party/rlss/third_party/json.hpp"
#include <splx/opt/PiecewiseCurveQPGenerator.hpp>
#include <rlss/TrajectoryOptimizers/RLSSHardOptimizer.hpp>
#include <rlss/TrajectoryOptimizers/RLSSSoftOptimizer.hpp>
#include <rlss/TrajectoryOptimizers/RLSSHardSoftOptimizer.hpp>
#include <rlss/DiscretePathSearchers/RLSSDiscretePathSearcher.hpp>
#include <rlss/ValidityCheckers/RLSSValidityChecker.hpp>
#include <rlss/GoalSelectors/RLSSGoalSelector.hpp>
#include <std_msgs/Time.h>
#include <rlss_ros/setTargetsConfig.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>

constexpr unsigned int DIM = DIMENSION;

using CollisionShape = rlss::CollisionShape<double, DIM>;
using AlignedBoxCollisionShape = rlss::AlignedBoxCollisionShape<double, DIM>;
using RLSS = rlss::RLSS<double, DIMENSION>;
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
using TrajectoryOptimizer = rlss::TrajectoryOptimizer<double, DIM>;
using DiscretePathSearcher = rlss::DiscretePathSearcher<double, DIM>;
using ValidityChecker = rlss::ValidityChecker<double, DIM>;
using GoalSelector = rlss::GoalSelector<double, DIM>;

int self_robot_idx;
std::map<unsigned int, AlignedBox> other_robot_collision_shapes; // robot_idx -> colshape
std::unique_ptr<OccupancyGrid> occupancy_grid_ptr;
ros::Time desired_trajectory_set_time = ros::Time(0);
PiecewiseCurve desired_trajectory;
StdVectorVectorDIM state;
unsigned int continuity_upto_degree;
double testing;

// Dynamic Planner Params
double reach_distance;
double optimization_obstacle_check_distance;
int solver_selection;
int number_of_drones;

// Duration and rescaling coeff
double rescaling_factor;
double intended_velocity;


void dynamicReconfigureCallback(rlss_ros::setTargetsConfig &config, uint32_t level){
    trajectory_target = config.trajectory_target;
    velocity = config.intended_velocity;
    number_of_drones = config.number_of_drones; 
    goal_pose[0] << config.x_0, config.y_0, config.z_0;
    goal_pose[1] << config.x_1, config.y_1, config.z_1;
}



void otherRobotShapeCallback(const rlss_ros::AABBCollisionShape::ConstPtr &msg) // each robot needs to have its own shape topic
{
    if (msg->bbox.min.size() != DIM)
    {
        return;
    }
    unsigned int robot_idx = msg->robot_idx;
    if (robot_idx != self_robot_idx)
    {
        VectorDIM shape_min, shape_max;
        for (unsigned int i = 0; i < DIM; i++)
        {
            shape_min(i) = msg->bbox.min[i]; // -0.6, 0.6, 0
            shape_max(i) = msg->bbox.max[i]; // 0.6, 0.6, 0.25
        }
        other_robot_collision_shapes[robot_idx] = AlignedBox(shape_min, shape_max);
    }
} // other robot shapes



void selfStateCallback(const rlss_ros::RobotState::ConstPtr &msg) // position callback = state[0][1-3] 
{
    if (msg->vars.size() != DIM * (continuity_upto_degree + 1))
    {
        ROS_FATAL_STREAM("state message vars length is not valid.");
    }
    else
    {
        for (unsigned int i = 0; i <= continuity_upto_degree; i++)
        {
            for (unsigned int j = 0; j < DIM; j++)
            {
                state[i](j) = msg->vars[i * DIM + j];
            }
        }
    }
}



void desiredTrajectoryCallback(const rlss_ros::PiecewiseTrajectory::ConstPtr &msg)
{
    if (1 + 1 == 2)
    {
        ROS_INFO_STREAM("callback is working");
    }
    PiecewiseCurve curve;
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
            for (unsigned int j = 0; j < piece_msg.cpts.size() / DIM; j++)
            {
                VectorDIM cpt;
                for (unsigned int k = 0; k < DIM; k++)
                {
                    cpt(k) = piece_msg.cpts[j * DIM + k];
                }
                bez.appendControlPoint(cpt);
            }
            curve.addPiece(bez);
        }
    }
    desired_trajectory = curve;
    desired_trajectory_set_time = ros::Time::now();
    testing = 7.0;
}



void occupancyGridCallback(const rlss_ros::OccupancyGrid::ConstPtr &msg)
{
    if (msg->step_size.size() != DIM)
    {
        ROS_FATAL_STREAM("occupancy grid dimensions not correct");
    }
    else
    {
        OccCoordinate step_size;
        for (unsigned int i = 0; i < DIM; i++)
        {
            step_size(i) = msg->step_size[i];
        }

        occupancy_grid_ptr = std::make_unique<OccupancyGrid>(step_size);
        for (std::size_t i = 0; i < msg->occupied_indexes.size() / DIM; i++)
        {
            OccIndex index;
            for (std::size_t j = 0; j < DIM; j++)
            {
                index(j) = msg->occupied_indexes[i * DIM + j];
            }
            occupancy_grid_ptr->setOccupancy(index);
        }
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "borealis-planner-3D");
    ros::NodeHandle nh;

    //dynamic reconfigure callback
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig> server;
    dynamic_reconfigure::Server<rlss_ros::setTargetsConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    server.setCallback(f);



    //required params from launch file*********************************************
    nh.getParam("robot_idx", self_robot_idx);
    
    //continuity upto deg
    int c_upto_d;
    nh.getParam("continuity_upto_degree", c_upto_d);
    continuity_upto_degree = c_upto_d;
    state.resize(continuity_upto_degree + 1);
    
    //replanning period
    double replanning_period;
    nh.getParam("replanning_period", replanning_period);
    //std::cout << "replanning_period: " << replanning_period << std::endl;


    //max derivative mag
    std::vector<int> maximum_derivative_magnitude_degrees;
    std::vector<double> maximum_derivative_magnitude_magnitudes;
    nh.getParam("maximum_derivative_magnitude_degrees", maximum_derivative_magnitude_degrees);
    nh.getParam("maximum_derivative_magnitude_magnitudes", maximum_derivative_magnitude_magnitudes);
    if (maximum_derivative_magnitude_degrees.size() != maximum_derivative_magnitude_magnitudes.size())
    {
        ROS_FATAL_STREAM("maximum derivative magnitude degree magnitude mismatch");
        return 0;
    }
    std::vector<std::pair<unsigned int, double>> maximum_derivative_magnitudes;
    for (std::size_t i = 0; i < maximum_derivative_magnitude_degrees.size(); i++)
    {
        maximum_derivative_magnitudes.push_back(
            std::make_pair(
                maximum_derivative_magnitude_degrees[i],
                maximum_derivative_magnitude_magnitudes[i]));
    }


    //workspace size 
    std::vector<double> workspace_min_vec, workspace_max_vec;
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
    VectorDIM workspace_min, workspace_max;
    for (unsigned int i = 0; i < DIM; i++)
    {
        workspace_min(i) = workspace_min_vec[i];
        workspace_max(i) = workspace_max_vec[i];
    }
    AlignedBox workspace(workspace_min, workspace_max);


    //UAV collision shape
    std::vector<double> colshape_min_vec, colshape_max_vec;
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
    VectorDIM colshape_min, colshape_max;
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



    //soft optimisation SVM params for r2r, r2o, ipc, cc 
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



    //obstacle check distance limit
    double optimization_obstacle_check_distance;
    nh.getParam("optimization_obstacle_check_distance", optimization_obstacle_check_distance);


    //soft-hard, hard, soft
    std::string optimizer;
    nh.getParam("optimizer", optimizer);


    //end point cost weights
    std::vector<double> piece_endpoint_cost_weights;
    nh.getParam("piece_endpoint_cost_weights", piece_endpoint_cost_weights);


    //integrated squared derivative weights
    std::vector<int> integrated_squared_derivative_weight_degrees;
    std::vector<double> integrated_squared_derivative_weight_weights;
    nh.getParam("integrated_squared_derivative_weight_degrees", integrated_squared_derivative_weight_degrees);
    nh.getParam("integrated_squared_derivative_weight_weights", integrated_squared_derivative_weight_weights);
    if (integrated_squared_derivative_weight_weights.size() != integrated_squared_derivative_weight_degrees.size())
    {
        ROS_FATAL_STREAM("integrated squared derivative weights size mismatch");
        return 0;
    }
    std::vector<std::pair<unsigned int, double>>
        integrated_squared_derivative_weights;
    for (std::size_t i = 0; i < integrated_squared_derivative_weight_degrees.size(); i++)
    {
        integrated_squared_derivative_weights.push_back(
            std::make_pair(
                integrated_squared_derivative_weight_degrees[i],
                integrated_squared_derivative_weight_weights[i]));
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
    double rescaling_multiplier;
    nh.getParam("rescaling_multiplier", rescaling_multiplier);

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

//***********************************************************



    //Goal selection constructor
    auto rlss_goal_selector = std::make_shared<RLSSGoalSelector>(
        desired_time_horizon,
        desired_trajectory,// taken from dynamic_desired_target_feeder
        workspace,
        self_col_shape,
        search_step);
    auto goal_selector = std::static_pointer_cast<GoalSelector>(rlss_goal_selector);

    double maximum_velocity = std::numeric_limits<double>::max();
    for (const auto &[d, v] : maximum_derivative_magnitudes)
    {
        if (d == 1)
        {
            maximum_velocity = v;
            break;
        }
    }


    //Path searching constructor
    auto rlss_discrete_path_searcher = std::make_shared<RLSSDiscretePathSearcher>(
        replanning_period,
        workspace,
        self_col_shape,
        maximum_velocity,
        qp_generator.numPieces());
    auto discrete_path_searcher = std::static_pointer_cast<DiscretePathSearcher>(
        rlss_discrete_path_searcher);


    //Traj optimisation constructor
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
                " not recognized."));
    }


    //validity check constructor
    auto rlss_validity_checker = std::make_shared<RLSSValidityChecker>(
        maximum_derivative_magnitudes,
        search_step);
    auto validity_checker = std::static_pointer_cast<ValidityChecker>(
        rlss_validity_checker);


    //RLSS constructor
    RLSS planner(
        goal_selector,
        trajectory_optimizer,
        discrete_path_searcher,
        validity_checker,
        max_rescaling_count,
        rescaling_multiplier);


    //collision shapes
    ros::Subscriber colshapesub = nh.subscribe("/other_robot_collision_shapes", 1000, otherRobotShapeCallback);
    

    //each robot's location
    ros::Subscriber statesub = nh.subscribe("self_state", 1, selfStateCallback);
    
    
    //Current drone's intended goal and starting cpt/position
    ros::Subscriber destrajsub = nh.subscribe("Pseudo_trajectory", 1, desiredTrajectoryCallback);
    
    
    //Occupancy grid of current drone
    ros::Subscriber occgridsub = nh.subscribe("occupancy_grid", 1, occupancyGridCallback);
    

    //Push the position of where this drone shud go into topic 
    ros::Publisher trajpub = nh.advertise<rlss_ros::PiecewiseTrajectory>("trajectory", 1);


    //loops at the rate of the replanning period
    ros::Rate rate(1 / replanning_period);

    while (ros::ok())
    {
        ros::spinOnce();

        //        if(self_robot_idx == 6) {
        //            ROS_INFO_STREAM("desired_trajectory_set_time:");
        //            ROS_INFO_STREAM(desired_trajectory_set_time);
        //        }

        //if(desired_trajectory_set_time != ros::Time(0)){
        //    ROS_INFO_STREAM("true");
        //}
        //else{
        //    ROS_INFO_STREAM("false");
        //}
        // piecewise curve is empty and needs fixing****
        //ROS_INFO_STREAM (desired_trajectory_set_time);
        if (desired_trajectory_set_time != ros::Time(0))
        {
            //ROS_INFO_STREAM("desired trajectory set.");
            rlss_goal_selector->setOriginalTrajectory(desired_trajectory);

            std::vector<AlignedBox> other_robot_shapes;
            for (const auto &elem : other_robot_collision_shapes)
            {
                other_robot_shapes.push_back(elem.second);
            }

            ros::Time current_time = ros::Time::now();
            ros::Duration time_on_trajectory =
                current_time - desired_trajectory_set_time;

            //            if(self_robot_idx == 6) {
            //                ROS_INFO_STREAM("before plan");
            //            }
            std::optional<PiecewiseCurve> curve = planner.plan(
                time_on_trajectory.toSec(),
                state,
                other_robot_shapes,
                *occupancy_grid_ptr);

            //            if(self_robot_idx == 6) {
            //                ROS_INFO_STREAM("after plan");
            //            }
            if (curve)
            {
                //                if(self_robot_idx == 6) {
                //                    ROS_INFO_STREAM("curve");
                //                }
                PiecewiseCurve traj = *curve;

                rlss_ros::PiecewiseTrajectory traj_msg;
                traj_msg.generation_time.data = current_time;
                for (std::size_t i = 0; i < traj.numPieces(); i++)
                {
                    rlss_ros::Bezier bez_msg;
                    bez_msg.dimension = DIMENSION;
                    bez_msg.duration = traj[i].maxParameter();

                    for (std::size_t j = 0; j < traj[i].numControlPoints(); j++)
                    {
                        const auto &cpt = traj[i][j];
                        for (unsigned int d = 0; d < DIM; d++)
                        {
                            bez_msg.cpts.push_back(cpt(d));
                        }
                    }
                    traj_msg.pieces.push_back(bez_msg);
                }
                trajpub.publish(traj_msg);
            }
            else
            {
                //                if(self_robot_idx == 6) {
                //                    ROS_INFO_STREAM("no curve");
                //                }
                ROS_WARN_STREAM("planner failed.");
            }
        }
        else
        {
            ROS_INFO_STREAM(testing);
            ROS_INFO_STREAM("desired trajectory not yet set.");
        }
        rate.sleep();
    }

    return 0;
}