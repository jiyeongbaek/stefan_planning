#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

#include <stefan_planning/validitycheck.h>
using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;


class planning_setup
{
public:
    planning_setup(ros::NodeHandle nh)
    {
        // construct the state space we are planning in
        space = std::make_shared<ob::SE3StateSpace>();
        // construct an instance of  space information from this state space
        si = std::make_shared<ob::SpaceInformation>(space);
        // create a problem instance
        pdef = std::make_shared<ob::ProblemDefinition>(si);
        
        ss = std::make_shared<og::SimpleSetup>(space);
        // vc = std::make_shared<ValidityCheck>(nh, ss->getSpaceInformation());

        vc = std::make_shared<ValidityCheck>(nh, si);
    }

    void setStartAndGoal()
    {
        ob::ScopedState<> start(space);
        auto *se3state = start->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        pos->values[0] = 0.95;
        pos->values[1] = 0.0;
        pos->values[2] = 0.72;
        Eigen::Quaterniond q;
        q = AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(deg2rad(30), Vector3d::UnitZ());
        rot->x = q.coeffs().x();
        rot->y = q.coeffs().y();
        rot->z = q.coeffs().z();
        rot->w = q.coeffs().w();
       
        // create a random goal state
        ob::ScopedState<> goal(space);
        auto *se3state2 = goal->as<ob::SE3StateSpace::StateType>();
        auto *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot2 = se3state2->as<ob::SO3StateSpace::StateType>(1);
        pos2->values[0] = 1.0;
        pos2->values[1] = 0.18;
        pos2->values[2] = 0.72;
        Eigen::Quaterniond q2;
        q2 = AngleAxisd(deg2rad(85), Vector3d::UnitX()) * AngleAxisd(deg2rad(3), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
        rot2->x = q2.coeffs().x();
        rot2->y = q2.coeffs().y();
        rot2->z = q2.coeffs().z();
        rot2->w = q2.coeffs().w();

        // pdef->setStartAndGoalStates(start, goal);

        pdef->setStartAndGoalStates(start, goal);
    }

    void plan()
    {
        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.3);
        bounds.setHigh(0, 1.5);
        bounds.setLow(1, -1.0);
        bounds.setHigh(1, 1.0);
        bounds.setLow(2, 0.6);
        bounds.setHigh(2, 2.0);
        space->setBounds(bounds);
        // Set the bounds of this state space. This defines the range of the space in which sampling is performed. 

        si->setStateValidityChecker(vc);
        
        // set the start and goal states
        setStartAndGoal();

        // create a planner for the defined space
        auto planner(std::make_shared<og::RRTConnect>(si));

        planner->setRange(0.2); //0.25 - 7, 0.1-24,  0.5-8
        std::cout << "range : " << planner->getRange() << std::endl;
        // set the problem we are trying to solve for the planner
        planner->setProblemDefinition(pdef);

    
        // si->printSettings(std::cout);  // print the settings for this space
        // pdef->print(std::cout);  // print the problem settings

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = planner->ob::Planner::solve(40.0);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path  ob::PathPtr
            auto path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;

            auto path2 = path->as<og::PathGeometric>();
            path2->printAsMatrix(std::cout);
            
            std::ofstream result_file("/home/jiyeong/catkin_ws/src/inverse_kinematics/object_state.txt");
            path2->printAsMatrix(result_file);

            path2->interpolate();
            path2->printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }

    void planSimple()
    {
        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.3);
        bounds.setHigh(0, 1.5);
        bounds.setLow(1, -1.0);
        bounds.setHigh(1, 1.0);
        bounds.setLow(2, 0.6);
        bounds.setHigh(2, 2.0);
        space->setBounds(bounds);

    
        ob::ScopedState<> start(space);
        auto *se3state = start->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        pos->values[0] = 0.95;
        pos->values[1] = 0.0;
        pos->values[2] = 0.72;
        Eigen::Quaterniond q;
        q = AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(deg2rad(30), Vector3d::UnitZ());
        rot->x = q.coeffs().x();
        rot->y = q.coeffs().y();
        rot->z = q.coeffs().z();
        rot->w = q.coeffs().w();

        // create a random goal state
        ob::ScopedState<> goal(space);
        auto *se3state2 = goal->as<ob::SE3StateSpace::StateType>();
        auto *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot2 = se3state2->as<ob::SO3StateSpace::StateType>(1);
        pos2->values[0] = 1.0;
        pos2->values[1] = 0.18;
        pos2->values[2] = 0.72;
        Eigen::Quaterniond q2;
        q2 = AngleAxisd(deg2rad(85), Vector3d::UnitX()) * AngleAxisd(deg2rad(3), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
        rot2->x = q2.coeffs().x();
        rot2->y = q2.coeffs().y();
        rot2->z = q2.coeffs().z();
        rot2->w = q2.coeffs().w();
        
        // ss->setStateValidityChecker(vc);
        ss->setStateValidityChecker(vc);

        auto planner = std::make_shared<og::RRTConnect>(ss->getSpaceInformation());
        planner->setRange(0.25);
        ss->setPlanner(planner);
        
        // ss->setPlanner(std::make_shared<og::RRTConnect>(ss->getSpaceInformation()));
        ss->setStartAndGoalStates(start, goal);
        ss->setup();
        // ss->print();
        

        ob::PlannerStatus solved = ss->solve(60.0);
        
        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            auto path =  ss->getSolutionPath();
            std::cout << "Found solution:" << std::endl;
            path.printAsMatrix(std::cout);

            ss->simplifySolution(5);
            auto simplePath = ss->getSolutionPath();
            simplePath.printAsMatrix(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }

    ob::SpaceInformationPtr si;
    // ob::StateSpaceptr space;
    std::shared_ptr<ob::SE3StateSpace> space;
    // boost::shared_ptr<ob::SE3StateSpace> space;
    ob::ProblemDefinitionPtr pdef;
    std::shared_ptr<ValidityCheck> vc;

    og::SimpleSetupPtr ss;
};