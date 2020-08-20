#pragma once

#include <iostream>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>

#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>


#include <ros/package.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
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
        vc = std::make_shared<ValidityCheck>(nh, ss->getSpaceInformation());

        // vc = std::make_shared<ValidityCheck>(nh, si);
    }

    void setStartAndGoal()
    {
        ob::ScopedState<> start(space);
        auto *se3state = start->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        pos->values[0] = 1.05;
        pos->values[1] = -0.24;
        pos->values[2] = 1.151;
        rot->x = 0.0;
        rot->y = 0.0;
        rot->z = -0.1736482;
        rot->w = 0.9848078;
        space->enforceBounds(se3state);
        
       
        ob::ScopedState<> goal(space);
        auto *se3state2 = goal->as<ob::SE3StateSpace::StateType>();
        auto *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot2 = se3state2->as<ob::SO3StateSpace::StateType>(1);
        pos2->values[0] = 1.15;
        pos2->values[1] = 0.05;
        pos2->values[2] = 1.2;
        rot2->x = 0.6881908;
        rot2->y = 0.0150164;
        rot2->z = -0.015824;
        rot2->w = 0.7252018;
        //         pos2->values[0] = 1.2;
        // pos2->values[1] = 0.3;
        // pos2->values[2] = 1.1;
        // rot2->x = 0.996493;
        // rot2->y = 0.0;
        // rot2->z = 0.0;
        // rot2->w = 0.0836778;
        space->enforceBounds(se3state2);

        pdef->setStartAndGoalStates(start, goal);
    }

    void plan()
    {
        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 1.0);
        bounds.setHigh(0, 1.3);
        bounds.setLow(1, -0.4);
        bounds.setHigh(1, 0.4);
        bounds.setLow(2, 1.15);
        bounds.setHigh(2, 1.3);
        space->setBounds(bounds);
        std::cout << "  - min: ";
        for (unsigned int i = 0; i < 3; ++i)
            std::cout << bounds.low[i] << " ";
        std::cout << std::endl;
        std::cout << "  - max: ";
        for (unsigned int i = 0; i < 3; ++i)
            std::cout << bounds.high[i] << "  ";
        std::cout << std::endl;
        
        si->setStateValidityChecker(vc);
        setStartAndGoal();
        
        // create a planner for the defined space
        auto planner(std::make_shared<og::PRM>(si));
        // auto planner(std::make_shared<og::RRTConnect>(si));
        // auto planner(std::make_shared<og::RRTstar>(si));

        // planner->setRange(0.25); //0.25 - 7, 0.1-24,  0.5-8
        // set the problem we are trying to solve for the planner
        // ss->setup();
        planner->setProblemDefinition(pdef);
        planner->setup();
        ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path  ob::PathPtr
            auto path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;

            auto path2 = path->as<og::PathGeometric>();
            path2->printAsMatrix(std::cout);            
            std::ofstream result_file("/home/jiyeong/catkin_ws/src/3_constraint_planning/stefan_planning/object_state.txt");            

            path2->printAsMatrix(result_file);
            path2->interpolate();
            path2->printAsMatrix(result_file);
            // path2->printAsMatrix(std::cout);


            result_file.close();
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
        // q = AngleAxisd(0, Vector3d::UnitX()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(deg2rad(30), Vector3d::UnitZ());
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
        // q2 = AngleAxisd(deg2rad(85), Vector3d::UnitX()) * AngleAxisd(deg2rad(3), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
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