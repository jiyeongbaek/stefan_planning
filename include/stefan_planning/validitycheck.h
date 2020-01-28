#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
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

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;
using namespace Eigen;

double deg2rad(double angle_in_degrees)
{
    return angle_in_degrees * (M_PI / 180.0);
}

class ValidityCheck : public ob::StateValidityChecker
{
public:
    ValidityCheck(ros::NodeHandle nh, const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si)
    {
        nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    }

    void ik_solver(Affine3d target_pos)
    {
        double eps = 1e-5;
        int num_samples = 1000;
        double timeout = 0.005;
        std::string chain_start = "panda_link0";
        std::string chain_end = "panda_link8";
        TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

        KDL::Chain chain;
        KDL::JntArray ll, ul; //lower joint limits, upper joint limits

        bool valid = tracik_solver.getKDLChain(chain);
        if (!valid)
        {
            ROS_ERROR("There was no valid KDL chain found");
            return;
        }

        valid = tracik_solver.getKDLLimits(ll, ul);
        if (!valid)
        {
            ROS_ERROR("There were no valid KDL joint limits found");
            return;
        }

        assert(chain.getNrOfJoints() == ll.data.size());
        assert(chain.getNrOfJoints() == ul.data.size());

        ROS_INFO("Using %d joints", chain.getNrOfJoints());

        // Create Nominal chain configuration midway between all joint limits
        KDL::JntArray nominal(chain.getNrOfJoints());
        for (uint j = 0; j < nominal.data.size(); j++)
        {
            nominal(j) = (ll(j) + ul(j)) / 2.0;
        }

        // Create desired number of valid, random joint configurations
        std::vector<KDL::JntArray> JointList;
        KDL::JntArray q(chain.getNrOfJoints());

        KDL::JntArray result;
        KDL::Frame end_effector_pose;
        for (size_t i = 0; i < 3; i++)
            end_effector_pose.p(i) = target_pos.translation()[i];

        // end_effector_pose.M.Quaternion(target_pos.linear().coeffs()[0], target_pos.linear().coeffs()[1], target_pos.linear().coeffs()[2], target_pos.linear().coeffs()[3]);
        int rc;
        for (uint i = 0; i < num_samples; i++)
        {
            double elapsed = 0;
            rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
            if (rc >= 0)
                break;
            //   success++;
        }
    }

    virtual bool isValid(const ob::State *state) const
    {
        /* grasping point */
        Affine3d base_left, base_right, base_object, left_object, right_object;
        Affine3d obj_Lgrasp, obj_Rgrasp;    //defined by object frame CONSTANT
        Affine3d left_Lgrasp, right_Rgrasp; //defined by each robot frame

        // obj_Lgrasp.translation() = ;
        // obj_Lgrasp.linear() = ;
        // obj_Rgrasp.translation() = ;
        // obj_Rgrasp.linear() = ;

        // base_left.translation() = Vector3d(0. 0.2, 0);
        // base_right.translation() = Vector3d(0, -0.2, 0);

        // cast the abstract state type to the type we expect
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        // for (int i = 0; i < 3; i++)
        //     base_object.translation()[i] = pos(i);
        // base_object.linear() = rot;
        left_object = base_left.inverse() * base_object;
        right_object = base_right.inverse() * base_object;

        left_Lgrasp = left_object * obj_Lgrasp;
        right_Rgrasp = right_object * obj_Rgrasp;

        // ik_solver(left_Lgrasp);
        // ik_solver(right_Rgrasp);

        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
        return (const void *)rot != (const void *)pos;
    }

    std::string urdf_param;
};

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

        vc = std::make_shared<ValidityCheck>(nh, si);
        // nh.param("urdf_param", vc->urdf_param, std::string("/robot_description"));
    }

    void setStartAndGoal()
    {
        ob::ScopedState<> start(space);
        start[0] = 1.1;
        start[1] = -0.15;
        start[2] = 0.601;
        start[3] = 0.0;
        start[4] = 0.0;
        start[5] = 0.0;
        start[6] = 1.0;
        cout << start << endl;

        // create a random goal state
        ob::ScopedState<> goal(space);
        goal[0] = 1.1;
        goal[1] = 0.15;
        goal[2] = 0.625;
        Eigen::Quaterniond q;
        q = AngleAxisd(deg2rad(85), Vector3d::UnitX()) * AngleAxisd(deg2rad(3), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
        goal[3] = q.coeffs()[0];
        goal[4] = q.coeffs()[1];
        goal[5] = q.coeffs()[2];
        goal[6] = q.coeffs()[3];
        cout << goal << endl;
        pdef->setStartAndGoalStates(start, goal);
    }

    void plan()
    {
        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(-2);
        bounds.setHigh(2);

        space->setBounds(bounds);

        si->setStateValidityChecker(vc);
        // set the start and goal states
        setStartAndGoal();

        // create a planner for the defined space
        auto planner(std::make_shared<og::RRTConnect>(si));

        // set the problem we are trying to solve for the planner
        planner->setProblemDefinition(pdef);

        // perform setup steps for the planner
        planner->setup();

        // print the settings for this space
        si->printSettings(std::cout);

        // print the problem settings
        // pdef->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

        if (solved)
        {
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            ob::PathPtr path = pdef->getSolutionPath();
            std::cout << "Found solution:" << std::endl;

            // print the path to screen
            path->print(std::cout);
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
};