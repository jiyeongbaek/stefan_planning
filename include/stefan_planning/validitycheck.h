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
        /* grasping point */
    }

    bool ik_solver(Affine3d target_pos) const
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
            return false;
        }

        valid = tracik_solver.getKDLLimits(ll, ul);
        if (!valid)
        {
            ROS_ERROR("There were no valid KDL joint limits found");
            return false;
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

        Matrix3d Rot_d = target_pos.linear();
        KDL::Rotation A;
        A.data[0] = Rot_d(0, 0);
        A.data[1] = Rot_d(0, 1);
        A.data[2] = Rot_d(0, 2);
        A.data[3] = Rot_d(1, 0);
        A.data[4] = Rot_d(1, 1);
        A.data[5] = Rot_d(1, 2);
        A.data[6] = Rot_d(2, 0);
        A.data[7] = Rot_d(2, 1);
        A.data[8] = Rot_d(2, 2);
        end_effector_pose.M = A;

        int rc = -1;
        for (uint i = 0; i < num_samples; i++)
        {
            double elapsed = 0;
            rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
            // std::cout << rc << std::endl;
            if (rc >= 0)
            {
                ROS_INFO("FOUND SOLUTION");
                return true;
            }
        }
        return false;
    }

    virtual bool isValid(const ob::State *state) const
    {
        Affine3d base_left, base_right, left_object, right_object;
        Affine3d obj_Lgrasp, obj_Rgrasp;    //defined by object frame CONSTANT
        Affine3d left_Lgrasp, right_Rgrasp; //defined by each robot frame
        obj_Lgrasp.translation() = Vector3d(-0.414640147082728, 0.298136277133603, 0.0177842219404252);
        Quaterniond q_Lgrasp(0.0090642, 0.720166, -0.693648, 0.0114454 );
        obj_Lgrasp.linear() = q_Lgrasp.toRotationMatrix();

        obj_Rgrasp.translation() = Vector3d(-0.410945269338053, 0.175340483200395, 0.35246196213878);
        Quaterniond q_Rgrasp( -0.0630359, 0.717459, -0.690838, -0.0634221);
        obj_Rgrasp.linear() = q_Rgrasp.toRotationMatrix();

        base_left.translation() = Vector3d(0, 0.2, 0.6);
        base_right.translation() = Vector3d(0, -0.2, 0.6);
        base_left.linear().setIdentity();
        base_right.linear().setIdentity();

        // cast the abstract state type to the type we expect
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        Affine3d base_object;
        base_object.translation() = Vector3d(pos->values[0], pos->values[1], pos->values[2]);
        base_object.linear() = Quaterniond(rot->w, rot->x, rot->y, rot->z).toRotationMatrix();

        left_Lgrasp = base_left.inverse() * base_object * obj_Lgrasp;
        right_Rgrasp = base_right.inverse() * base_object * obj_Rgrasp;

        return (ik_solver(left_Lgrasp) && ik_solver(right_Rgrasp));

        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
        // return (const void *)rot != (const void *)pos;
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
        auto *se3state = start->as<ob::SE3StateSpace::StateType>();
        auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        pos->values[0] = 1.1;
        pos->values[1] = -0.15;
        pos->values[2] = 0.601;
        rot->x = rot->y = rot->z = 0.0;
        rot->w = 1.0;
        cout << start << endl;

        // create a random goal state
        ob::ScopedState<> goal(space);
        auto *se3state2 = goal->as<ob::SE3StateSpace::StateType>();
        auto *pos2 = se3state2->as<ob::RealVectorStateSpace::StateType>(0);
        auto *rot2 = se3state2->as<ob::SO3StateSpace::StateType>(1);
        pos2->values[0] = 1.1;
        pos2->values[1] = 0.15;
        pos2->values[2] = 0.625;
        Eigen::Quaterniond q;
        q = AngleAxisd(deg2rad(85), Vector3d::UnitX()) * AngleAxisd(deg2rad(3), Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
        rot2->x = q.coeffs().x();
        rot2->y = q.coeffs().y();
        rot2->z = q.coeffs().z();
        rot2->w = q.coeffs().w();

        cout << goal << endl;
        pdef->setStartAndGoalStates(start, goal);
    }

    void plan()
    {
        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.5);
        bounds.setHigh(0, 2.5);
        bounds.setLow(1, -1.5);
        bounds.setHigh(1, 1.5);
        bounds.setLow(2, 0.6);
        bounds.setHigh(2, 1.6);
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
        ob::PlannerStatus solved = planner->ob::Planner::solve(600.0);

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