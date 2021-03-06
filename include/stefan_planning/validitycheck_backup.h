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

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;
using namespace Eigen;

typedef collision_detection::CollisionWorldFCL DefaultCWorldType;
typedef collision_detection::CollisionRobotFCL DefaultCRobotType;
typedef Matrix<double, 7, 1> Vector7d;
const std::string panda_joint_names[14] = {"panda_left_joint1", "panda_left_joint2", "panda_left_joint3", "panda_left_joint4", "panda_left_joint5", "panda_left_joint6", "panda_left_joint7",
                            "panda_right_joint1", "panda_right_joint2", "panda_right_joint3", "panda_right_joint4", "panda_right_joint5", "panda_right_joint6", "panda_right_joint7"};


double deg2rad(double angle_in_degrees)
{
    return angle_in_degrees * (M_PI / 180.0);
}

struct Ik_sol
{
    Vector7d left;
    Vector7d right;
};

class PandaCollisionCheck
{
public:
  PandaCollisionCheck()
  {
    robot_model_ = moveit::core::loadTestingRobotModel("dual_panda");
    robot_model_ok_ = static_cast<bool>(robot_model_);
    acm_.reset(new collision_detection::AllowedCollisionMatrix(robot_model_->getLinkModelNames(), false));
    
    acm_->setEntry("base", "panda_left_link0", true);
    acm_->setEntry("base", "panda_left_link1", true);
    acm_->setEntry("base", "panda_left_link2", true);
    acm_->setEntry("base", "panda_left_link3", true);
    acm_->setEntry("base", "panda_left_link4", true);

    acm_->setEntry("base", "panda_right_link0", true);
    acm_->setEntry("base", "panda_right_link1", true);
    acm_->setEntry("base", "panda_right_link2", true);
    acm_->setEntry("base", "panda_right_link3", true);
    acm_->setEntry("base", "panda_right_link4", true);

    acm_->setEntry("panda_left_hand" ,"panda_left_leftfinger" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link5" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_hand" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link1" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link2" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_hand" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_leftfinger" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link5" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link6" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link7" , true);
    acm_->setEntry("panda_left_link1" ,"panda_left_link2" , true);
    acm_->setEntry("panda_left_link1" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_link1" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link5" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link6" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link7" , true);
    acm_->setEntry("panda_left_link2" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_link2" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link5" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link5" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_link5" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link5" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_link5" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link5" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link5" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link5" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link6" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link6" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link6" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link6" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link7" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link7" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link7" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_rightfinger" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_rightfinger" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_leftfinger" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link5" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link1" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link2" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link1" ,"panda_right_link2" , true);
    acm_->setEntry("panda_right_link1" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_link1" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link2" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_link2" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link5" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_link5" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link5" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_link5" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link6" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link6" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link7" ,"panda_right_rightfinger" , true);

    crobot_.reset(new DefaultCRobotType(robot_model_));
    cworld_.reset(new DefaultCWorldType());
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    base_.reset(new DefaultCWorldType());

    shapes::Shape* box = new shapes::Box(2.4, 1.2, 0.6);
    shapes::ShapeConstPtr box_ptr(box);
    Eigen::Isometry3d box_pos{Eigen::Isometry3d::Identity()};
    box_pos.translation().x() = 1.0;
    box_pos.translation().y() = 0.0;
    box_pos.translation().z() = 0.3; // bos : 0.6, z : 0.23 됨!

    shapes::Shape* temp_box = new shapes::Box(0.8, 0.8, 0.15);
    shapes::ShapeConstPtr temp_box_ptr(temp_box);
    Eigen::Isometry3d temp_box_pos{Eigen::Isometry3d::Identity()};
    temp_box_pos.translation().x() = 0.9;
    temp_box_pos.translation().y() = 0.0;
    temp_box_pos.translation().z() = 0.675; // bos : 0.6, z : 0.23 됨!

    base_->getWorld()->addToObject("box", box_ptr, box_pos);
    base_->getWorld()->addToObject("temp_box_base", temp_box_ptr, temp_box_pos);
  }

public:
  bool robot_model_ok_;

  robot_model::RobotModelPtr robot_model_;
  collision_detection::CollisionRobotPtr crobot_;
  collision_detection::CollisionWorldPtr cworld_;
  collision_detection::CollisionWorldPtr base_;

  collision_detection::AllowedCollisionMatrixPtr acm_;

  robot_state::RobotStatePtr robot_state_;
};


class ValidityCheck : public ob::StateValidityChecker
{
public:
    PandaCollisionCheck colli;
    Ik_sol ik_sol;
    shapes::ShapeConstPtr shape_ptr;
    std::string urdf_param;
    Affine3d obj_Lgrasp, obj_Rgrasp;
    Affine3d base_left, base_right;
    ValidityCheck(ros::NodeHandle nh, const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si)
    {    
        nh.param("urdf_param", urdf_param, std::string("/robot_description"));
        
        shapes::Mesh* stefan = shapes::createMeshFromResource("file:///home/jiyeong/catkin_ws/src/stefan_description/stl/assembly.stl");
        shape_ptr = shapes::ShapeConstPtr(stefan);
        
        ik_sol.left.setZero();
        ik_sol.right.setZero();
        
        base_left.translation() = Vector3d(0, 0.3, 0.6);
        base_right.translation() = Vector3d(0, -0.3, 0.6);
        base_left.linear().setIdentity();
        base_right.linear().setIdentity();
        Vector3d z_offset(0, 0, -0.109);

        Quaterniond q_Lgrasp(0.48089 , 0.518406, -0.518406, 0.48089);
        obj_Lgrasp.linear() = Quaterniond(0.48089 , 0.518406, -0.518406, 0.48089).toRotationMatrix();
        obj_Lgrasp.translation() = Vector3d(-0.417291983962059, 0.385170965965183, 0.189059236695616) + obj_Lgrasp.linear() * z_offset;

        Quaterniond q_Rgrasp(  -0.0630359, 0.717459, -0.690838, -0.0634221);
        obj_Rgrasp.linear() = q_Rgrasp.toRotationMatrix();
        obj_Rgrasp.translation() = Vector3d(-0.408115020594241, 0.101560465864071, 0.339098439321291) + obj_Rgrasp.linear() * z_offset;
    }

    bool ik_solver(Affine3d left_target, Affine3d right_target) const
    {
        double eps = 1e-5;
        int num_samples = 50;
        double timeout = 0.005;
        std::string chain_start = "panda_link0";
        std::string chain_end = "panda_hand";
        TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

        KDL::Chain chain;
        KDL::JntArray ll, ul; //lower joint limits, upper joint limits

        bool valid = tracik_solver.getKDLChain(chain);
        if (!valid){ROS_ERROR("There was no valid KDL chain found");return false;}

        valid = tracik_solver.getKDLLimits(ll, ul);
        if (!valid){ROS_ERROR("There were no valid KDL joint limits found");return false;}

        assert(chain.getNrOfJoints() == ll.data.size());
        assert(chain.getNrOfJoints() == ul.data.size());

        // Create Nominal chain configuration midway between all joint limits
        KDL::JntArray nominal(chain.getNrOfJoints());
        for (uint j = 0; j < nominal.data.size(); j++)
        {
            nominal(j) = (ll(j) + ul(j)) / 2.0;
        }

        // Create desired number of valid, random joint configurations
        std::vector<KDL::JntArray> JointList;
        KDL::JntArray q(chain.getNrOfJoints());

        KDL::JntArray Lresult, Rresult;
        KDL::Frame Lend_effector_pose;
        KDL::Frame Rend_effector_pose;
        for (size_t i = 0; i < 3; i++){
            Lend_effector_pose.p(i) = left_target.translation()[i];
            Rend_effector_pose.p(i) = right_target.translation()[i];}

        Matrix3d Rot_d = left_target.linear();
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
        Lend_effector_pose.M = A;
        
        Matrix3d Rot_d2 = right_target.linear();
        KDL::Rotation B;
        B.data[0] = Rot_d2(0, 0);
        B.data[1] = Rot_d2(0, 1);
        B.data[2] = Rot_d2(0, 2);
        B.data[3] = Rot_d2(1, 0);
        B.data[4] = Rot_d2(1, 1);
        B.data[5] = Rot_d2(1, 2);
        B.data[6] = Rot_d2(2, 0);
        B.data[7] = Rot_d2(2, 1);
        B.data[8] = Rot_d2(2, 2);
        Rend_effector_pose.M = B;

        for (uint i = 0; i < num_samples; i++)
        {
            if (tracik_solver.CartToJnt(nominal, Lend_effector_pose, Lresult) >= 0)
            {
                for (uint j = 0; j < num_samples; j++)
                {
                    if (tracik_solver.CartToJnt(nominal, Rend_effector_pose, Rresult) >= 0 && !selfCollisionCheck(Lresult, Rresult) )
                        return true;
                }
            }
        }
        // OMPL_ERROR("FAIL");
        return false;
    }


    void updateRobotState(const Vector7d ik_left, const Vector7d ik_right, robot_state::RobotState &panda_state) const
    {
        panda_state.setToDefaultValues();
        for (int i = 0; i < 7; i++)
        {
            panda_state.setJointPositions(panda_joint_names[i], &ik_left[i]);
            panda_state.setJointPositions(panda_joint_names[i + 7], &ik_right[i]);
        }
        panda_state.update();
    }

    bool RobotObjectCollision(const Affine3d object_pos, const Vector7d ik_left, const Vector7d ik_right) const
    {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.max_contacts = 10;
        req.contacts = true;
        req.verbose = true;
        
        Eigen::Isometry3d pos1;
        pos1.translation().x() = object_pos.translation()[0];
        pos1.translation().y() = object_pos.translation()[1];
        pos1.translation().z() = object_pos.translation()[2];
        pos1.linear() = object_pos.linear();
        
        /* update robot state with ik solutions */
        updateRobotState(ik_left, ik_right, *colli.robot_state_);
        colli.cworld_->getWorld()->addToObject("stefan", shape_ptr, pos1);

        colli.acm_->setEntry("panda_right_rightfinger", "stefan", true);
        colli.acm_->setEntry("panda_right_leftfinger", "stefan", true);
        colli.acm_->setEntry("panda_left_rightfinger", "stefan", true);
        colli.acm_->setEntry("panda_left_leftfinger", "stefan", true);

        colli.cworld_->checkRobotCollision(req, res, *colli.crobot_, *colli.robot_state_, *colli.acm_);
        
        /* COLLISION : TRUE, SAFE : FALSE*/
        return res.collision;
    }
    
    bool ObjectWorldCollision(const Affine3d base_object) const
    {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.max_contacts = 15;
        req.contacts = true;
        req.verbose = false;
        
        Eigen::Isometry3d pos1;
        pos1.translation().x() = base_object.translation()[0];
        pos1.translation().y() = base_object.translation()[1];
        pos1.translation().z() = base_object.translation()[2];
        pos1.linear() = base_object.linear();
        
        colli.cworld_->getWorld()->addToObject("stefan", shape_ptr, pos1);
        colli.cworld_->checkWorldCollision(req, res, *colli.base_);
        
        /* COLLISION : TRUE, SAFE : FALSE*/
        return res.collision;
    }

    bool selfCollisionCheck(const Vector7d ik_left, const Vector7d ik_right) const
    {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        updateRobotState(ik_left, ik_right, *colli.robot_state_);
        colli.crobot_->checkSelfCollision(req, res, *colli.robot_state_, *colli.acm_);
        
        /* COLLISION : TRUE, SAFE : FALSE*/
        return res.collision;
    }
    bool selfCollisionCheck(const KDL::JntArray Lresult, const KDL::JntArray Rresult) const
    {
        Vector7d ik_left, ik_right;
        for (int i = 0; i < 7; i++){
            ik_left[i] = Lresult.data[i];
            ik_right[i] = Rresult.data[i];
        }
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.contacts = true;
        req.verbose = false;
        
        updateRobotState(ik_left, ik_right, *colli.robot_state_);
        colli.crobot_->checkSelfCollision(req, res, *colli.robot_state_, *colli.acm_);
        /* COLLISION : TRUE, SAFE : FALSE*/
        if (!res.collision){
            OMPL_INFORM("NO SELF COLLISION");
            std::cout <<  ik_left.transpose() << " " << ik_right.transpose() << std::endl;
            return false;}
        
        OMPL_WARN("SELF COLLISION");
        return true;
    }


    virtual bool isValid(const ob::State *state) const
    {
        // cast the abstract state type to the type we expect
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        Affine3d base_object;
        base_object.translation() = Vector3d(pos->values[0], pos->values[1], pos->values[2]);
        base_object.linear() = Quaterniond(rot->w, rot->x, rot->y, rot->z).toRotationMatrix();
        
        Affine3d left_Lgrasp, right_Rgrasp; //defined by each robot frame
        left_Lgrasp = base_left.inverse() * base_object * obj_Lgrasp;
        right_Rgrasp = base_right.inverse() * base_object * obj_Rgrasp;


        // // return false;
        // if (!ObjectWorldCollision(base_object))
        //     return ik_solver(left_Lgrasp, right_Rgrasp);
        // OMPL_WARN("FOUND CONTACT");
        // return false;

        // return ik_solver(left_Lgrasp, right_Rgrasp);
        if (!ObjectWorldCollision(base_object)){
            OMPL_INFORM(" OBJECT VALID ");
            return true;
            // return ik_solver(left_Lgrasp, right_Rgrasp);    
        }
        
        else {
            // OMPL_WARN(" INVALID OBJECT STATE ");
            return false;}
        // return (!ObjectWorldCollision(base_object));
    }
};