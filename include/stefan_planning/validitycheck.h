#pragma once

#include <string>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>
#include <geometric_shapes/shape_operations.h>

#include <stefan_planning/fcl_eigen_utils.h>
#include <stefan_planning/vtk_mesh_utils.h>

#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/BV/BV.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/narrowphase/narrowphase.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fcl/broadphase/broadphase.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

#include <ros/package.h>
#include <cmath>

typedef fcl::OBBRSS BV;
typedef fcl::BVHModel<BV> BVHM;
typedef std::shared_ptr<BVHM> BVHMPtr;
using fcl::Box;
typedef std::shared_ptr<fcl::Box> BoxPtr;
using fcl::CollisionObject;
typedef std::shared_ptr<fcl::CollisionObject> CollisionObjectPtr;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudT;
typedef pcl::PointXYZRGBNormal PointT;
typedef std::shared_ptr<PointT> PointTPtr;

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;
using namespace Eigen;
using namespace pcl;

class ValidityCheck : public ob::StateValidityChecker
{
public:
    BVHMPtr mesh_model_;
    BoxPtr table_model_;
    fcl::Transform3f table_transform;
    std::string file_name;

    ValidityCheck(ros::NodeHandle nh, const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si)
    {
        // std::string file_name = ros::package::getPath("grasping_point") + "/STEFAN/stl/assembly.stl";
        file_name = "/home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly_fcl.stl";
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFile(file_name, mesh);
        std::vector<TrianglePlaneData> triangles = buildTriangleData(mesh);
        loadMesh(triangles);

        table_model_ = std::make_shared<Box>(1.0, 0.6, 0.15);
        Eigen::Isometry3d table_t_;
        table_t_.setIdentity();
        table_t_.translation() << 0.95, 0.0, 1.07;

        FCLEigenUtils::convertTransform(table_t_, table_transform);
    }
    void loadMesh(const std::vector<TrianglePlaneData> &mesh)
    {
        std::vector<fcl::Vec3f> points;
        std::vector<fcl::Triangle> triangles;
        mesh_model_ = std::make_shared<BVHM>();

        for (const auto &tri_plane : mesh)
        {
            fcl::Triangle tri;

            for (int i = 0; i < 3; i++)
            {
                tri[i] = points.size();
                points.push_back(
                    fcl::Vec3f(
                        tri_plane.points[i](0),
                        tri_plane.points[i](1),
                        tri_plane.points[i](2)));
            }
            triangles.push_back(tri);
        }
        mesh_model_->beginModel();
        mesh_model_->addSubModel(points, triangles);
        mesh_model_->endModel();
    }

    bool isFeasible(Eigen::Isometry3d mesh_transform) const
    {
        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        fcl::Transform3f init;
        init.setIdentity();

        bool is_collided = false;
        fcl::Transform3f fcl_transform;
        FCLEigenUtils::convertTransform(mesh_transform, fcl_transform);

        fcl::collide(mesh_model_.get(), fcl_transform, table_model_.get(), table_transform,
                     request, result);

        return !result.isCollision();
        // if (result.isCollision() == true)
        // {
        //     is_collided = true;
        // }

        // return !is_collided;
    }

    bool isValid(const ob::State *state) const
    {
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        Isometry3d base_object;
        base_object.setIdentity();
        base_object.translation() = Vector3d(pos->values[0], pos->values[1], pos->values[2]);
        base_object.linear() = Quaterniond(rot->w, rot->x, rot->y, rot->z).toRotationMatrix();
        return isFeasible(base_object);
    }
};