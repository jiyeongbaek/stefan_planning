#include <iostream>
#include <fstream>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>
#include <ros/package.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "stefan_listener");
    ros::NodeHandle node;

    tf::TransformListener listener; //tf listener & transformations
    tf::StampedTransform t_stefan1;
    t_stefan1.setOrigin(tf::Vector3(0.95, 0, 0.72));
    t_stefan1.setRotation(tf::Quaternion(0, 0, 0.258819, 0.965926));

    std::vector<moveit_msgs::CollisionObject> collision_objects; //vector of objects
    std::vector<std::string> object_ids;                         //vector of strings (names of objects)
    object_ids.push_back("assembly");

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //planning interface
    moveit_msgs::CollisionObject stefan;                                         //Create an object msg

    shapes::Mesh *m = shapes::createMeshFromResource("package://grasping_point/STEFAN/stl/assembly.stl"); //find mesh
    shape_msgs::Mesh mesh;                                                                                //create a mesh msg
    shapes::ShapeMsg mesh_msg;                                                                            //create a shape msg
    shapes::constructMsgFromShape(m, mesh_msg);                                                           //convert shape into a shape msg
    ros::Duration(0.5).sleep();
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg); // shape msg is assigned to mesh msg

    stefan.header.frame_id = "base"; //"father frame"
    stefan.meshes.resize(1);         //scale
    stefan.meshes[0] = mesh;         //mesh

    stefan.mesh_poses.resize(1);            //vector resize
    stefan.mesh_poses[0].position.x = 0.95; //pose
    stefan.mesh_poses[0].position.y = 0;
    stefan.mesh_poses[0].position.z = 0.72;
    stefan.mesh_poses[0].orientation.x = 0;
    stefan.mesh_poses[0].orientation.y = 0;
    stefan.mesh_poses[0].orientation.z = 0.258819;
    stefan.mesh_poses[0].orientation.w = 0.965926;

    stefan.operation = stefan.ADD; //add object to collitions

    stefan.id = object_ids[0];           //rename object
    
    ros::Rate rate(1000);

    tf::Transform transform;
    tf::Quaternion q;
    tf::Vector3 pos;
    std::ifstream file(ros::package::getPath("stefan_planning") + "/object_state.txt");

    moveit_msgs::CollisionObject box_object;
    shape_msgs::SolidPrimitive box_table;
    geometry_msgs::Pose box_pose;
    box_table.type = box_table.BOX;
    box_table.dimensions.resize(3);
    box_table.dimensions[0] = 0.7;
    box_table.dimensions[1] = 0.6;
    box_table.dimensions[2] = 0.15;
    box_pose.position.x = 0.95;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.675;
    box_pose.orientation.w = 1.0;
    box_object.id = "sub_table";
    box_object.header.frame_id = "base";
    box_object.operation = box_object.ADD;
    box_object.primitives.push_back(box_table);
    box_object.primitive_poses.push_back(box_pose);

    moveit_msgs::CollisionObject box_object2;
    shape_msgs::SolidPrimitive box_table2;
    geometry_msgs::Pose box_pose2;
    box_table2.type = box_table2.BOX;
    box_table2.dimensions.resize(3);
    box_table2.dimensions[0] = 0.5;
    box_table2.dimensions[1] = 0.2;
    box_table2.dimensions[2] = 0.15;
    box_pose2.position.x = 0.5;
    box_pose2.position.y = -0.3;
    box_pose2.position.z = 0.675;
    box_pose2.orientation.w = 1.0;
    box_object2.id = "sub_table2";
    box_object2.header.frame_id = "base";
    box_object2.operation = box_object2.ADD;
    box_object2.primitives.push_back(box_table2);
    box_object2.primitive_poses.push_back(box_pose2);

    // planning_scene_interface.addCollisionObjects(box_object);
    // planning_scene_interface.addCollisionObjects(box_object2);

    collision_objects.push_back(stefan); //you can insert different objects using a vector of collition objects
    collision_objects.push_back(box_object);
    collision_objects.push_back(box_object2);
    planning_scene_interface.addCollisionObjects(collision_objects); //add objects to planning interface
    stefan.meshes.clear();          //Clear mesh required for MOVE operation (Only to avoid a warning)
    stefan.operation = stefan.MOVE; //change operation to MOVE



    while (ros::ok())
    {
        collision_objects.clear(); //clear previous data in the vector
        stefan.id = object_ids[0];

        while (file)
        {
            double data[7];
            bool eof = false;
            for (int i = 0; i < 7; i++)
            {
                if (!(file >> data[i]))
                {
                    eof = true;
                    break;
                }
            }
            stefan.mesh_poses[0].position.x = data[0];
            stefan.mesh_poses[0].position.y = data[1];
            stefan.mesh_poses[0].position.z = data[2];
            stefan.mesh_poses[0].orientation.x = data[3];
            stefan.mesh_poses[0].orientation.y = data[4];
            stefan.mesh_poses[0].orientation.z = data[5];
            stefan.mesh_poses[0].orientation.w = data[6];

            collision_objects.push_back(stefan); // add to vector

            planning_scene_interface.applyCollisionObjects(collision_objects); //apply changes to planning interface
            ros::Duration(0.1).sleep();                                          //ZzZzZ
        }
    }
    planning_scene_interface.removeCollisionObjects(object_ids); //delete objects from planning interface*/

    ros::shutdown(); //turn off
    return 0;
}