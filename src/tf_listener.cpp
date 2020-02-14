#include <iostream>
#include <fstream>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <tf/transform_listener.h>

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
    collision_objects.push_back(stefan); //you can insert different objects using a vector of collition objects

    planning_scene_interface.addCollisionObjects(collision_objects); //add objects to planning interface

    stefan.meshes.clear();          //Clear mesh required for MOVE operation (Only to avoid a warning)
    stefan.operation = stefan.MOVE; //change operation to MOVE

    ros::Rate rate(1000);

    tf::Transform transform;
    tf::Quaternion q;
    tf::Vector3 pos;
    std::ifstream file("/home/jiyeong/catkin_ws/src/inverse_kinematics/object_state.txt");

    while (ros::ok())
    {
        collision_objects.clear(); //clear previous data in the vector
        stefan.id = object_ids[0];          

        while (file){
            double data[7];
            bool eof = false;
            for(int i=0; i<7; i++)
            {
                if (!(file >> data[i]))
                {eof = true;break;}
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
        ros::Duration(2).sleep();                                        //ZzZzZ
        }
    }
    planning_scene_interface.removeCollisionObjects(object_ids); //delete objects from planning interface*/

    ros::shutdown(); //turn off
    return 0;
}