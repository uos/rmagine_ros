#include <ros/ros.h>

#include <mesh_msgs/MeshGeometry.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/TriangleIndices.h>

#include <rmagine/map/EmbreeMap.hpp>

using namespace rmagine;

mesh_msgs::MeshGeometry embreeToRos(EmbreeMeshPtr mesh, Matrix4x4 T)
{
    mesh_msgs::MeshGeometry mesh_ros;

    // Vertices
    auto vertices = mesh->verticesTransformed();
    for(int i=0; i<vertices.size(); i++)
    {
        geometry_msgs::Point vertex_ros;
        // mesh->ver
        auto vt = T * vertices[i];
        vertex_ros.x = vt.x;
        vertex_ros.y = vt.y;
        vertex_ros.z = vt.z;
        mesh_ros.vertices.push_back(vertex_ros);
    }

    // Faces
    auto faces = mesh->faces();
    for(int i=0; i<faces.size(); i++)
    {
        mesh_msgs::TriangleIndices face_ros;
        face_ros.vertex_indices[0] = faces[i].v0;
        face_ros.vertex_indices[1] = faces[i].v1;
        face_ros.vertex_indices[2] = faces[i].v2;
        mesh_ros.faces.push_back(face_ros);
    }

    return mesh_ros;
}

std::vector<mesh_msgs::MeshGeometry> embreeToRos(
    EmbreeScenePtr scene, 
    Matrix4x4 T = Matrix4x4::Identity())
{
    std::vector<mesh_msgs::MeshGeometry> ret;

    for(auto elem : scene->geometries())
    {
        size_t geom_id = elem.first;
        EmbreeInstancePtr inst = std::dynamic_pointer_cast<EmbreeInstance>(elem.second);
        if(inst)
        {
            Matrix4x4 T_ = T * inst->matrix();

            std::vector<mesh_msgs::MeshGeometry> ret_ = embreeToRos(inst->scene(), T_);
            ret.insert(ret.end(), ret_.begin(), ret_.end());
        } else {
            EmbreeMeshPtr mesh = std::dynamic_pointer_cast<EmbreeMesh>(elem.second);
            if(mesh)
            {
                // leaf
                ret.push_back(embreeToRos(mesh, T));
            }
        }
    }

    return ret;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh_p("~");

    std::string map_frame;
    std::string meshfile;

    nh_p.param<std::string>("file", meshfile, "/home/amock/ros_workspaces/amcl_flex/avz_floor.ply");
    nh_p.param<std::string>("frame", map_frame, "map");

    auto map = importEmbreeMap(meshfile);

    // std::vector<mesh_msgs::MeshGeometryStamped> meshes_ros;

    std::vector<mesh_msgs::MeshGeometry> meshes_ros = embreeToRos(map->scene);

    std::vector<mesh_msgs::MeshGeometryStamped> meshes_stamped(meshes_ros.size());

    for(size_t i=0; i<meshes_stamped.size(); i++)
    {
        std::stringstream ss;
        ss << "mesh/" << i;
        meshes_stamped[i].mesh_geometry = meshes_ros[i];
        meshes_stamped[i].header.frame_id = map_frame;
        meshes_stamped[i].uuid = ss.str();
        std::cout << meshes_stamped[i].uuid << std::endl;
    }

    // ros::Publisher mesh_pub = nh_p.advertise<mesh_msgs::MeshGeometryStamped>("map_mesh", 100);

    std::vector<ros::Publisher> mesh_pubs;
    for(size_t i=0; i<meshes_stamped.size(); i++)
    {
        mesh_pubs.push_back(nh_p.advertise<mesh_msgs::MeshGeometryStamped>(meshes_stamped[i].uuid, 10));
    }

    ros::Rate r(0.1);

    std::cout << "Publishing Meshes (" << meshes_stamped.size() << ") ..." << std::endl;
    while(ros::ok())
    {
        for(size_t i=0; i<meshes_stamped.size(); i++)
        {
            auto mesh = meshes_stamped[i];
            auto pub = mesh_pubs[i];
            mesh.header.stamp = ros::Time::now();
            pub.publish(mesh);
        }
        
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
