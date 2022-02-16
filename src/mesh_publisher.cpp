#include <ros/ros.h>

#include <mesh_msgs/MeshGeometry.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/TriangleIndices.h>

#include <imagine/map/EmbreeMap.hpp>

using namespace imagine;

// mesh_msgs::MeshGeometry assimpToRos(const aiMesh* mesh)
// {
//     mesh_msgs::MeshGeometry mesh_ros;

//     for(int i=0; i<mesh->mNumVertices; i++)
//     {
//         const aiVector3D& vertex = mesh->mVertices[i];
        
//         geometry_msgs::Point vertex_ros;
//         vertex_ros.x = vertex.x;
//         vertex_ros.y = vertex.y;
//         vertex_ros.z = vertex.z;

//         mesh_ros.vertices.push_back(vertex_ros);
//     }

//     for(int i=0; i<mesh->mNumFaces; i++)
//     {
//         const aiFace& face = mesh->mFaces[i];

//         mesh_msgs::TriangleIndices face_ros;
//         if(face.mNumIndices > 3) 
//         {
//             std::cout << "WARNING: found more than 3 indices in triangle." << std::endl;
//         }
//         face_ros.vertex_indices[0] = face.mIndices[0];
//         face_ros.vertex_indices[1] = face.mIndices[1];
//         face_ros.vertex_indices[2] = face.mIndices[2];

//         mesh_ros.faces.push_back(face_ros);
//     }

//     return mesh_ros;
// }

mesh_msgs::MeshGeometry embreeToRos(const EmbreeMesh& mesh)
{
    mesh_msgs::MeshGeometry mesh_ros;

    // Vertices
    for(int i=0; i<mesh.Nvertices; i++)
    {
        geometry_msgs::Point vertex_ros;
        vertex_ros.x = mesh.vertices[i * 3 + 0];
        vertex_ros.y = mesh.vertices[i * 3 + 1];
        vertex_ros.z = mesh.vertices[i * 3 + 2];
        mesh_ros.vertices.push_back(vertex_ros);
    }

    // Faces
    for(int i=0; i<mesh.Nfaces; i++)
    {
        mesh_msgs::TriangleIndices face_ros;
        face_ros.vertex_indices[0] = mesh.faces[i * 3 + 0];
        face_ros.vertex_indices[1] = mesh.faces[i * 3 + 1];
        face_ros.vertex_indices[2] = mesh.faces[i * 3 + 2];
        mesh_ros.faces.push_back(face_ros);
    }

    return mesh_ros;
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

    std::vector<mesh_msgs::MeshGeometryStamped> meshes_ros;

    for(size_t i=0; i<map->meshes.size(); i++)
    {
        mesh_msgs::MeshGeometryStamped mesh_ros;
        mesh_ros.header.frame_id = map_frame;
        mesh_ros.header.stamp = ros::Time::now();
        std::stringstream ss;
        ss << i;
        mesh_ros.uuid = ss.str();
        std::cout << "Converting Mesh " << ss.str() << std::endl;
        mesh_ros.mesh_geometry = embreeToRos(map->meshes[i]);
        meshes_ros.push_back(mesh_ros);
    }

    std::vector<ros::Publisher> mesh_pubs;

    for(auto mesh : meshes_ros)
    {
        std::stringstream ss;
        ss << "mesh/" << mesh.uuid;
        mesh_pubs.push_back(nh_p.advertise<mesh_msgs::MeshGeometryStamped>(ss.str(), 1));
    }

    // ros::Publisher mesh_pub = nh_p.advertise<mesh_msgs::MeshGeometryStamped>("mesh", 1);

    ros::Rate r(0.1);

    std::cout << "Publishing Mesh ..." << std::endl;
    while(ros::ok())
    {
        for(size_t i=0; i<meshes_ros.size(); i++)
        {
            auto mesh = meshes_ros[i];
            auto pub = mesh_pubs[i];
            mesh.header.stamp = ros::Time::now();
            pub.publish(mesh);
        }
        
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
