#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <imagine/simulation/OptixSimulator.hpp>


#include <imagine/simulation/EmbreeSimulator.hpp>
#include <imagine/math/types.h>

#include <sensor_msgs/PointCloud.h>

#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <imagine/util/StopWatch.hpp>


#include <dynamic_reconfigure/server.h>
#include <imagine_ros/LidarModelConfig.h>


using namespace imagine;

// EmbreeSimulatorPtr sim;

OptixSimulatorPtr sim_gpu;

Memory<LiDARModel, RAM> model;

// user inputs
bool pose_received = false;

ros::Publisher cloud_pub;
sensor_msgs::PointCloud cloud;
ros::Publisher marker_pub;
visualization_msgs::Marker cloud_normals;

std::string sensor_frame = "sensor_link";
std::string base_frame = "base_link";

geometry_msgs::TransformStamped T_sensor_base;
geometry_msgs::TransformStamped T_base_map;

Memory<LiDARModel, RAM> velodyne_model()
{
    Memory<LiDARModel, RAM> model;
    model->theta.min = -M_PI;
    model->theta.max = M_PI;
    model->theta.size = 440;
    model->theta.computeStep();
    
    model->phi.min = -0.261799;
    model->phi.max = 0.261799;
    model->phi.size = 16;
    model->phi.computeStep();
    
    model->range.min = 0.5;
    model->range.max = 130.0;
    return model;
}

void modelCB(imagine_ros::LidarModelConfig &config, uint32_t level) 
{
    ROS_INFO("Changing Model");

    model->theta.min = config.theta_min;
    model->theta.max = config.theta_max;
    model->theta.size = config.theta_N;
    model->theta.computeStep();

    model->phi.min = config.phi_min;
    model->phi.max = config.phi_max;
    model->phi.size = config.phi_N;
    model->phi.computeStep();

    model->range.min = config.range_min;
    model->range.max = config.range_max;

    sim_gpu->setModel(model);
}

void fillPointCloud(const Memory<float, RAM>& ranges)
{
    cloud.points.resize(0);
    for(unsigned int vid = 0; vid < model->phi.size; vid++)
    {
        for(unsigned int hid = 0; hid < model->theta.size; hid++)
        {
            unsigned int buff_id = model->getBufferId(vid, hid);
            float range = ranges[buff_id];
            
            if(model->range.inside(range))
            {
                Vector ray = model->getRay(vid, hid);
                Point p = ray * range;
                geometry_msgs::Point32 p_ros;
                p_ros.x = p.x;
                p_ros.y = p.y;
                p_ros.z = p.z;
                cloud.points.push_back(p_ros);
            }
        }
    }
}

void fillCloudNormals(
    const Memory<float, RAM>& ranges,
    const Memory<Vector, RAM>& normals)
{
    cloud_normals.points.resize(0);
    for(unsigned int vid = 0; vid < model->phi.size; vid++)
    {
        for(unsigned int hid = 0; hid < model->theta.size; hid++)
        {
            unsigned int buff_id = model->getBufferId(vid, hid);
            float range = ranges[buff_id];
            
            if(model->range.inside(range))
            {
                Vector ray = model->getRay(vid, hid);
                Point p = ray * range;
                geometry_msgs::Point p_ros;
                p_ros.x = p.x;
                p_ros.y = p.y;
                p_ros.z = p.z;
                cloud_normals.points.push_back(p_ros);
                Vector normal = normals[buff_id];
                Point p_shifted = p + normal * 0.2;
                geometry_msgs::Point p_shifted_ros;
                p_shifted_ros.x = p_shifted.x;
                p_shifted_ros.y = p_shifted.y;
                p_shifted_ros.z = p_shifted.z;
                cloud_normals.points.push_back(p_shifted_ros);
            }
        }
    }
}

void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    T_base_map.header.frame_id = msg->header.frame_id;
    T_base_map.child_frame_id = base_frame;
    T_base_map.header.stamp = ros::Time::now();
    
    T_base_map.transform.translation.x = msg->pose.pose.position.x;
    T_base_map.transform.translation.y = msg->pose.pose.position.y;
    T_base_map.transform.translation.z = msg->pose.pose.position.z;
    T_base_map.transform.rotation = msg->pose.pose.orientation;

    pose_received = true;
}

void simulate()
{
    Memory<Transform, RAM> Tbm;
    Tbm->R.x = T_base_map.transform.rotation.x;
    Tbm->R.y = T_base_map.transform.rotation.y;
    Tbm->R.z = T_base_map.transform.rotation.z;
    Tbm->R.w = T_base_map.transform.rotation.w;

    Tbm->t.x = T_base_map.transform.translation.x;
    Tbm->t.y = T_base_map.transform.translation.y;
    Tbm->t.z = T_base_map.transform.translation.z;

    Memory<Transform, VRAM_CUDA> Tbm_gpu;
    Tbm_gpu = Tbm;

    StopWatch sw;
    double el;

    Memory<float, RAM> ranges;
    Memory<float, VRAM_CUDA> ranges_gpu(Tbm_gpu.size() * model->phi.size * model->theta.size);

    sw();
    sim_gpu->simulateRanges(Tbm_gpu, ranges_gpu);
    el = sw();
    std::cout << "Simulated cloud in " << el * 1000.0 << "ms" << std::endl;
    
    ranges = ranges_gpu;
    fillPointCloud(ranges);

    Memory<Vector, RAM> normals;
    Memory<Vector, VRAM_CUDA> normals_gpu(Tbm_gpu.size() * model->phi.size * model->theta.size);
    sw();
    sim_gpu->simulateNormals(Tbm_gpu, normals_gpu);
    el = sw();
    std::cout << "Simulated cloud normals in " << el * 1000.0 << "ms" << std::endl;

    normals = normals_gpu;
    fillCloudNormals(ranges, normals);
}

void updateTF()
{
    // tf update
    static tf2_ros::TransformBroadcaster br;
    T_base_map.header.stamp = ros::Time::now();
    br.sendTransform(T_base_map);
    T_sensor_base.header.stamp = ros::Time::now();
    br.sendTransform(T_sensor_base);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("lidar_simulator_node started.");
    
    // std::string mapfile = "/home/amock/workspaces/ros/mamcl_ws/src/uos_tools/uos_gazebo_worlds/Media/models/avz_neu.dae";
    // std::string mapfile = "/home/amock/workspaces/imagine/dat/sphere.ply";
    // std::string mapfile = "/home/amock/workspaces/imagine/dat/two_cubes.dae";
    std::string mapfile = "/home/amock/workspaces/imagine/dat/many_objects.dae";

    // EmbreeMapPtr map = importEmbreeMap(mapfile);
    // sim = std::make_shared<EmbreeSimulator>(map);

    OptixMapPtr map_gpu = importOptixMap(mapfile);
    sim_gpu = std::make_shared<OptixSimulator>(map_gpu);

    // Define Sensor Model
    model = velodyne_model();
    sim_gpu->setModel(model);

    // Define Sensor to Base transform
    Memory<Transform, RAM> Tsb;
    Tsb->setIdentity();
    // lift scanner up
    Tsb->t.z = 1.0;

    sim_gpu->setTsb(Tsb);

    // make point cloud publisher
    cloud.header.frame_id = sensor_frame;
    cloud_pub = nh_p.advertise<sensor_msgs::PointCloud>("cloud", 1);

    // make point normals publisher
    cloud_normals.header.frame_id = sensor_frame;
    cloud_normals.type = visualization_msgs::Marker::LINE_LIST;
    cloud_normals.action = visualization_msgs::Marker::ADD;
    cloud_normals.pose.orientation.w = 1.0;
    cloud_normals.scale.x = 0.01;
    cloud_normals.id = 0;
    cloud_normals.color.r = 1.0;
    cloud_normals.color.a = 1.0;
    marker_pub = nh_p.advertise<visualization_msgs::Marker>("cloud_normals", 1);


    dynamic_reconfigure::Server<imagine_ros::LidarModelConfig> server;
    dynamic_reconfigure::Server<imagine_ros::LidarModelConfig>::CallbackType f;

    f = boost::bind(&modelCB, _1, _2);
    server.setCallback(f);


    // Wait for pose to come in
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, poseCB);

    ROS_INFO("Waiting for pose estimation: Open RViz, click 2D Pose Estimate");

    // CONTINUOUS TF TREE UPDATES
    T_sensor_base.header.frame_id = base_frame;
    T_sensor_base.child_frame_id = sensor_frame;
    T_sensor_base.transform.rotation.x = Tsb->R.x;
    T_sensor_base.transform.rotation.y = Tsb->R.y;
    T_sensor_base.transform.rotation.z = Tsb->R.z;
    T_sensor_base.transform.rotation.w = Tsb->R.w;
    T_sensor_base.transform.translation.x = Tsb->t.x;
    T_sensor_base.transform.translation.y = Tsb->t.y;
    T_sensor_base.transform.translation.z = Tsb->t.z;

    ros::Rate r(20);

    while(ros::ok())
    {
        if(pose_received)
        {
            simulate();

            updateTF();

            cloud.header.stamp = ros::Time::now();
            cloud_pub.publish(cloud);

            cloud_normals.header.stamp = ros::Time::now();
            marker_pub.publish(cloud_normals);
        }
        
        ros::spinOnce();
        r.sleep();
    }

    sim_gpu.reset();

    return 0;
}

