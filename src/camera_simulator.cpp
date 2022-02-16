#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <imagine/math/types.h>

// Include Sphere Simulators

#include <imagine/simulation/PinholeSimulatorEmbree.hpp>
#include <imagine/simulation/PinholeSimulatorOptix.hpp>

#include <sensor_msgs/PointCloud.h>

#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <imagine/util/StopWatch.hpp>


#include <dynamic_reconfigure/server.h>
#include <imagine_ros/CameraModelConfig.h>

#include <std_msgs/ColorRGBA.h>

#include <utility>

using namespace imagine;

static const std_msgs::ColorRGBA make_color(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA ret;
    ret.r = r;
    ret.g = g;
    ret.b = b;
    ret.a = a;
    return ret;
}

static const std_msgs::ColorRGBA make_color(float r, float g, float b)
{
    return make_color(r, g, b, 1.0);
}

std_msgs::ColorRGBA color_map[] = {
    make_color(1.0, 0.0, 0.0), // red
    make_color(0.0, 1.0, 0.0), // green
    make_color(0.0, 0.0, 1.0), // blue
    make_color(1.0, 1.0, 0.0), // yellow
    make_color(0.0, 1.0, 1.0), // cyan
    make_color(1.0, 0.0, 1.0), // purple
    make_color(1.0, 1.0, 1.0), // white
    make_color(0.0, 0.0, 0.0), // black
};


// EmbreeSimulatorPtr sim;

PinholeSimulatorOptixPtr sim_gpu;

Memory<PinholeModel, RAM> model;

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

Memory<PinholeModel, RAM> camera_model()
{
    Memory<PinholeModel, RAM> model(1);
    model->width = 400;
    model->height = 300;
    model->c[0] = 200.0;
    model->c[1] = 150.0;
    model->f[0] = 1000.0;
    model->f[1] = 1000.0;
    model->range.min = 0.0;
    model->range.max = 100.0;
    
    return model;
}

bool first_call = true;

void modelCB(imagine_ros::CameraModelConfig &config, uint32_t level) 
{
    if(first_call)
    {
        first_call = false;
        return;
    }

    ROS_INFO("Changing Model");

    // std::cout << level << std::endl;

    model->width = config.width;
    model->height = config.height;

    model->c[0] = config.cx;
    model->c[1] = config.cy;
    model->f[0] = config.fx;
    model->f[1] = config.fy;

    model->range.min = config.range_min;
    model->range.max = config.range_max;
    
    sim_gpu->setModel(model);
}

void fillPointCloud(
    const Memory<float, RAM>& ranges, 
    const Memory<unsigned int, RAM>& object_ids)
{
    cloud.points.resize(0);
    for(unsigned int vid = 0; vid < model->getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model->getWidth(); hid++)
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
    const Memory<Vector, RAM>& normals,
    const Memory<unsigned int, RAM>& object_ids)
{
    cloud_normals.points.resize(0);
    cloud_normals.colors.resize(0);
    for(unsigned int vid = 0; vid < model->getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model->getWidth(); hid++)
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

                unsigned int object_id = object_ids[buff_id];
                cloud_normals.colors.push_back(color_map[object_id]);
                cloud_normals.colors.push_back(color_map[object_id]);
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
    Memory<Transform, RAM> Tbm(1);
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

    using ResultT = Bundle<Ranges<VRAM_CUDA>, Normals<VRAM_CUDA>, ObjectIds<VRAM_CUDA> >;
    ResultT res;
    res.ranges.resize(Tbm_gpu.size() * model->size() );
    res.normals.resize(Tbm_gpu.size() * model->size() );
    res.object_ids.resize(Tbm_gpu.size() * model->size() );

    Memory<float, RAM> ranges;
    Memory<Vector, RAM> normals;
    Memory<unsigned int, RAM> object_ids;

    sw();
    sim_gpu->simulate(Tbm_gpu, res);
    el = sw();
    std::cout << "Simulated " << ranges.size() << " ranges, normals and object ids in " << el * 1000.0 << "ms" << std::endl;

    
    // Memory<Vector, VRAM_CUDA> normals_gpu(Tbm_gpu.size() * model->phi.size * model->theta.size);
    // sw();
    // sim_gpu->simulateNormals(Tbm_gpu, normals_gpu);
    // el = sw();
    // std::cout << "Simulated cloud normals in " << el * 1000.0 << "ms" << std::endl;

    // sw();
    // sim_gpu->simulate(Tbm_gpu, ranges_gpu, normals_gpu);
    // el = sw();
    // std::cout << "Simulated " << ranges_gpu.size() << " ranges and normals in " << el * 1000.0 << "ms" << std::endl;

    
    ranges = res.ranges;
    normals = res.normals;
    object_ids = res.object_ids;

    fillPointCloud(ranges, object_ids);
    fillCloudNormals(ranges, normals, object_ids);
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
    ros::init(argc, argv, "camera_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("camera_simulator_node started.");
    
    // hand crafted models
    // std::string mapfile = "/home/amock/workspaces/imagine_stack/imagine/dat/sphere.ply";
    // std::string mapfile = "/home/amock/workspaces/imagine_stack/imagine/dat/two_cubes.dae";
    // std::string mapfile = "/home/amock/workspaces/imagine_stack/imagine/dat/many_objects.dae";
    // real models
    // std::string mapfile = "/home/amock/workspaces/ros/mamcl_ws/src/uos_tools/uos_gazebo_worlds/Media/models/avz_neu.dae";
    // std::string mapfile = "/home/amock/datasets/physics_building/physics.dae";

    std::string map_frame;
    std::string meshfile;

    nh_p.param<std::string>("file", meshfile, "/home/amock/datasets/physics_building/physics.dae");
    nh_p.param<std::string>("frame", map_frame, "map");

    // EmbreeMapPtr map = importEmbreeMap(mapfile);
    // sim = std::make_shared<EmbreeSimulator>(map);

    OptixMapPtr map_gpu = importOptixMap(meshfile);
    sim_gpu = std::make_shared<PinholeSimulatorOptix>(map_gpu);

    // Define Sensor Model
    model = camera_model();
    sim_gpu->setModel(model);

    // Define Sensor to Base transform
    Memory<Transform, RAM> Tsb(1);
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
    cloud_normals.scale.x = 0.02;
    cloud_normals.id = 0;
    cloud_normals.color.r = 1.0;
    cloud_normals.color.a = 1.0;
    marker_pub = nh_p.advertise<visualization_msgs::Marker>("cloud_normals", 1);


    dynamic_reconfigure::Server<imagine_ros::CameraModelConfig> server;
    dynamic_reconfigure::Server<imagine_ros::CameraModelConfig>::CallbackType f;

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

    // Segfault if not resetting it here
    sim_gpu.reset();

    return 0;
}

