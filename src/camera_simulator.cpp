#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rmagine/math/types.h>

// Include Sphere Simulators

#include <rmagine/simulation/SimulatorEmbree.hpp>
#include <rmagine/simulation/SimulatorOptix.hpp>



#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/GaussianNoiseCuda.hpp>
#include <rmagine/noise/RelGaussianNoiseCuda.hpp>
#include <rmagine/noise/UniformDustNoiseCuda.hpp>

#include <sensor_msgs/PointCloud.h>

#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <rmagine/util/StopWatch.hpp>


#include <dynamic_reconfigure/server.h>
#include <rmagine_ros/CameraModelConfig.h>

#include <std_msgs/ColorRGBA.h>

#include <utility>
#include <shared_mutex>

using namespace rmagine;

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


SimulatorPtr<PinholeModel, Embree> sim_cpu;
NoisePtr noise_cpu_gauss;
NoisePtr noise_cpu_rel_gauss;
NoisePtr noise_cpu_dust;

SimulatorPtr<PinholeModel, Optix> sim_gpu;
NoiseCudaPtr noise_gpu_gauss;
NoiseCudaPtr noise_gpu_rel_gauss;
NoiseCudaPtr noise_gpu_dust;

std::shared_mutex updater_mutex;

PinholeModel model;

// user inputs
bool pose_received = false;

bool use_gpu = false;
bool noise_enabled = false;
float noise_mean = 0.0;
float noise_stddev = 0.0;

ros::Publisher cloud_pub;
sensor_msgs::PointCloud cloud;
ros::Publisher marker_pub;
visualization_msgs::Marker cloud_normals;

// ray viz
ros::Publisher ray_pub;
visualization_msgs::Marker ray_marker;

std::string sensor_frame = "sensor_link";
std::string base_frame = "base_link";

geometry_msgs::TransformStamped T_sensor_base;
geometry_msgs::TransformStamped T_base_map;

PinholeModel camera_model()
{
    PinholeModel model;
    model.width = 200;
    model.height = 150;
    model.c[0] = 100.0; // ~ half of width
    model.c[1] = 75.0; // ~ half of height
    model.f[0] = 100.0;
    model.f[1] = 100.0;
    model.range.min = 0.0;
    model.range.max = 100.0;
    return model;
}

bool first_call = true;

void modelCB(rmagine_ros::CameraModelConfig &config, uint32_t level) 
{
    // if(first_call)
    // {
    //     first_call = false;
    //     return;
    // }

    ROS_INFO("Changing Model");

    model.width = config.width;
    model.height = config.height;

    model.c[0] = config.cx;
    model.c[1] = config.cy;
    model.f[0] = config.fx;
    model.f[1] = config.fy;

    model.range.min = config.range_min;
    model.range.max = config.range_max;

    sim_cpu->setModel(model);
    sim_gpu->setModel(model);

    if(config.computing_unit == 0)
    {
        use_gpu = false;
    } else if(config.computing_unit == 1) {
        use_gpu = true;
    }

    
    // Noise

    NoiseCuda::Options opt_gpu = {};
    Noise::Options opt_cpu = {};

    opt_gpu.max_range = model.range.max;
    opt_cpu.max_range = model.range.max;

    if(config.noise_gauss)
    {   
        noise_cpu_gauss = std::make_shared<GaussianNoise>(config.noise_gauss_mean, config.noise_gauss_stddev, opt_cpu);
        noise_gpu_gauss = std::make_shared<GaussianNoiseCuda>(config.noise_gauss_mean, config.noise_gauss_stddev, opt_gpu);
    } else {
        noise_cpu_gauss.reset();
        noise_gpu_gauss.reset();
    }

    if(config.noise_rel_gauss)
    {
        noise_cpu_rel_gauss = std::make_shared<RelGaussianNoise>(
            config.noise_rel_gauss_mean, 
            config.noise_rel_gauss_stddev,
            config.noise_rel_gauss_range_exp,
            opt_cpu);
        noise_gpu_rel_gauss = std::make_shared<RelGaussianNoiseCuda>(
            config.noise_rel_gauss_mean, 
            config.noise_rel_gauss_stddev,
            config.noise_rel_gauss_range_exp,
            opt_gpu);
    } else {
        noise_cpu_rel_gauss.reset();
        noise_gpu_rel_gauss.reset();
    }

    if(config.noise_dust)
    {
        noise_cpu_dust = std::make_shared<UniformDustNoise>(config.noise_dust_hit_prob, config.noise_dust_ret_prob, opt_cpu);
        noise_gpu_dust = std::make_shared<UniformDustNoiseCuda>(config.noise_dust_hit_prob, config.noise_dust_ret_prob, opt_gpu);
    } else {
        noise_cpu_dust.reset();
        noise_gpu_dust.reset();
    }
}

void fillPointCloud(
    const Memory<float, RAM>& ranges, 
    const Memory<unsigned int, RAM>& object_ids)
{
    cloud.points.resize(0);
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            unsigned int buff_id = model.getBufferId(vid, hid);
            float range = ranges[buff_id];
            
            if(model.range.inside(range))
            {
                Vector ray = model.getDirection(vid, hid);
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
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            unsigned int buff_id = model.getBufferId(vid, hid);
            float range = ranges[buff_id];
            
            if(model.range.inside(range))
            {
                Vector ray = model.getDirection(vid, hid);
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
                if(object_id == UINT_MAX)
                {
                    // TODO: fix
                    object_id = 0;
                }
                cloud_normals.colors.push_back(color_map[object_id]);
                cloud_normals.colors.push_back(color_map[object_id]);
            }
        }
    }
}

void fillRayMarker(
    const Memory<float, RAM>& ranges)
{
    ray_marker.points.resize(0);
    ray_marker.colors.resize(0);
    for(unsigned int vid = 0; vid < model.getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < model.getWidth(); hid++)
        {
            unsigned int buff_id = model.getBufferId(vid, hid);
            float range = ranges[buff_id];
            
            if(model.range.inside(range))
            {
                Vector ray = model.getDirection(vid, hid);
                Point p_int = ray * range;
                
                geometry_msgs::Point p_int_ros;
                geometry_msgs::Point p_orig_ros;

                p_int_ros.x = p_int.x;
                p_int_ros.y = p_int.y;
                p_int_ros.z = p_int.z;

                p_orig_ros.x = 0.0;
                p_orig_ros.y = 0.0;
                p_orig_ros.z = 0.0;

                ray_marker.points.push_back(p_orig_ros);
                ray_marker.points.push_back(p_int_ros);
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

    using ResultTCPU = Bundle<
        Ranges<RAM>, 
        Normals<RAM>, 
        ObjectIds<RAM> >;

    ResultTCPU res_cpu;

    if(use_gpu)
    {
        using ResultTGPU = Bundle<Ranges<VRAM_CUDA>, Normals<VRAM_CUDA>, ObjectIds<VRAM_CUDA> >;
        ResultTGPU res_gpu;
        res_gpu.ranges.resize(Tbm_gpu.size() * model.size() );
        res_gpu.normals.resize(Tbm_gpu.size() * model.size() );
        res_gpu.object_ids.resize(Tbm_gpu.size() * model.size() );

        sw();
        sim_gpu->simulate(Tbm_gpu, res_gpu);
        if(noise_gpu_gauss)
        {
            noise_gpu_gauss->apply(res_gpu.ranges);
        }
        if(noise_gpu_rel_gauss)
        {
            noise_gpu_rel_gauss->apply(res_gpu.ranges);
        }
        if(noise_gpu_dust)
        {
            noise_gpu_dust->apply(res_gpu.ranges);
        }
        el = sw();

        // download
        res_cpu.ranges = res_gpu.ranges;
        res_cpu.normals = res_gpu.normals;
        res_cpu.object_ids = res_gpu.object_ids;
    } else {
        res_cpu.ranges.resize(Tbm.size() * model.size() );
        res_cpu.normals.resize(Tbm.size() * model.size() );
        res_cpu.object_ids.resize(Tbm.size() * model.size() );

        sw();
        sim_cpu->simulate(Tbm, res_cpu);
        if(noise_cpu_gauss)
        {
            noise_cpu_gauss->apply(res_cpu.ranges);
        }
        if(noise_cpu_rel_gauss)
        {
            noise_cpu_rel_gauss->apply(res_cpu.ranges);
        }
        if(noise_cpu_dust)
        {
            noise_cpu_dust->apply(res_cpu.ranges);
        }
        el = sw();
    }

    if(use_gpu)
    {
        std::cout << "[GPU] ";
    } else {
        std::cout << "[CPU] ";
    }

    std::cout << "Simulated " << res_cpu.ranges.size() << " ranges, normals and object ids in " << el * 1000.0 << "ms" << std::endl;

    fillPointCloud(res_cpu.ranges, res_cpu.object_ids);
    fillCloudNormals(res_cpu.ranges, res_cpu.normals, res_cpu.object_ids);
    fillRayMarker(res_cpu.ranges);
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
    // std::string mapfile = "/home/amock/workspaces/rmagine_stack/rmagine/dat/sphere.ply";
    // std::string mapfile = "/home/amock/workspaces/rmagine_stack/rmagine/dat/two_cubes.dae";
    // std::string mapfile = "/home/amock/workspaces/rmagine_stack/rmagine/dat/many_objects.dae";
    // real models
    // std::string mapfile = "/home/amock/workspaces/ros/mamcl_ws/src/uos_tools/uos_gazebo_worlds/Media/models/avz_neu.dae";
    // std::string mapfile = "/home/amock/datasets/physics_building/physics.dae";

    std::string map_frame;
    std::string meshfile;
    std::vector<float> Tsb_raw;
    Transform Tsb;

    nh_p.param<std::string>("map_file", meshfile, "/home/amock/datasets/physics_building/physics.dae");
    nh_p.param<std::string>("map_frame", map_frame, "map");
    nh_p.param<std::string>("sensor_frame", sensor_frame, "sensor_frame");

    nh_p.param<std::vector<float> >("Tsb", Tsb_raw, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Tsb.t.x = Tsb_raw[0];
    Tsb.t.y = Tsb_raw[1];
    Tsb.t.z = Tsb_raw[2];
    EulerAngles euler = {Tsb_raw[3], Tsb_raw[4], Tsb_raw[5]};
    Tsb.R = euler;

    EmbreeMapPtr map_cpu = import_embree_map(meshfile);
    sim_cpu = std::make_shared<PinholeSimulatorEmbree>(map_cpu);

    OptixMapPtr map_gpu = import_optix_map(meshfile);
    sim_gpu = std::make_shared<PinholeSimulatorOptix>(map_gpu);

    // Define Sensor Model
    model = camera_model();
    sim_gpu->setModel(model);
    sim_cpu->setModel(model);

    // Set Sensor to Base transform
    sim_gpu->setTsb(Tsb);
    sim_cpu->setTsb(Tsb);

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


    ray_marker.header.frame_id = sensor_frame;
    ray_marker.type = visualization_msgs::Marker::LINE_LIST;
    ray_marker.action = visualization_msgs::Marker::ADD;
    ray_marker.pose.orientation.w = 1.0;
    ray_marker.scale.x = 0.01;
    ray_marker.id = 0;
    ray_marker.color.r = 1.0;
    ray_marker.color.a = 0.2;

    ray_pub = nh_p.advertise<visualization_msgs::Marker>("rays", 1);

    dynamic_reconfigure::Server<rmagine_ros::CameraModelConfig> server;
    dynamic_reconfigure::Server<rmagine_ros::CameraModelConfig>::CallbackType f;

    f = boost::bind(&modelCB, _1, _2);
    server.setCallback(f);


    // Wait for pose to come in
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, poseCB);

    ROS_INFO("Waiting for pose estimation: Open RViz, click 2D Pose Estimate");

    // CONTINUOUS TF TREE UPDATES
    T_sensor_base.header.frame_id = base_frame;
    T_sensor_base.child_frame_id = sensor_frame;
    T_sensor_base.transform.rotation.x = Tsb.R.x;
    T_sensor_base.transform.rotation.y = Tsb.R.y;
    T_sensor_base.transform.rotation.z = Tsb.R.z;
    T_sensor_base.transform.rotation.w = Tsb.R.w;
    T_sensor_base.transform.translation.x = Tsb.t.x;
    T_sensor_base.transform.translation.y = Tsb.t.y;
    T_sensor_base.transform.translation.z = Tsb.t.z;

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

            ray_marker.header.stamp = ros::Time::now();
            ray_pub.publish(ray_marker);
        }
        
        ros::spinOnce();
        r.sleep();
    }

    // Segfault if not resetting it here
    sim_gpu.reset();

    return 0;
}

