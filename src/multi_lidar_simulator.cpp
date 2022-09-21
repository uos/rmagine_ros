#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rmagine/math/types.h>

// Include Sphere Simulators

#include <rmagine/simulation/SimulatorEmbree.hpp>
#include <rmagine/simulation/SimulatorOptix.hpp>
#include <rmagine/util/prints.h>

// #include <rmagine/noise/noise.cuh>

#include <sensor_msgs/PointCloud.h>

#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <rmagine/util/StopWatch.hpp>


#include <dynamic_reconfigure/server.h>
#include <rmagine_ros/LidarModelConfig.h>

#include <std_msgs/ColorRGBA.h>

// rosmath
#include <rosmath/sensor_msgs/conversions.h>
#include <rosmath/sensor_msgs/math.h>
#include <rosmath/eigen/conversions.h>
#include <rosmath/eigen/stats.h>


#include <utility>

#include <geometry_msgs/PoseArray.h>


using namespace rmagine;
using namespace rosmath;

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

SimulatorPtr<LiDARModel, Embree> sim_cpu;
SimulatorPtr<LiDARModel, Optix> sim_gpu;

Transform Tbm;
ros::Time   time_begin;
Memory<Transform, RAM>          poses_ram;
Memory<Transform, VRAM_CUDA>    poses_vram;

size_t Nposes = 100;

Transform Tsb;
LiDARModel model;

// user inputs
bool pose_received = false;

bool use_gpu = false;
bool noise_enabled = false;
float noise_mean = 0.0;
float noise_stddev = 0.0;

ros::Publisher cloud_pub;
sensor_msgs::PointCloud cloud;
std::vector<sensor_msgs::PointCloud> clouds;

ros::Publisher poses_pub;
geometry_msgs::PoseArray pose_arr;

ros::Publisher pose_points_pub;
sensor_msgs::PointCloud pose_points;


std::string sensor_frame = "sensor_link";
std::string base_frame = "base_link";

geometry_msgs::TransformStamped T_sensor_base;
geometry_msgs::TransformStamped T_base_map;

LiDARModel velodyne_model()
{
    LiDARModel model;
    model.theta.min = -M_PI;
    model.theta.inc = 0.4 * M_PI / 180.0;
    model.theta.size = 900;
    
    model.phi.min = -15.0 * M_PI / 180.0;
    model.phi.inc = 2.0 * M_PI / 180.0;
    model.phi.size = 16;
    
    model.range.min = 0.0;
    model.range.max = 100.0;
    return model;
}

Memory<Transform, RAM> random_poses(Transform Tmean, size_t N)
{
    Memory<Transform, RAM> poses_ram(N);

    Eigen::Matrix<double, 6, 6> cov;
    cov <<  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 6, 1> mean;
    mean(0) = Tmean.t.x;
    mean(1) = Tmean.t.y;
    mean(2) = Tmean.t.z;
    EulerAngles euler;
    euler.set(Tmean.R);
    mean(3) = euler.roll;
    mean(4) = euler.pitch;
    mean(5) = euler.yaw;

    stats::Normal normal_dist(mean, cov);

    for(size_t i=0; i<N; i++)
    {
        poses_ram[i] = Tmean;
        // auto particle = normal_dist.sample();
        // poses_ram[i].t.x = particle(0);
        // poses_ram[i].t.y = particle(1);
        // poses_ram[i].t.z = particle(2);
        // EulerAngles rpy;
        // rpy.roll = particle(3);
        // rpy.pitch = particle(4);
        // rpy.yaw = particle(5);
        // poses_ram[i].R.set(rpy);
    }

    time_begin = ros::Time::now();

    return poses_ram;
}

void populate()
{
    Eigen::Matrix<double, 6, 6> cov;
    cov <<  0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    for(size_t i=0; i<poses_ram.size(); i++)
    {
        Transform Tmean = poses_ram[i];
        Eigen::Matrix<double, 6, 1> mean;
        mean(0) = Tmean.t.x;
        mean(1) = Tmean.t.y;
        mean(2) = Tmean.t.z;
        EulerAngles euler;
        euler.set(Tmean.R);
        mean(3) = euler.roll;
        mean(4) = euler.pitch;
        mean(5) = euler.yaw;

        stats::Normal normal_dist(mean, cov);

        auto particle = normal_dist.sample();
        poses_ram[i].t.x = particle(0);
        poses_ram[i].t.y = particle(1);
        poses_ram[i].t.z = particle(2);
        EulerAngles rpy;
        rpy.roll = particle(3);
        rpy.pitch = particle(4);
        rpy.yaw = particle(5);
        poses_ram[i].R.set(rpy);
    }

    poses_vram = poses_ram;
}

void populate_sin()
{
    double secs = (ros::Time::now() - time_begin).toSec();

    double nposes_d = static_cast<double>(poses_ram.size());
    double angle_inc = 2.0 * M_PI / nposes_d;
    

    double speed = 0.5;
    double radius = 10.0;

    for(size_t i=0; i<poses_ram.size(); i++)
    {

        double length = (-cos(secs * speed) + 1.0) / 2.0;

        length = (length * length) * radius;
        double angle = i * angle_inc;
        
        Vector dir;
        dir.x = cos(angle);
        // dir.x = 0.0;
        dir.y = sin(angle);
        dir.z = 0.0;

        poses_ram[i] = Tbm;
        poses_ram[i].t += dir * length;
    }

    poses_vram = poses_ram;
}



void modelCB(rmagine_ros::LidarModelConfig &config, uint32_t level) 
{
    ROS_INFO("Changing Model");

    model.theta.min = config.theta_min;
    model.theta.inc = DiscreteInterval::IncFromMinMaxSize(config.theta_min, config.theta_max, config.theta_size);
    model.theta.size = config.theta_size;

    model.phi.min = config.phi_min;
    model.phi.inc = DiscreteInterval::IncFromMinMaxSize(config.phi_min, config.phi_max, config.phi_size);
    model.phi.size = config.phi_size;

    model.range.min = config.range_min;
    model.range.max = config.range_max;

    sim_gpu->setModel(model);
    sim_cpu->setModel(model);

    if(config.computing_unit == 0)
    {
        use_gpu = false;
    } else if(config.computing_unit == 1) {
        use_gpu = true;
    }
}

void fillPoseArray()
{
    pose_arr.poses.resize(poses_ram.size());

    for(size_t i=0; i<poses_ram.size(); i++)
    {
        pose_arr.poses[i].position.x = poses_ram[i].t.x;
        pose_arr.poses[i].position.y = poses_ram[i].t.y;
        pose_arr.poses[i].position.z = poses_ram[i].t.z;
        pose_arr.poses[i].orientation.x = poses_ram[i].R.x;
        pose_arr.poses[i].orientation.y = poses_ram[i].R.y;
        pose_arr.poses[i].orientation.z = poses_ram[i].R.z;
        pose_arr.poses[i].orientation.w = poses_ram[i].R.w;
    }
}

void fillPosePoints()
{
    pose_points.points.resize(poses_ram.size());

    for(size_t i=0; i<poses_ram.size(); i++)
    {
        pose_points.points[i].x = poses_ram[i].t.x;
        pose_points.points[i].y = poses_ram[i].t.y;
        pose_points.points[i].z = poses_ram[i].t.z;
    }
}

void fillPointClouds(
    const Memory<float, RAM>& ranges_, 
    const Memory<unsigned int, RAM>& object_ids_)
{
    clouds.resize(poses_ram.size());
    
    #pragma omp parallel for
    for(size_t i=0; i<poses_ram.size(); i++)
    {
        sensor_msgs::PointCloud& cloud = clouds[i];

        Transform Tbm = poses_ram[i];
        auto ranges = ranges_(i * model.size(), (i+1) * model.size());
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
                    Point p_sensor = ray * range;
                    Point p_map = Tbm * Tsb * p_sensor;

                    geometry_msgs::Point32 p_ros;
                    p_ros.x = p_map.x;
                    p_ros.y = p_map.y;
                    p_ros.z = p_map.z;
                    cloud.points.push_back(p_ros);
                }
            }
        }
    }
}


void fillPointCloud(
    const Memory<float, RAM>& ranges_, 
    const Memory<unsigned int, RAM>& object_ids_)
{
    // cloud.resize(poses_ram.size());
    cloud.points.resize(ranges_.size());

    
    #pragma omp parallel for
    for(size_t i=0; i<poses_ram.size(); i++)
    {

        Transform Tbm = poses_ram[i];
        auto ranges = ranges_(i * model.size(), (i+1) * model.size());
        geometry_msgs::Point32* points = &cloud.points[i * model.size()];

        for(unsigned int vid = 0; vid < model.getHeight(); vid++)
        {
            for(unsigned int hid = 0; hid < model.getWidth(); hid++)
            {
                unsigned int buff_id = model.getBufferId(vid, hid);
                // unsigned int glob_id = buff_id + model.size() * i;

                float range = ranges[buff_id];

                if(model.range.inside(range))
                {
                    Vector ray = model.getDirection(vid, hid);
                    Point p_sensor = ray * range;
                    Point p_map = Tbm * Tsb * p_sensor;

                    geometry_msgs::Point32 p_ros;
                    p_ros.x = p_map.x;
                    p_ros.y = p_map.y;
                    p_ros.z = p_map.z;
                    points[buff_id] = p_ros;
                } else {
                    points[buff_id].x = std::numeric_limits<float>::quiet_NaN();
                    points[buff_id].y = std::numeric_limits<float>::quiet_NaN();
                    points[buff_id].z = std::numeric_limits<float>::quiet_NaN();

                    // points[i].x = std::numeric_limits<float>::quiet_NaN();
                    // points[i].y = std::numeric_limits<float>::quiet_NaN();
                    // points[i].z = std::numeric_limits<float>::quiet_NaN();
                }
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

    
    Tbm.t.x = msg->pose.pose.position.x;
    Tbm.t.y = msg->pose.pose.position.y;
    Tbm.t.z = msg->pose.pose.position.z;
    Tbm.R.x = msg->pose.pose.orientation.x;
    Tbm.R.y = msg->pose.pose.orientation.y;
    Tbm.R.z = msg->pose.pose.orientation.z;
    Tbm.R.w = msg->pose.pose.orientation.w;

    poses_ram = random_poses(Tbm, Nposes);
    poses_vram = poses_ram;

    pose_received = true;
}

void simulate()
{
    // Memory<Transform, RAM> Tbm(1);
    // Tbm->R.x = T_base_map.transform.rotation.x;
    // Tbm->R.y = T_base_map.transform.rotation.y;
    // Tbm->R.z = T_base_map.transform.rotation.z;
    // Tbm->R.w = T_base_map.transform.rotation.w;

    // Tbm->t.x = T_base_map.transform.translation.x;
    // Tbm->t.y = T_base_map.transform.translation.y;
    // Tbm->t.z = T_base_map.transform.translation.z;

    // Memory<Transform, VRAM_CUDA> Tbm_gpu;
    // Tbm_gpu = Tbm;

    StopWatch sw;
    double el;

    using ResultTCPU = Bundle<
        Ranges<RAM>, 
        Normals<RAM>, 
        ObjectIds<RAM> >;

    ResultTCPU res_cpu;

    if(use_gpu)
    {
        using ResultTGPU = Bundle<
        Ranges<VRAM_CUDA>, 
        Normals<VRAM_CUDA>, 
        ObjectIds<VRAM_CUDA> >;
    
        ResultTGPU res_gpu;
        res_gpu.ranges.resize(poses_vram.size() * model.size() );
        res_gpu.normals.resize(poses_vram.size() * model.size() );
        res_gpu.object_ids.resize(poses_vram.size() * model.size() );

        sw();
        sim_gpu->simulate(poses_vram, res_gpu);
        el = sw();

        // download
        res_cpu.ranges = res_gpu.ranges;
        res_cpu.normals = res_gpu.normals;
        res_cpu.object_ids = res_gpu.object_ids;

    } else {
        res_cpu.ranges.resize(poses_ram.size() * model.size() );
        res_cpu.normals.resize(poses_ram.size() * model.size() );
        res_cpu.object_ids.resize(poses_ram.size() * model.size() );

        sw();
        sim_cpu->simulate(poses_ram, res_cpu);
        el = sw();
    }
    
    // Memory<Vector, VRAM_CUDA> normals_gpu(Tbm_gpu.size() * model.phi.size * model.theta.size);
    // sw();
    // sim_gpu->simulateNormals(Tbm_gpu, normals_gpu);
    // el = sw();
    // std::cout << "Simulated cloud normals in " << el * 1000.0 << "ms" << std::endl;

    // sw();
    // sim_gpu->simulate(Tbm_gpu, ranges_gpu, normals_gpu);
    // el = sw();
    // std::cout << "Simulated " << ranges_gpu.size() << " ranges and normals in " << el * 1000.0 << "ms" << std::endl;
    
    // if(noise_enabled)
    // {
    //     GaussianNoise(noise_mean, noise_stddev).apply(res_cpu.ranges);
    // }

    if(use_gpu)
    {
        std::cout << "[GPU] ";
    } else {
        std::cout << "[CPU] ";
    }

    std::cout << "Simulated " << poses_ram.size() << " X " << model.size() << " ranges, normals and object ids in " << el * 1000.0 << "ms" << std::endl;


    sw();
    // fillPointClouds(res_cpu.ranges, res_cpu.object_ids);
    fillPointCloud(res_cpu.ranges, res_cpu.object_ids);
    // fillPoseArray();
    fillPosePoints();
    el = sw();
    std::cout << "Filled ROS buffers: " << el * 1000.0 << "ms" << std::endl;

}

// void updateTF()
// {
//     // tf update
//     static tf2_ros::TransformBroadcaster br;
//     T_base_map.header.stamp = ros::Time::now();
//     br.sendTransform(T_base_map);
//     T_sensor_base.header.stamp = ros::Time::now();
//     br.sendTransform(T_sensor_base);
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_lidar_simulator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("multi_lidar_simulator_node started.");
    

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
    

    nh_p.param<std::string>("map_file", meshfile, "/home/amock/datasets/physics_building/physics.dae");
    nh_p.param<std::string>("map_frame", map_frame, "map");
    nh_p.param<std::string>("sensor_frame", sensor_frame, "sensor_frame");

    nh_p.param<std::vector<float> >("Tsb", Tsb_raw, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    Tsb.t.x = Tsb_raw[0];
    Tsb.t.y = Tsb_raw[1];
    Tsb.t.z = Tsb_raw[2];
    EulerAngles euler = {Tsb_raw[3], Tsb_raw[4], Tsb_raw[5]};
    Tsb.R = euler;

    EmbreeMapPtr map_cpu = importEmbreeMap(meshfile);
    sim_cpu = std::make_shared<SphereSimulatorEmbree>(map_cpu);

    OptixMapPtr map_gpu = importOptixMap(meshfile);
    sim_gpu = std::make_shared<SphereSimulatorOptix>(map_gpu);

    // Define Sensor Model
    model = velodyne_model();
    sim_cpu->setModel(model);
    sim_gpu->setModel(model);
    sim_cpu->setTsb(Tsb);
    sim_gpu->setTsb(Tsb);

    // make point cloud publisher
    
    cloud_pub = nh_p.advertise<sensor_msgs::PointCloud>("cloud", 1);

    poses_pub = nh_p.advertise<geometry_msgs::PoseArray>("poses", 1);

    pose_points_pub = nh_p.advertise<sensor_msgs::PointCloud>("pose_points", 1);



    dynamic_reconfigure::Server<rmagine_ros::LidarModelConfig> server;
    dynamic_reconfigure::Server<rmagine_ros::LidarModelConfig>::CallbackType f;

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

            // updateTF();
            
            // for(auto cloud : clouds)
            // {
            //     cloud.header.frame_id = map_frame;
            //     cloud.header.stamp = ros::Time::now();
            //     cloud_pub.publish(cloud);
            // }

            {
                cloud.header.frame_id = map_frame;
                cloud.header.stamp = ros::Time::now();
                cloud_pub.publish(cloud);
            }

            // {
            //     pose_arr.header.frame_id = map_frame;
            //     pose_arr.header.stamp = ros::Time::now();
            //     poses_pub.publish(pose_arr);
            // }

            {
                pose_points.header.frame_id = map_frame;
                pose_points.header.stamp = ros::Time::now();
                pose_points_pub.publish(pose_points);
            }



            populate_sin();
            

            // cloud_normals.header.stamp = ros::Time::now();
            // marker_pub.publish(cloud_normals);

            // ray_marker.header.stamp = ros::Time::now();
            // ray_pub.publish(ray_marker);
        }
        
        ros::spinOnce();
        r.sleep();
    }

    // Segfault if not resetting it here
    sim_gpu.reset();
    sim_cpu.reset();

    return 0;
}

