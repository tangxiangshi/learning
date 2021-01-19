#include <ros/ros.h>
#include <cstring>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <queue>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>



// define pcl::PointXYZI as PointType
typedef pcl::PointXYZI PointType;

// initialize things
int Degree = 10;
Eigen::Vector3f initial_pose_guess(0, 0, 0);
geometry_msgs::PoseStamped initial_pose_guess_ros; 
Eigen::Vector3f point_pose(0,0,0);
Eigen::Quaternionf orientation(1, 0, 0, 0);

// down sample
pcl::VoxelGrid<PointType> voxel_grid;
pcl::PassThrough<PointType> pass_through;
// Iterative Point Cloud
pcl::IterativeClosestPoint<PointType, PointType> icp; // first_icp
pcl::IterativeClosestPoint<PointType, PointType> icp_f2f;// frame2frame icp
pcl::IterativeClosestPoint<PointType, PointType> icp_f2m;// frame2map icp
// bool flag
bool gps_flag = true;
bool is_first_icp = true;

pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr map_cloud_pcl(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr map_cloud_pcl_pass_through(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr lidar_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr lidar_cloud_last(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr lidar_cloud_current(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr lidar_cloud_current_icp_transformed(new pcl::PointCloud<PointType>);

std::queue<pcl::PointCloud<PointType>::Ptr> lidar_cloud_queue;

ros::Publisher part_map_pub, full_map_pub, scan_cloud_pub, car_pose_pub, f2f_icp_pub, f2m_icp_pub;
ros::Subscriber gps_sub, imu_sub, lidar_sub, tf_sub;

void frame2world_icp(pcl::PointCloud<PointType>::Ptr cloud_current, pcl::PointCloud<PointType>::Ptr cloud_pass_through)
{
    Eigen::Matrix4f rt_matrix;
    Eigen::Vector3f trans;

    pcl::PointCloud<PointType>::Ptr output_icp(new pcl::PointCloud<PointType>());
    sensor_msgs::PointCloud2 output;

    icp_f2m.setInputTarget(cloud_pass_through);
    icp_f2m.setInputSource(cloud_current);
    icp_f2m.setMaxCorrespondenceDistance(0.05);
    icp_f2m.setMaximumIterations(50);
    icp_f2m.setTransformationEpsilon(1e-5);
    icp_f2m.setEuclideanFitnessEpsilon(1);
    icp_f2m.align(*output_icp);

    rt_matrix = icp_f2m.getFinalTransformation();
    for(int i = 0; i< cloud_current->size(); i++)
    {
        trans << cloud_current->points[i].x , cloud_current->points[i].y , cloud_current->points[i].z;
        trans = rt_matrix.block<3,3>(0,0) * trans + rt_matrix.block<3,1>(0,3); 
        cloud_current->points[i].x = trans(0);
        cloud_current->points[i].y = trans(1);
        cloud_current->points[i].z = trans(2); 
    }
    pcl::toROSMsg(*cloud_current, output);
    output.header.frame_id = "/map";
    output.header.stamp = ros::Time::now();
    f2m_icp_pub.publish(output);
    ROS_WARN("TEST 01");
    
}

void frame2frame_icp(pcl::PointCloud<PointType>::Ptr cloud_last, pcl::PointCloud<PointType>::Ptr cloud_current)
{
    Eigen::Matrix4f rt_matrix;
    Eigen::Vector3f trans;

    pcl::PointCloud<PointType>::Ptr output_icp(new pcl::PointCloud<PointType>());
    sensor_msgs::PointCloud2 output;
    icp_f2f.setInputTarget(cloud_last);
    icp_f2f.setInputSource(cloud_current);
    icp_f2f.setMaxCorrespondenceDistance(0.05);  // Set the maximum distance threshold between two correspondent points in source
    icp_f2f.setMaximumIterations(50); 
    icp_f2f.setTransformationEpsilon(1e-5); // Set the transformation epsilon (maximum allowable translation squared difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
    icp_f2f.setEuclideanFitnessEpsilon(1);
    icp_f2f.align(*output_icp);

    rt_matrix = icp_f2f.getFinalTransformation();
    // update pose_guess
    orientation = rt_matrix.block<3,3>(0,0) * orientation.toRotationMatrix(); // calculate current frame car orientation
    initial_pose_guess = orientation.toRotationMatrix() * rt_matrix.block<3,1>(0,3) + initial_pose_guess; // update initial_pose_guess

    initial_pose_guess_ros.pose.position.x = initial_pose_guess(0);
    initial_pose_guess_ros.pose.position.y = initial_pose_guess(1);
    initial_pose_guess_ros.pose.position.z = initial_pose_guess(2);
    initial_pose_guess_ros.pose.orientation.w = orientation.w();
    initial_pose_guess_ros.pose.orientation.x = orientation.x();
    initial_pose_guess_ros.pose.orientation.y = orientation.y();
    initial_pose_guess_ros.pose.orientation.z = orientation.z();
    initial_pose_guess_ros.header.frame_id = "/map";
    initial_pose_guess_ros.header.stamp = ros::Time::now();
    car_pose_pub.publish(initial_pose_guess_ros);
    // update point_cloud_guess
    for (int i=0; i<cloud_current->size();i++)
    {
        trans << cloud_current->points[i].x, cloud_current->points[i].y, cloud_current->points[i].z;
        trans = orientation * trans + initial_pose_guess;
        cloud_current->points[i].x = trans(0);
        cloud_current->points[i].y = trans(1);
        cloud_current->points[i].z = trans(2);
    }
    lidar_cloud_current_icp_transformed = cloud_current; // this cloud has been rotated and transformed by frame to frame icp
    pcl::toROSMsg(*cloud_current, output);
    output.header.frame_id = "/map";
    output.header.stamp = ros::Time::now();
    f2f_icp_pub.publish(output);
    ROS_WARN("TEST 02");

}

void lf2wf_first_icp(Eigen::Vector3f pose_lf2wf, pcl::PointCloud<PointType>::Ptr map_lf2wf)  // lidar_frame to world_frame ICP
{
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity(); // just rotation matrix
    Eigen::Matrix4f rt_matrix = Eigen::Matrix4f::Identity(); // rotation and translation matrix

    pcl::PointCloud<PointType>::Ptr lidar_scan_cloud(new pcl::PointCloud<PointType>()); // scan_cloud in the lidar frame
    pcl::PointCloud<PointType>::Ptr scan_cloud_wf(new pcl::PointCloud<PointType>()); // scan_cloud transformed to world frame by gps, here we have not used icp;
    pcl::PointCloud<PointType>::Ptr icp_cloud(new pcl::PointCloud<PointType>());  // input_cloud and outputcloud_icp

    sensor_msgs::PointCloud2 output;

    int max_score = 99999; // define getfitness_score 
    int count = 1; // Mark the best rotation of initial guess

    lidar_scan_cloud = lidar_cloud_queue.front();
    scan_cloud_wf = lidar_scan_cloud; // initialize scan_cloud_wf make sure both size are the same for line 172~174

    // initial guess of rotation and translation from laser frame to gps frame
    for(int i = 0; i < 360 / Degree; i++)
    {
        rotation_matrix << cos(i * Degree / 180 * M_PI), -sin(i * Degree / 180 * M_PI),  0,
                            sin(i * Degree) / 180 * M_PI,  cos(i * Degree / 180 * M_PI), 0,
                            0,                             0,                            1;

        
        for (int j = 0; j < lidar_scan_cloud->points.size();j++)
        {
            // define pose of point cloud in lidar frame
            Eigen::Vector3f pose_pc_lf;
            // define pose of point cloud in world frame
            Eigen::Vector3f pose_pc_wf;
            pose_pc_lf << lidar_scan_cloud->points[j].x, lidar_scan_cloud->points[j].y, lidar_scan_cloud->points[j].z; 
            pose_pc_wf =  rotation_matrix * pose_pc_lf + initial_pose_guess;
            scan_cloud_wf->points[j].x = pose_pc_wf[0];
            scan_cloud_wf->points[j].y = pose_pc_wf[1];
            scan_cloud_wf->points[j].z = pose_pc_wf[2];
        }
        icp.setInputTarget(map_cloud_pcl_pass_through);
        icp.setInputSource(scan_cloud_wf);
        icp.setMaxCorrespondenceDistance(0.05);  // Set the maximum distance threshold between two correspondent points in source
        icp.setMaximumIterations(50); 
        icp.setTransformationEpsilon(1e-5); // Set the transformation epsilon (maximum allowable translation squared difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.
        icp.setEuclideanFitnessEpsilon(1); //  sum of the differences between correspondences in an Euclidean sense
        icp.align(*icp_cloud);
        if(icp.getFitnessScore() < max_score)
        {  
            max_score = icp.getFitnessScore();
            count = i;
            rt_matrix = icp.getFinalTransformation();            
        }
    }

    rotation_matrix << cos(count * Degree / 180 * M_PI), -sin(count * Degree / 180 * M_PI), 0,
                        sin(count * Degree) / 180 * M_PI,  cos(count * Degree / 180 * M_PI), 0,
                        0,                             0,                                    1;

    for (int j = 0; j < scan_cloud_wf->points.size(); j++)
    {
        Eigen::Vector3f trans;
        trans << lidar_scan_cloud->points[j].x, lidar_scan_cloud->points[j].y, lidar_scan_cloud->points[j].z; 
        trans = rotation_matrix * trans + initial_pose_guess;
        trans = rt_matrix.block<3,3>(0,0) * trans;

        icp_cloud->points[j].x =  trans[0] + rt_matrix(0,3);
        icp_cloud->points[j].y =  trans[1] + rt_matrix(1,3);
        icp_cloud->points[j].z =  trans[2] + rt_matrix(2,3);
    }
    pcl::toROSMsg(*icp_cloud, output);
    output.header.frame_id = "/map";
    output.header.stamp = ros::Time::now();
    scan_cloud_pub.publish(output);
                        
    orientation = rotation_matrix * rt_matrix.block<3,3>(0,0);
    initial_pose_guess_ros.pose.position.x = initial_pose_guess(0);
    initial_pose_guess_ros.pose.position.y = initial_pose_guess(1);
    initial_pose_guess_ros.pose.position.z = initial_pose_guess(2);
    initial_pose_guess_ros.pose.orientation.w = orientation.w();
    initial_pose_guess_ros.pose.orientation.x = orientation.x();
    initial_pose_guess_ros.pose.orientation.y = orientation.y();
    initial_pose_guess_ros.pose.orientation.z = orientation.z();
    initial_pose_guess_ros.header.frame_id = "/map";
    initial_pose_guess_ros.header.stamp = ros::Time::now();
    car_pose_pub.publish(initial_pose_guess_ros);

    ROS_WARN("TEST 03");


}

void show_map(pcl::PointCloud<PointType>::Ptr map)
{
    // define sensor_msg for publishing
    sensor_msgs::PointCloud2 output_of_voxelgrid, output_of_passthrough;

    // voxel grid filter in order to down sample
    bool voxel_grid_flag = true;
    if(voxel_grid_flag)
    {
        voxel_grid.setInputCloud(map);
        voxel_grid.setLeafSize(1.0, 1.0, 1.0);
        voxel_grid.filter(*map_cloud_pcl);
        voxel_grid_flag = false; // for the map_cloud_pcl, we only need to down_sample for the first time;
    } 

    // pass through filter in order to down_sample
    pass_through.setInputCloud(map_cloud_pcl);
    pass_through.setFilterFieldName("x");
    pass_through.setFilterLimits(initial_pose_guess(0) - 50, initial_pose_guess(0) + 50);
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(initial_pose_guess(1) - 50, initial_pose_guess(1) + 50);
    pass_through.filter(*map_cloud_pcl_pass_through);

    // publish map_voxel_grid and map_passthrough
    pcl::toROSMsg(*map_cloud_pcl, output_of_voxelgrid);
    output_of_voxelgrid.header.stamp = ros::Time::now();
    output_of_voxelgrid.header.frame_id = "/map";
    full_map_pub.publish(output_of_voxelgrid);
    // part_map_pub.publish(output_of_voxelgrid);

    pcl::toROSMsg(*map_cloud_pcl_pass_through, output_of_passthrough);
    output_of_passthrough.header.stamp = ros::Time::now();
    output_of_passthrough.header.frame_id = "/map";
    part_map_pub.publish(output_of_passthrough);
    // full_map_pub.publish(output_of_passthrough);
    ROS_WARN("TEST 04");


}

void gps_call_back(geometry_msgs::PointStamped::ConstPtr gps_msgs)
{
    if(gps_flag == true && ((gps_msgs->point.x * gps_msgs->point.x) + (gps_msgs->point.y * gps_msgs->point.y) + (gps_msgs->point.z * gps_msgs->point.z))!=0 )
    {
        initial_pose_guess(0) = gps_msgs->point.x;
        initial_pose_guess(1) = gps_msgs->point.y;
        initial_pose_guess(2) = gps_msgs->point.z;
        gps_flag = false;
    }
    ROS_WARN("TEST 05");
}

void lidar_call_back(sensor_msgs::PointCloud2::ConstPtr lidar_msgs)
{
    pcl::fromROSMsg(*lidar_msgs, *lidar_cloud);
    lidar_cloud_queue.push(lidar_cloud);
    ROS_WARN("TEST 06");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdc");
    ros::NodeHandle n;
   

    gps_sub = n.subscribe<geometry_msgs::PointStamped>("/fix", 500, gps_call_back); 
    lidar_sub = n.subscribe<sensor_msgs::PointCloud2>("/lidar_points", 500, lidar_call_back);

    part_map_pub = n.advertise<sensor_msgs::PointCloud2>("part_map_pub", 10);
    full_map_pub = n.advertise<sensor_msgs::PointCloud2>("full_map_pub", 10);
    scan_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("scan_cloud_pub", 10);
    car_pose_pub = n.advertise<geometry_msgs::PoseStamped>("car_pose_pub", 10);
    f2f_icp_pub = n.advertise<sensor_msgs::PointCloud2>("f2f_icp_pub", 10);
    f2m_icp_pub = n.advertise<sensor_msgs::PointCloud2>("f2m_icp_pub", 10); 


    sensor_msgs::PointCloud2 map_cloud_ros;

    pcl::io::loadPCDFile("/home/tangxiangshi/Desktop/ITRI/map.pcd", map_cloud_ros);

    pcl::fromROSMsg(map_cloud_ros, *map_cloud_pcl);

    ros::Rate loop_rate(10);

    while(ros::ok)
    {
        show_map(map_cloud_pcl);
        ROS_WARN("TEST 001 ");
        if(is_first_icp == true && !lidar_cloud_queue.empty()) // first icp with gps as initial guess
        {
            lf2wf_first_icp(initial_pose_guess, map_cloud_pcl_pass_through);
            is_first_icp = false;
            ROS_WARN("TEST 002");
        }
        if(is_first_icp == false && lidar_cloud_queue.size() > 1) // lidar_cloud_queue.size() > 1 to make sure that lidar_cloud_queue restore more than 2 frame point cloud
        {
            // this method is better because lidar_cloud_queue.size() may be larger than 2, such as 3,4,5 this method is better than lidar_cloud_queue_front() and lidar_cloud_queue.back(), because queue.size() may not be 2;
            lidar_cloud_last = lidar_cloud_queue.front();
            lidar_cloud_queue.pop(); // in order to make sure next lidar_cloud_queue.front() is the current_cloud;
            lidar_cloud_current = lidar_cloud_queue.front();

            frame2frame_icp(lidar_cloud_last, lidar_cloud_current);// in order to get translation and rotation between 2 frames; so as we can get the pose_of_car and update initial_pose_guess
            frame2world_icp(lidar_cloud_current_icp_transformed, map_cloud_pcl_pass_through); // in order to get relocallization;
            ROS_WARN("TEST 003");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
     
}