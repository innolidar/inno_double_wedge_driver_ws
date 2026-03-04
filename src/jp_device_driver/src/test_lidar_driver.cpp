#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <jp_device_driver/LidarScanBundle.h>
#include <iomanip> // Include the <iomanip> library for setprecision
#include <chrono>

#define USE_VOXEL_GRID_FILTER
#define USE_OUTLIER_REMOVAL
// #define USE_CUT_OFF_PC
// #define TEST

class LidarScanSubscriber {
public:
    LidarScanSubscriber() {
        // 订阅 /lidar_data 主题
        lidar_scan_sub_ = nh_.subscribe("/lidar_data", 10, &LidarScanSubscriber::lidarScanCallback, this);
        // 发布点云 sensor_msgs::PointCloud2
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/jp_lidar_cloud", 10);
#ifdef TEST
        // Subscribe to /jp_lidar_cloud for voxel filtering
        voxel_filtered_cloud_sub_ = nh_.subscribe("/jp_lidar_cloud", 10, &LidarScanSubscriber::voxelFilterCallback, this);
        voxel_filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/jp_lidar_filtered_cloud", 10);
#endif
    }

    void lidarScanCallback(const jp_device_driver::LidarScanBundle::ConstPtr& scan_msg) {
        // 时间戳
        // ROS_INFO("Received a LidarScanBundle at %f", scan_msg->header.stamp.sec + scan_msg->header.stamp.nsec * 1e-9);
        // 解析 LidarScanBundle 消息
        // ROS_INFO("Received a LidarScanBundle with %lu scans.", scan_msg->clouds.size());
    
        // 解析其他信息
        // ROS_INFO("Software version: %s", scan_msg->software_version.c_str());
        // ROS_INFO("LIDAR Temperature: %.2f C", scan_msg->lidar_temperature);
        // ROS_INFO("LIDAR Rotation Speed: %.2f RPM", scan_msg->lidar_rotation_speed);

        // 创建新的点云对象
        pcl::PointCloud<pcl::PointXYZI> combined_cloud;
        ros::Time acquisition_time;
        // 解析每个扫描
        for (size_t i = 0; i < scan_msg->clouds.size(); ++i) { 
            // 计算点云的点数
            size_t total_points = scan_msg->clouds[i].width* scan_msg->clouds[i].height;
            if(total_points == 0){
                continue;
            }
            // 打印点云，点数
            // ROS_INFO("Scan %lu has %lu points.", i, total_points);
            // 打印点云的时间戳
            // ROS_INFO("Scan %lu has timestamp %f", i, scan_msg->clouds[i].header.stamp.toSec());
            // 将点云数据添加到新的点云对象中
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::fromROSMsg(scan_msg->clouds[i], cloud);
            cloud.header.frame_id = "lidar_link";
 
            // 获取 ROS 消息的时间戳
            acquisition_time = scan_msg->clouds[i].header.stamp;
            // 打印点云的时间戳
            // ROS_INFO("Scan %lu has timestamp %f", i, acquisition_time.toSec());
            // acquisition_time = scan_msg->clouds[i].header.stamp;
            combined_cloud += cloud;

        }

        // 将点云数据转换为 PointCloud2 格式
        sensor_msgs::PointCloud2 cloud2;
        // // 将点云X 和 Z 坐标互换
        // for (size_t j = 0; j < combined_cloud.points.size(); ++j) {
        //     float temp = combined_cloud.points[j].x;
        //     combined_cloud.points[j].x = combined_cloud.points[j].z;
        //     combined_cloud.points[j].z = temp;
        // }
        pcl::toROSMsg(combined_cloud, cloud2);
        static int  seq=0;
        // 增加点云信息
        cloud2.header.frame_id = "lidar_link";
        cloud2.header.seq = seq++;

        // cloud2.header.stamp = acquisition_time.toNSec() / 1000ull;
        // cloud2.header.stamp = ros::Time().now();
        cloud2.header.stamp = acquisition_time;
        // 打印时间戳
        ROS_INFO("cloud2 has timestamp %f", cloud2.header.stamp.toSec());
        

        //acquisition_time;
        // 打印点云的时间戳
        // ROS_INFO("cloud2 cloud has timestamp %f", acquisition_time.toSec());
        // 打印 cloud2.header.stamp
        // ROS_INFO("cloud2 has timestamp %f", cloud2.header.stamp);
        // 发布点云数据
        cloud_pub_.publish(cloud2);
    }


    void voxelFilterCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // 打印原始点云的数量
        ROS_INFO("Original cloud has %lu points.", cloud->points.size());

#ifdef USE_VOXEL_GRID_FILTER
//=== 体素滤波
        // Measure time for VoxelGrid filter
        auto start_voxel = std::chrono::steady_clock::now();

        // Apply VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.5f, 0.5f, 0.5f);  // You can adjust the leaf size

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        sor.filter(*cloud_filtered);

        auto end_voxel = std::chrono::steady_clock::now();
        auto duration_voxel = std::chrono::duration_cast<std::chrono::milliseconds>(end_voxel - start_voxel).count();
        ROS_INFO("VoxelGrid filter took %ld ms", duration_voxel);
        ROS_INFO("Number of points after VoxelGrid filter: %lu", cloud_filtered->points.size());

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>());
        // 帮我切掉 X 5 m 以内的点
        #ifdef USE_CUT_OFF_PC
        for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
            if (cloud_filtered->points[i].x > 5) {
                cloud_filtered2->points.push_back(cloud_filtered->points[i]);
            }
        }
        #else 
            cloud_filtered2 = cloud_filtered;
        #endif
#else
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 = cloud;
#endif

#ifdef USE_OUTLIER_REMOVAL
//=== 离群点去除
        // Measure time for Statistical Outlier Removal filter
        auto start_sor = std::chrono::steady_clock::now();

        // Apply initial filtering to cut off points with X > 30m
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cutoff(new pcl::PointCloud<pcl::PointXYZI>());
        for (size_t i = 0; i < cloud_filtered2->points.size(); ++i) {
            if (cloud_filtered2->points[i].x <= 30) {
                cloud_cutoff->points.push_back(cloud_filtered2->points[i]);
            }
        }

        // Apply Statistical Outlier Removal filter on the remaining points
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_outlier_removal;
        sor_outlier_removal.setInputCloud(cloud_cutoff);
        sor_outlier_removal.setMeanK(50);           // Set number of neighboring points to analyze for mean distance
        sor_outlier_removal.setStddevMulThresh(1); // Set standard deviation multiplier threshold
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final(new pcl::PointCloud<pcl::PointXYZI>());
        sor_outlier_removal.filter(*cloud_filtered_final);

        // Add back the points with X > 30m
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final2(new pcl::PointCloud<pcl::PointXYZI>());
        for (auto& point : cloud_filtered2->points) {
            if (point.x > 30) {
                cloud_filtered_final2->points.push_back(point);
            }
        }

        // Append the filtered points to the final cloud
        cloud_filtered_final2->points.insert(cloud_filtered_final2->points.end(), cloud_filtered_final->points.begin(), cloud_filtered_final->points.end());

        auto end_sor = std::chrono::steady_clock::now();
        auto duration_sor = std::chrono::duration_cast<std::chrono::milliseconds>(end_sor - start_sor).count();
        ROS_INFO("Statistical Outlier Removal filter took %ld ms", duration_sor);
        ROS_INFO("Number of points after Outlier Removal filter: %lu", cloud_filtered_final2->points.size());
#else
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_final2 = cloud_filtered2;
#endif

        sensor_msgs::PointCloud2 cloud_filtered_msg;
        pcl::toROSMsg(*cloud_filtered_final2, cloud_filtered_msg);
        cloud_filtered_msg.header = cloud_msg->header; // Preserve the original header

        voxel_filtered_cloud_pub_.publish(cloud_filtered_msg);
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_scan_sub_;
    ros::Publisher cloud_pub_;

    ros::Subscriber voxel_filtered_cloud_sub_;
    ros::Publisher voxel_filtered_cloud_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_scan_subscriber");
    LidarScanSubscriber subscriber;
    ros::spin();
    return 0;
}
