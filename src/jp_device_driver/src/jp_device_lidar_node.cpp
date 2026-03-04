#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iomanip> // 必须包含这个头文件来设置输出格式
#include <signal.h>
#include <jp_device_driver/LidarScanBundle.h>
#include "jp_device_driver/CustomMsg.h"
#include "jp_device_driver/CustomPoint.h"
#include "jp_device_driver/LidarStatus.h"
#include <termios.h>   // for keyboard input (调试模式)
#include <unistd.h>    // for keyboard input (调试模式)
#include <fcntl.h>     // for keyboard input (调试模式)
#include <iostream>
#include <sstream>
#include <ctime>
#include <cmath>
#include <limits>

//=== 安装方向
#define INSTALL_DIRECTION 1             // 1: 正常安装 , 2: 倒立安装 
#define IS_FILTER_LOW_REFLECTIVITY 0 // 是否过滤低反射率点云 0: 不过滤, 1: 过滤
#define LOW_REFLECTIVITY_THRESHOLD 130 // 低反射率阈值

//===================【角度补偿相关】====================
// 调试模式(1)时，可以使用键盘按键动态调整补偿角；使用模式(0)时，不需要键盘输入。
#define DEBUG_MODE 0
static float gPitchOffset = 0.0f; 
static float gYawOffset   = 0.0f;
static float gAngleStep   = 0.05f;  // 步进大小(单位：度)

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
//===================【键盘输入处理】==========================
// 非阻塞方式检测是否有键盘按下
bool kbhit() {
  termios oldt, newt;
  int ch;
  int oldf;
  
  // 获取当前终端设置
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  // 关闭回显与缓冲
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  // 还原终端设置
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF) {
    ungetc(ch, stdin);
    return true;
  }
  
  return false;
}
// 在调试模式下的键盘处理函数，您可自行修改按键映射、步进大小
//   - w/s → pitchOffset ± step  
//   - a/d → yawOffset   ± step  
//   - r   → step += 1  
//   - f   → step -= 1 (并做下限保护)  
void handleKeyboardInput() {
  static ros::Time lastPressTime(0);
  const double keyPressInterval = 0.2;
  if(!kbhit()) {
    return; // 没有按键按下则直接返回
  }
  ros::Time nowTime = ros::Time::now();
  // 若距离上一次有效按键还没超过 0.2 秒，就忽略本次按键
  if((nowTime - lastPressTime).toSec() < keyPressInterval) {
     // 也要把键从输入缓冲中读走避免下一帧再读它
     getchar(); 
     return ;
  }
  int c = getchar();
  bool changed = false;
  switch(c) {
    case 'w': // 俯仰角 +
      gPitchOffset += gAngleStep;
      std::cout << "[DEBUG] pitchOffset -> " << gPitchOffset << " deg" << std::endl;
      changed = true; 
      break;
    case 's': // 俯仰角 -
      gPitchOffset -= gAngleStep;
      std::cout << "[DEBUG] pitchOffset -> " << gPitchOffset << " deg" << std::endl;
      changed = true; 
      break;
    case 'a': // 水平角 -
      gYawOffset -= gAngleStep;
      std::cout << "[DEBUG] yawOffset -> " << gYawOffset << " deg" << std::endl;
      changed = true; 
      break;
    case 'd': // 水平角 +
      gYawOffset += gAngleStep;
      std::cout << "[DEBUG] yawOffset -> " << gYawOffset << " deg" << std::endl;
      changed = true; 
      break;
    case 'r': 
      gAngleStep += 0.01f; // 增加步进大小
      changed = true; 
      break;
    case 'f':
      gAngleStep -= 0.01f; // 减小步进大小
      // 做一个简单保护，避免等于或小于0
      if(gAngleStep < 0.01f) {
        gAngleStep = 0.01f;
      }
      changed = true; 
      break;
    default:
      // 其他按键不做处理
      break;
  }
  if(changed) {
      lastPressTime = nowTime;
      std::cout << "[DEBUG] pitchOffset=" << gPitchOffset 
                << "°, yawOffset=" << gYawOffset 
                << "°, angleStep=" << gAngleStep << "°."
                << std::endl;
  }

}
// 竖直方向的角度分辨率
const float pitchResolution = 0.0258f;
// 水平方向的角度分辨率
const float yawResolution = 0.0432;
// 距离分辨率
const float distanceResolution = 16.0f;

// 定义超时的时间（毫秒）
const int timeout_ms = 2500;

// 缓存包数据的最大数量
const int cache_packet_count = 100; // 10Hz

// 声明LidarData结构来保存解析后的雷达数据
struct LidarData
{
    float distance;
    float pitch;
    float yaw;
    uint8_t reflectivity;
};
//=== livox
 
uint64_t pack_start_time_offset = 0;

// 初始化Boost Asio
boost::asio::io_service io_service;
boost::asio::ip::udp::socket udp_socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 2368));
boost::asio::ip::udp::endpoint sender_endpoint;

pcl::PointCloud<pcl::PointXYZI>::Ptr
    laserSaveCloud(new pcl::PointCloud<pcl::PointXYZI>());
ros::Time last_receive_time;
// 初始化lidar发布者
ros::Publisher lidar_pub;
ros::Publisher livox_pub;
ros::Publisher livox_status_pub;
int counter = 0;
int count_xx = 0;
size_t clouds_received_ = 0;
jp_device_driver::LidarScanBundle bundle_;
jp_device_driver::CustomMsg livox_msg;
jp_device_driver::LidarStatus livox_status_msg;
bool sync_flag = false;
// 定义合理的时间范围（防止数据越界）
const double MIN_TIMESTAMP = 0; // Unix时间起点 1970-01-01 00:00:00 UTC
const double MAX_TIMESTAMP = std::numeric_limits<uint32_t>::max(); // 最大 32 位无符号整数秒数


 
std::string convertTimestampToHumanReadable(double timestamp) {
    ros::Time ros_time;
    ros_time.fromSec(timestamp);

    // 将 ROS 时间对象转换为 time_t
    std::time_t time_t_format = static_cast<std::time_t>(ros_time.toSec());

    // 使用标准库的 localtime 和 put_time 函数将 time_t 转换为可读格式并打印
    std::tm* ptm = std::localtime(&time_t_format);
    char buffer[32];
    // 格式化时间字符串
    std::strftime(buffer, 32, "%Y-%m-%d %H:%M:%S", ptm);

    // 计算毫秒部分
    int milliseconds = static_cast<int>((ros_time.toSec() - std::floor(ros_time.toSec())) * 1000);

    // 使用 stringstream 添加毫秒部分
    std::ostringstream oss;
    oss << buffer << "." << std::setfill('0') << std::setw(3) << milliseconds;
    
    return oss.str();
}

unsigned long long my_mktimea(const unsigned int year0, const unsigned int mon0,
       const unsigned int day, const unsigned int hour,
       const unsigned int min, const unsigned int sec)
{
    unsigned int mon = mon0, year = year0;
 
    /* 1..12 -> 11,12,1..10 */
    if (0 >= (int) (mon -= 2)) {
        mon += 12;  /* Puts Feb last since it has leap day */
        year -= 1;
    }
 
    // T = ((X * 24 + hour) * 60 + min) * 60 + sec
    // 先算出从1970年1月1日开始的天数X，再进而求出具体的时间值T的
    return ((((unsigned long long)
          (year/4 - year/100 + year/400 + 367*mon/12 + day) +
          year*365 - 719499
        )*24 + hour /* now have hours */
      )*60 + min /* now have minutes */
    )*60 + sec; /* finally seconds */
}

//=== 雷达状态上传
double timeout_threshold = 0.5; // 超时时间阈值（秒）
struct SystemStatusStruct {
    bool isDisconnect;
    bool isSyncTimeOK;
    bool isMotorOneOK;
    bool isMotorTwoOK;
    bool isEncoderOneOK;
    bool isEncoderTwoOK;
    bool isSysTempOK;
    bool isReceiverTempOK;
    bool isLaserTempOK;
    bool isLaserRayOK;
    bool isReceiverBiasVoltageOK;
};

SystemStatusStruct lidar_status;
void parseStatus(uint16_t data) {
    // lidar_status.isMotorOneOK           = (data & (1 << 0));  // 第0位，1号电机状态
    // lidar_status.isMotorTwoOK           = (data & (1 << 1));  // 第1位，2号电机状态
    // lidar_status.isEncoderOneOK         = (data & (1 << 2));  // 第2位，1号码盘状态
    // lidar_status.isEncoderTwoOK         = (data & (1 << 3));  // 第3位，2号码盘状态
    // lidar_status.isSysTempOK            = (data & (1 << 4));  // 第4位，系统温度状态
    // lidar_status.isReceiverTempOK       = (data & (1 << 5));  // 第5位，接收板温度状态
    // lidar_status.isLaserTempOK          = (data & (1 << 6));  // 第6位，激光器温度状态
    // lidar_status.isLaserRayOK           = (data & (1 << 7));  // 第7位，激光器出光状态
    // lidar_status.isReceiverBiasVoltageOK= (data & (1 << 8));  // 第8位，接收偏置电压状态/*
 
    /*
      factory 16bit:  |bit15 bit14 bit13 bit12|bit11...........bit0|
                       \_______ 属性 ________/ \_____   值   _______/
      属性(attribute): 0 => 故障码 (0 正常，1 故障)
                      1 => 俯仰角: 分辨率 0.01°，(N - 2048)*0.01°，上为正
                      2 => 航向角: 分辨率 0.01°，(N - 2048)*0.01°，左为正
                      3~11 => 预留
    */ 
    //int8_t  attribute = (data >> 12) & 0x0F;   // 高 4bit
   /// uint16_t value     =  data        & 0x0FFF; // 低 12bit

    
   // uint16_t a_data,b_data;
   // a_data = (data>>8 )& 0x00ff;
   // b_data = (data & 0x00FF)<< 8;
   // data = b_data + a_data;

    uint8_t  attribute = (data >> 12) & 0x0F;   // 高 4bit
    uint16_t value     =  data        & 0x0FFF; // 低 12bit

 // ROS_WARN("Factory Status: Unknown Fault Code = %d", data);
    switch (attribute)
    {
    case 0: // 故障码
    {
        // 如果协议仅规定 0=正常、1=故障，则可以直接判断
        if (value == 0)
        {
            // ROS_INFO("Factory Status: Fault Code = 0 (Normal)");
        }
        else if (value == 1)
        {
     //       ROS_WARN("Factory Status: Fault Code = 1 (Fault Detected) !");
        }
        else
        {
      //      ROS_WARN("Factory Status: Unknown Fault Code = %d", value);
        }
        break;
    }
    case 1: // 俯仰角 (分辨率 0.01°，上为正)
    {
        // 原始值范围 [0..4095]，需要先转成有符号值，再计算
        // (N - 2048)*0.01°, 上为正
        double pitch_deg = static_cast<int>(value) - 2048;
        pitch_deg *= 0.01;
        #if (DEBUG_MODE == 0)
        {
           gPitchOffset = pitch_deg; // 更新全局偏移量
        }
        #endif
       //  ROS_INFO("Factory Status: Pitch = %.2f°", gPitchOffset);
        break;
    }
    case 2: // 航向角 (分辨率 0.01°，左为正)
    {
        // 原始值范围 [0..4095]，需要先转成有符号值，再计算
        // (N - 2048)*0.01°, 上为正// 同俯仰角类似
        double heading_deg = static_cast<int>(value) - 2048;
        heading_deg *= 0.01;
        #if (DEBUG_MODE == 0)
        {
           gYawOffset = heading_deg; // 更新全局偏移量
        }
        #endif
       //ROS_INFO("Factory Status: YAW = %.2f°", gYawOffset);
        break;
    }
    default:
        // 3~11 => 预留
     //  ROS_INFO("Factory Status: Reserved attribute = %d, value = %d", attribute, value);
        break;
    }
}

Eigen::Affine3f getLidarRotationTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  float pitch_rad = gPitchOffset * float(M_PI) / 180.0f;
  float yaw_rad   = gYawOffset   * float(M_PI) / 180.0f;

  // 先绕Y轴旋转 pitch，再绕Z轴旋转 yaw（可按实际情况改动顺序或轴）
  transform.rotate(Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(yaw_rad,   Eigen::Vector3f::UnitZ()));
  return transform;
}
// 
// 处理UDP包的函数
void processUdpPacket(const std::vector<uint8_t> &packet)
{
    // 确认UDP包大小
    if (packet.size() < 1456)
    {
        ROS_WARN("Received incomplete UDP packet.");
        return;
    }
    // printf("%d size \n",packet.size());
    // 跳过42字节的Header和8字节的Timestamp来到达数据段(假设Header紧跟着UDP数据开始)
    size_t dataIndex = 0; // 42
    size_t timestampIndex = 0 + 1400;
    size_t factoryIndex = timestampIndex + 8;

    // 提取雷达数据
    std::vector<LidarData> lidarPoints;
    for (int i = 0; i < 200; ++i)
    {
        LidarData dataPoint;
        dataPoint.distance = ((packet[ 7*i] << 8) | packet[1 + 7*i]) * distanceResolution;
        dataPoint.pitch = ((packet[2  + 7*i ] << 8) | packet[3 + 7*i]) * pitchResolution;
        dataPoint.yaw = ((packet[ 4  + 7*i] << 8) | packet[5 + 7*i]) * yawResolution;
        dataPoint.reflectivity = packet[6 + 7*i];

        lidarPoints.push_back(dataPoint);
    }
    uint64_t ms_time = 0;
    uint64_t s_time = 0;
    double timestamp = 0;
   
    // printf("%d %d %d %d %d %d %d %d\n", packet[timestampIndex], packet[timestampIndex + 1], packet[timestampIndex + 2], packet[timestampIndex + 3], packet[timestampIndex + 4], packet[timestampIndex + 5], packet[timestampIndex + 6], packet[timestampIndex + 7]);
    // 提取时间戳信息
    if(packet[timestampIndex] & 0x80){
        uint16_t a = packet[timestampIndex + 6] << 8;
        ms_time = a  + packet[timestampIndex + 7];

        tm to_tm;
        to_tm.tm_year = int(packet[timestampIndex] & 0x7f) + 100;
        // printf("%d \n",packet[timestampIndex]&0x7f);
        to_tm.tm_mon = packet[timestampIndex + 1] - 1;
        
        to_tm.tm_mday = packet[timestampIndex + 2];
        to_tm.tm_hour = packet[timestampIndex + 3];
        to_tm.tm_min = packet[timestampIndex + 4];
        to_tm.tm_sec = packet[timestampIndex + 5];
        uint64_t aaa = my_mktimea(int(to_tm.tm_year + 1900),int(to_tm.tm_mon) + 1,int(to_tm.tm_mday),int(to_tm.tm_hour),int(to_tm.tm_min),int(to_tm.tm_sec));
        timestamp = aaa + ms_time*0.001 ;//- 8*3600;
        sync_flag = true;
    }else{
        uint16_t a = packet[timestampIndex + 6] << 8;
        ms_time = a  + packet[timestampIndex + 7];
        uint64_t cc = 0;
        uint64_t dd = 0;
        uint64_t ee = 0;
        uint64_t ff = 0;
        uint64_t gg = 0;
        uint64_t hh = 0;
        cc = packet[timestampIndex + 5];
        dd= packet[timestampIndex + 4]<<8;
        ee  =packet[timestampIndex + 3]<<16;
        ff = packet[timestampIndex + 2]<<24;
        gg = packet[timestampIndex + 1]<<32;
        hh = packet[timestampIndex]&0x7f;
        // s_time = packet[timestampIndex + 0]<<40 + packet[timestampIndex + 1]<<32 + packet[timestampIndex + 2]<<24 + packet[timestampIndex + 3]<<16 + packet[timestampIndex + 4]<<8 +  (packet[timestampIndex + 5]);
        s_time = cc+dd+ee+ff+gg+hh;
        timestamp = s_time + ms_time ;
        sync_flag = false;
        // 检查timestamp是否在合理的范围内
        if (timestamp < MIN_TIMESTAMP || timestamp > MAX_TIMESTAMP) {
            // std::cerr << "Error: Timestamp " << timestamp << " is out of the valid range!" << std::endl;
            timestamp = ros::Time::now().toSec(); // 使用当前系统时间
        }
        // timestamp = ros::Time::now().toSec(); // 使用当前系统时间
    }

    lidar_status.isSyncTimeOK = !sync_flag;
    // 提取Timestamp和Factory数据
    uint16_t factory = (packet[factoryIndex + 1] << 8) | packet[factoryIndex];

    parseStatus(factory);
    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZI> lidar_cloud;

    // 将极坐标转换为笛卡尔坐标并填充PointCloud2消息
    for (size_t i = 0; i < lidarPoints.size(); ++i)
    {
        // 如果距离为0，跳过
        // if (lidarPoints[i].distance == 0)
        // {
        //     continue;
        // }
        //lidarPoints[i].reflectivity =(lidarPoints[i].reflectivity -110)*10;
        #if IS_FILTER_LOW_REFLECTIVITY==1
            if(lidarPoints[i].reflectivity > LOW_REFLECTIVITY_THRESHOLD){
                continue; // 过滤低反射率点云
            }
        #endif
        float distance = lidarPoints[i].distance / 1000.0f;
        // 加上补偿角(单位：度)，再转弧度
        float yaw   = lidarPoints[i].yaw   / 180.0f * M_PI;
        float pitch = lidarPoints[i].pitch / 180.0f * M_PI;

        float x = distance * cos(yaw) * cos(pitch);
        float y = distance * sin(yaw) * cos(pitch);
        float z = distance * sin(pitch);

        pcl::PointXYZI point;
        #if INSTALL_DIRECTION == 2  //- 倒立安装
        point.x = z;
        point.y = x;
        point.z = y;
        #elif INSTALL_DIRECTION == 1   //- 正常安装
        point.x = z;
        point.y = -x;
        point.z = -y;
        #endif

        point.intensity = lidarPoints[i].reflectivity;
        lidar_cloud.points.push_back(point);

        // 兼容 livox
        jp_device_driver::CustomPoint custom_point;
        #if INSTALL_DIRECTION == 2  //- 倒立安装
        custom_point.x = z;
        custom_point.y = x;
        custom_point.z = y;
        #elif INSTALL_DIRECTION == 1   //- 正常安装
        custom_point.x = z;
        custom_point.y = -x;
        custom_point.z = -y;
        #endif
        
        custom_point.reflectivity = lidarPoints[i].reflectivity;
        custom_point.tag = 0;
        custom_point.line = 0;
        custom_point.offset_time = i*5000 + clouds_received_*1e6 ;
        livox_msg.points.push_back(custom_point);
    }

    if(clouds_received_ == 0){
        /*-使用系统时间-*/
        // bundle_.header.stamp = ros::Time::now();

        /*-使用雷达时间-*/
        bundle_.header.stamp.fromSec(timestamp);
        livox_msg.header.stamp.fromSec(timestamp);
        livox_msg.timebase = timestamp*1e9;
        // pack_start_time_offset = 0;
        // 打印时间戳
        // std::string human_time = convertTimestampToHumanReadable(timestamp);
        
        /*-调试-*/
         std::string human_time = convertTimestampToHumanReadable(timestamp);
         std::cout << "Sync : "<< sync_flag <<" Human-readable time: "<<std::fixed << std::setprecision(6) << human_time << std::endl; 
        // std::cout << "Sync : "<< sync_flag  << " Timestamp: "<<std::fixed << std::setprecision(6)  << timestamp << std::endl;
        // 打印时间戳
        // ROS_INFO("clouds_received_ == 0 timestamp %.6f", timestamp);
    }
    // 打印了点云数量
    // printf("----------------%d\n",lidar_cloud.points.size());


    //将结果填充到 bundle_ 消息中
    if (clouds_received_ < cache_packet_count)
    {
        pcl::toROSMsg(lidar_cloud, bundle_.clouds[clouds_received_]);
        lidar_cloud.clear();  
        // bundle_.clouds[clouds_received_].header.stamp = ros::Time::now();
        bundle_.clouds[clouds_received_].header.stamp.fromSec(timestamp);
        bundle_.clouds[clouds_received_].header.frame_id = "lidar_frame";
        bundle_.clouds[clouds_received_].header.seq = counter++;     

        livox_msg.header.seq = counter;
        // pack_start_time_offset =  timestamp*1e9 - livox_msg.timebase;
        // livox_msg.header.frame_id = "livox_frame";
        // livox_msg.header.stamp.fromSec(timestamp);
        // 打印接收到的点云数量
        // ROS_INFO("Received a LidarScanBundle at %f", bundle_.clouds[clouds_received_].header.stamp.sec + bundle_.clouds[clouds_received_].header.stamp.nsec * 1e-9);
        clouds_received_++;
    }
    if (clouds_received_ >= cache_packet_count)
    {

        // 根据安装方向和标定角度，生成综合旋转矩阵
        Eigen::Affine3f rotate_tf = Eigen::Affine3f::Identity();

        //--------------- 先根据宏判断倒装还是正装 ---------------
        #if INSTALL_DIRECTION == 2
            // 倒装时：先加一个 180° 翻转 (举例：绕X轴)
            rotate_tf.rotate(Eigen::AngleAxisf(float(M_PI), Eigen::Vector3f::UnitX()));
        #endif

        // 再叠加您标定出的俯仰/偏航角 (无论正/倒装都用同一组)
        float pitch_rad = gPitchOffset * float(M_PI) / 180.0f;
        float yaw_rad   = gYawOffset   * float(M_PI) / 180.0f;
        rotate_tf.rotate(Eigen::AngleAxisf(pitch_rad, Eigen::Vector3f::UnitY()));
        rotate_tf.rotate(Eigen::AngleAxisf(yaw_rad,   Eigen::Vector3f::UnitZ()));

        // 1) 对 bundle_ 的每个云做一次性旋转补偿
        for (size_t i = 0; i < cache_packet_count; ++i)
        {
            pcl::PointCloud<pcl::PointXYZI> tmp;
            pcl::fromROSMsg(bundle_.clouds[i], tmp);

            pcl::transformPointCloud(tmp, tmp, rotate_tf);

            pcl::toROSMsg(tmp, bundle_.clouds[i]);
            // 发布时仍固定为 "lidar_frame" (前/左/上)
            bundle_.clouds[i].header.frame_id = "lidar_frame";
        }

        // 2) 对 livox_msg.points 做同样的旋转补偿
        for (auto &p : livox_msg.points)
        {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            pt = rotate_tf * pt;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
        }
        bundle_.software_version = std::to_string(static_cast<int>(factory));
        bundle_.lidar_temperature = 25;
        bundle_.lidar_rotation_speed = 600;
        lidar_pub.publish(bundle_);
        //=== 发布livox自定义数据 ===
        livox_msg.lidar_id = 1;
        livox_msg.point_num = livox_msg.points.size();
        livox_pub.publish(livox_msg);
        livox_msg.points.clear();
        // pack_start_time_offset = 0;

        #if (DEBUG_MODE == 0)
        ROS_INFO("livox_msg.point_num = %d\r\n",livox_msg.point_num);
        #endif
  
        clouds_received_ = 0;
        // 清空每个点云数据用于下一次循环
        for (auto& cloud : bundle_.clouds) {
            cloud.data.clear(); // 清除点云数据
            cloud.width = 0;
            cloud.height = 0;
            cloud.row_step = 0;
            cloud.point_step = 0;
            cloud.is_dense = false;
        }
    }

    // 更新最后接收时间
    last_receive_time = ros::Time::now();
}

// 回调函数，用于处理接收到的UDP数据
void handle_receive(const boost::system::error_code &error, std::size_t /*bytes_transferred*/, std::vector<uint8_t> *data)
{
    if (!error)
    {
        // 打印收到的 data 数据
        // for(auto value:*data){
        //     std::cout << static_cast<int>(value) << " ";
        // }
        // std::cout << std::endl;
        // int count = 0; // 初始计数器
        // // 我的猜测是：data是一个指向容器的指针，例如std::vector<unsigned char>* 或类似的
        // for(auto value : *data) {
        //     // 以16进制的形式输出value，并使用setw和setfill设置格式
        //     std::cout << std::setw(2) << std::setfill('0') << std::hex
        //             << (static_cast<unsigned char>(value) & 0xff) << " ";
        //     // 如果打印了16个元素，输出换行符
        //     if (++count % 16 == 0) {
        //         std::cout << std::endl;
        //     }
        // }
        // // 检查并补充最后的换行符，以防上面的数据不足16个元素
        // if (count % 16 != 0) {
        //     std::cout << std::endl;
        // }
        // 解析数据
        processUdpPacket(*data);

        // 重新发起异步接收
        udp_socket.async_receive_from(boost::asio::buffer(*data), sender_endpoint,
                                      boost::bind(&handle_receive,
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred,
                                                  data));
    }
    else
    {
        std::cerr << "Error receiving UDP data: " << error.message() << std::endl;
    }
}

// 定时器回调函数，用于检查是否超时
void timeoutCheckCallback(const ros::TimerEvent& event) {
    ros::Time current_time = ros::Time::now();
    // 计算从最后一次收到数据到现在的时间间隔
    double time_difference = (current_time - last_receive_time).toSec();

    if (time_difference > timeout_threshold) {
        lidar_status.isDisconnect = true;
        ROS_WARN("Laser scan data timeout! Last message was %.2f seconds ago.", time_difference);
    } else {
        lidar_status.isDisconnect = false;
    }
    
    livox_status_msg.header.stamp = ros::Time::now();
    livox_status_msg.header.frame_id = "livox_frame";
    livox_status_msg.ConnectState = lidar_status.isDisconnect;
    livox_status_msg.SyncTimeState = lidar_status.isSyncTimeOK;
    livox_status_msg.MotorOneState = lidar_status.isMotorOneOK;
    livox_status_msg.MotorTwoState = lidar_status.isMotorTwoOK;
    livox_status_msg.EncoderOneState = lidar_status.isEncoderOneOK;
    livox_status_msg.EncoderTwoState = lidar_status.isEncoderTwoOK;
    livox_status_msg.SysTempState = lidar_status.isSysTempOK;
    livox_status_msg.RecTempState = lidar_status.isReceiverTempOK;
    livox_status_msg.LaserTempState = lidar_status.isLaserTempOK;
    livox_status_msg.LaserRayState = lidar_status.isLaserRayOK;
    livox_status_msg.RecBiasVolState = lidar_status.isReceiverBiasVoltageOK;

    livox_status_pub.publish(livox_status_msg);
}

// 信号处理函数
void Sigint_handler(int signum)
{
    // 首先调用ros::shutdown()来通知ROS停止
    ros::shutdown();
    // 调用io_service.stop()来停止Boost Asio
    io_service.stop();
}
int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "jp_device_driver_lidar");
    ros::NodeHandle nh;

    signal(SIGINT, Sigint_handler);
    // 预先定义数据缓冲区
    std::vector<uint8_t> data(1500); // 大小自定义，但应足以容纳最大UDP包

    bundle_.clouds.resize(cache_packet_count); // 确保我们有足够的空间

    // 初始化ROS发布者
    livox_status_pub = nh.advertise<jp_device_driver::LidarStatus>("/livox_status", 5);
    lidar_pub = nh.advertise<jp_device_driver::LidarScanBundle>("/lidar_data", 5);
    livox_pub = nh.advertise<jp_device_driver::CustomMsg>("/livox_data", 5);

    // 异步接收数据
    udp_socket.async_receive_from(boost::asio::buffer(data), sender_endpoint,
                                  boost::bind(&handle_receive,
                                              boost::asio::placeholders::error,
                                              boost::asio::placeholders::bytes_transferred,
                                              &data));

    // 开始异步I/O事件循环
    boost::thread io_thread(boost::bind(&boost::asio::io_service::run, &io_service));

    // 现在这一行安全，因为它在NodeHandle创建之后
    last_receive_time = ros::Time::now();

    // 设置超时检查定时器
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timeoutCheckCallback);

    #if (DEBUG_MODE == 1)
        ROS_WARN("Debug mode is ON. Use 'w/s' to adjust pitch, 'a/d' to adjust yaw, 'r' to increase step, 'f' to decrease step.");
    #else
        ROS_INFO("Debug mode is OFF. No keyboard input handling.");
    #endif
    // ROS主循环
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        ros::spinOnce();
        #if DEBUG_MODE
        handleKeyboardInput();
        #endif
        loop_rate.sleep();
    }

    // Clean up
    io_thread.join();
    udp_socket.close();

    return 0;
}
