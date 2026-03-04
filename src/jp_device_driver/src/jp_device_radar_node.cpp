#include "sensor_msgs/PointCloud2.h"
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <std_msgs/UInt8.h>
#include <jp_device_driver/RadarStatus.h>
#include <jp_device_driver/RadarMsg.h>

// 帧信息宏定义
const uint16_t Frame_Header = 0xaa55;
// 帧头定义（网络字节顺序，即大端）
const uint8_t frame_header_bytes[] = {0xaa, 0x55}; // 直接定义字节序列
const uint16_t Minimum_Frame_Length = 11;

// 毫米波雷达信息帧结构体定义
struct RadarInfoFrame {
  uint16_t frame_header;    // 帧头，预设为0xaa55
  uint8_t frame_id;         // 帧ID
  uint16_t packet_length;   // 数据包长度
  uint8_t firmware_version; // 固件版本号
  uint16_t radar_status;    // 雷达状态码
  uint16_t target_count;    // 测量目标总数
  // 接下来是雷达的目标距离和信号强度的列表，这里只做简化展示
  // 其它目标数据根据实际需要进行添加
  std::vector<float> target_distances;
  std::vector<uint8_t> target_signal_strengths;
  uint8_t checksum; // 奇偶校验
};

// 数据缓存数组
unsigned char send_data[100];
unsigned char recv_data[2048];

// 初始化lidar发布者
ros::Publisher radar_pub;
// ros::Publisher radar_status_pub;
ros::Publisher radar_msg_pub;
ros::Timer checktimer;
ros::Time last_data_time;

jp_device_driver::RadarStatus RadarStatusMsg;
jp_device_driver::RadarMsg  RadarMsg;
// dump data
void printData(const uint8_t *data, size_t length) {
  // 检查data是否为空指针
  if (data == nullptr) {
    std::cout << "数据指针为空！" << std::endl;
    return;
  }

  for (size_t i = 0; i < length; ++i) {
    // 以十六进制格式打印数据，并确保每个输出有2个字符宽度
    // std::hex 用来设置十六进制输出模式
    // std::setfill('0') 和 std::setw(2) 用来保证即使是单个数字也会显示为两位数
    std::cout << std::hex << std::setfill('0') << std::setw(2)
              << static_cast<int>(data[i]);

    // 检查是否到达16字节边界，是的话打印换行
    if ((i + 1) % 16 == 0) {
      std::cout << std::endl;
    } else {
      std::cout << " "; // 否则打印空格，用作分隔
    }
  }

  // 如果总长度不是16的倍数，确保输出在最终结束后有一个换行符
  if (length % 16 != 0) {
    std::cout << std::endl;
  }
}
void printData(const std::vector<uint8_t> &data) {
  printData(data.data(), data.size());
}
void printData(const std::deque<uint8_t> &data) {
  printData(&data[0], data.size());
}
// 封装操作字节流的类
class ByteStream {
public:
  ByteStream(const uint8_t *data, size_t length)
      : mData(data), mLength(length) {}

  // 从字节流中读取特定类型的值
  template <typename T> T read(size_t offset) {
    if (offset + sizeof(T) > mLength) {
      throw std::runtime_error("Attempt to read beyond end of data buffer");
    }
    T value = 0;
    for (size_t i = 0; i < sizeof(T); ++i) {
      // value |= static_cast<T>(mData[offset + i]) << (i * 8);
      value = (value << 8) | static_cast<T>(mData[offset + i]);
    }
    // std::cout << "Hex: " << std::hex << value << std::endl;
    return value;
  }

  // 检查帧头是否匹配
  bool verifyFrameHeader(uint16_t header) {
    return read<uint16_t>(0) == header;
  }

  // 计算并验证校验和
  bool verifyChecksum(size_t length) {
    uint8_t sum = 0;
    for (size_t i = 3; i < length - 1; ++i) {
      sum += mData[i];
    }
    // std::cout << "Checksum: " << std::hex << sum << std::endl;
    return sum == mData[length - 1];
  }

private:
  const uint8_t *mData;
  size_t mLength;
};

enum parse_step_def {
  _ST_HEAD_1_PARSE = 0,
  _ST_HEAD_2_PARSE = 1,
  _LENGTH_PARSE = 2,
  _LENGTH_PARSE_L = 5,
  _FRAME_TYPE_PARSE = 3,
  _DATA_PARSE_CHECK = 4,
};

#define ST_HEAD_1 0xAA
#define ST_HEAD_2 0x55
class Interface_handler_Type_def {
public:
  Interface_handler_Type_def() {
    rec_cnt = 0;
    frame_type = 0;
    length = 0;
    parse_step = 0;
  }

  uint8_t length;
  uint8_t frame_type;
  uint8_t data_content[256];
  uint8_t sumcheck_rec;
  uint8_t rec_cnt;
  uint8_t parse_step;
  uint8_t sumcheck_cal, printf_cnt_over;

  uint8_t parse_data(uint8_t c_data);

private:
  uint8_t checksum(uint8_t *data, uint8_t len);
};

uint8_t Interface_handler_Type_def::checksum(uint8_t *data, uint8_t len) {
  uint8_t i = 0;
  uint8_t sum_check = data[0];
  while (i < len - 1) {
    sum_check += data[i + 1];
    i++;
  }
  return sum_check;
}
uint8_t Interface_handler_Type_def::parse_data(uint8_t c_data) {
  switch (parse_step) {
  case _ST_HEAD_1_PARSE:
    if (c_data == ST_HEAD_1) {
      parse_step = _ST_HEAD_2_PARSE;
      data_content[0] = ST_HEAD_1;
    }
    break;

  case _ST_HEAD_2_PARSE:

    if (c_data == ST_HEAD_2) {
      parse_step = _FRAME_TYPE_PARSE;
      data_content[1] = ST_HEAD_2;
    } else
      parse_step = _ST_HEAD_1_PARSE;
    break;

  case _FRAME_TYPE_PARSE:

    data_content[2] = frame_type = c_data;
    parse_step = _LENGTH_PARSE;

    break;

  case _LENGTH_PARSE:

    data_content[3] = c_data;
    parse_step = _LENGTH_PARSE_L;
    rec_cnt = 3;

    break;
  case _LENGTH_PARSE_L:

    data_content[4] = c_data;
    length = (data_content[3] << 8) + data_content[4];
    if (length < 6) {
      rec_cnt = 0;
      parse_step = _ST_HEAD_1_PARSE;
    } else {
      parse_step = _DATA_PARSE_CHECK;
      rec_cnt = 4;
    }

    break;

  case _DATA_PARSE_CHECK:
    rec_cnt++;
    data_content[rec_cnt] = c_data;

    if (rec_cnt == (length + 3)) {
      // printf(" radar len %d  \n ", length);
      parse_step = _ST_HEAD_1_PARSE;

      sumcheck_rec = data_content[length + 3];
      sumcheck_cal = checksum(&data_content[3], length);

      if (sumcheck_cal == sumcheck_rec) {
        return 1;
      } else {
        if (printf_cnt_over < 200) {
          printf_cnt_over++;
          ROS_INFO(
              "crc err frame_type = %02x crc rec = %02x crc calc = %02x \n",
              frame_type, sumcheck_rec, sumcheck_cal);
          for (int i = 0; i < length + 4; i++) {
            // 16进制的方式打印到屏幕
            std::cout << std::hex << (data_content[i] & 0xff) << " ";
          }
          std::cout << std::endl;
        } else if (printf_cnt_over == 200) {
          printf_cnt_over++;
          ROS_INFO("crc err frame_type = %02x crc rec = %02x crc calc = %02x "
                   "stop printf\n",
                   frame_type, sumcheck_rec, sumcheck_cal);
          for (int i = 0; i < length + 4; i++) {
            // 16进制的方式打印到屏幕
            std::cout << std::hex << (data_content[i] & 0xff) << " ";
          }
          std::cout << std::endl;
        }
      }
    }
    break;
  default:;
  }
  return 0;
}

void frame_handler(uint8_t frame_id, uint8_t *data, uint16_t len) {

  RadarInfoFrame frame;
  ByteStream stream(data, len);

  switch (frame_id) {
  case 1: {
    frame.firmware_version = stream.read<uint8_t>(5);
    frame.radar_status = stream.read<uint16_t>(6);
    frame.target_count = stream.read<uint16_t>(8);

    RadarStatusMsg.software_version = frame.firmware_version;
    RadarStatusMsg.status_code = frame.radar_status;
    // std::cout << "target_count:" << frame.target_count << std::endl;
    // 创建 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZI> radar_cloud;
    // 解析目标信息，根据目标数量解析每个目标的距离和信号强度
    size_t offset = 10;

    for (uint16_t i = 0; i < frame.target_count; ++i) {
      float target_distance = stream.read<uint32_t>(offset);

      frame.target_distances.push_back(target_distance);
      offset += sizeof(uint32_t);

      pcl::PointXYZI point;
      point.x = target_distance * 0.001;
      point.y = 0;
      point.z = 0;

      if (offset < static_cast<size_t>(frame.packet_length + 3)) {
        uint8_t target_signal_strength = stream.read<uint8_t>(offset);
        frame.target_signal_strengths.push_back(target_signal_strength);
        offset += sizeof(uint8_t);
        point.intensity = target_signal_strength;
        radar_cloud.points.push_back(point);
      } else {
        ROS_WARN("Insufficient data for all targets");
        break;
      }
    }

// 处理雷达信息...
// 这里可以是任何你需要执行的操作，例如，发布到ROS话题或打印输出等。
// 将结果填充到PointCloud2消息中
#if 0 // 用于测试
                if(radar_cloud.points.size() == 0){
                    std::cout << "Radar No Points! --------------------"<< std::endl;
                    return bytes_processed;
                }
                std::cout << "Radar Points Size:" << radar_cloud.points.size() << std::endl;
                // 在一行中打印所有距离
                std::cout << "Distances: ";
                for (auto distance : frame.target_distances) {
                    std::cout << distance*0.001 << "\t";
                }
                std::cout << std::endl;
                
                sensor_msgs::PointCloud2 cloud_msg;
                pcl::toROSMsg(radar_cloud, cloud_msg);
                cloud_msg.header.frame_id = "map";
                cloud_msg.header.stamp = ros::Time::now();
                radar_pub.publish(cloud_msg);
#else

    sensor_msgs::PointCloud2 cloud_msg;

    // 定义两个PointXYZI结构来保存最小点和最大点
    pcl::PointXYZI minPt, maxPt;

    // 获取点云中的最小点和最大点
    if (radar_cloud.points.size() != 0){
      pcl::getMinMax3D(radar_cloud, minPt, maxPt);

      // minPt 现在就含有了 x 最小的点的坐标
      std::cout << "Point with minimum x: (" << minPt.x << ", " << minPt.y << ", "
                << minPt.z << ", " << minPt.intensity << ")" << std::endl;

    } else {
      // 如果点云为空，清空 minPt 的坐标值并将其强度设为 0
      minPt.x = 0;
      minPt.y = 0;
      minPt.z = 0;
      minPt.intensity = 0;
      
      std::cout << "-----------------------" << std::endl;
    }
    RadarMsg.header.frame_id = "map";
    RadarMsg.header.stamp = ros::Time::now();
    RadarMsg.distance = minPt.x;
    RadarMsg.software_version = frame.firmware_version;
    if(frame.radar_status == 0x0808){
      RadarMsg.status_code = 0;
    }else{
      RadarMsg.status_code = frame.radar_status;
    }
    RadarMsg.intensity = minPt.intensity;
    radar_msg_pub.publish(RadarMsg);
    
    // 清除原始点云，并只添加最小点回去
    radar_cloud.clear();
    radar_cloud.points.push_back(minPt);
    pcl::toROSMsg(radar_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    radar_pub.publish(cloud_msg);
#endif
  } break;
  case 0x02:

    //命令回复
    ROS_INFO("Radar Command Reply Frame Parsed");
    break;
  default:

    //未知帧
    ROS_INFO("Unknown Frame Parsed");
    break;
  }
}
Interface_handler_Type_def serial_handler;

void timerCallback(const ros::TimerEvent&)
{
    bool isDisconnect = false;
    if ((ros::Time::now() - last_data_time).toSec() > 1.0)
    {
        ROS_WARN_STREAM("Radar data stream has stopped!");
        isDisconnect = true;  
    }else{
        isDisconnect = false;
    } 

    // RadarStatusMsg.isDisconnect = isDisconnect;
    // radar_status_pub.publish(RadarStatusMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "jp_device_driver_radar");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // 初始化ROS发布者
  radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_data", 5);
  checktimer = nh.createTimer(ros::Duration(0.5), timerCallback); // 每0.5秒检查一次
  // radar_status_pub = nh.advertise<jp_device_driver::RadarStatus>("/radar_status", 5);
  radar_msg_pub = nh.advertise<jp_device_driver::RadarMsg>("/radar_msg", 5);
  std::string serial_port;
  int baud_rate;
  double blind_area;
  bool hight_air_mode = false;
  nhPrivate.param<std::string>("radar_serial_port", serial_port,
                               "/dev/ttyUSB0");
  nhPrivate.param("radar_baud_rate", baud_rate, 57600);
  nhPrivate.param("blind_area", blind_area, 0.5);
  nhPrivate.param("hight_air_mode", hight_air_mode, false);

  ROS_INFO_STREAM("serial_port:" + serial_port +
                  " baud_rate:" + std::to_string(baud_rate));
  ROS_INFO_STREAM("hight_air_mode:" + std::to_string(hight_air_mode) +
                  " blind_area:" + std::to_string(blind_area));
  serial::Serial radar_serial;
  try {
    radar_serial.setPort(serial_port);
    radar_serial.setBaudrate(baud_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    radar_serial.setTimeout(to);
    radar_serial.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  // 设置为自动输出模式
  unsigned char radar_echo_mode[11] = {0xAA, 0x55, 0x03, 0x00, 0x07, 0x04,
                                       0x00, 0x00, 0x00, 0x01, 0x0C};
  radar_serial.write(radar_echo_mode, sizeof(radar_echo_mode));
  ros::Duration(1.0).sleep();

  // 设置飞行模式
  // if(hight_air_mode){
  //   unsigned char radar_air_mode[11] = {0xAA, 0x55, 0x03, 0x00, 0x07, 0x06, 0x00, 0x00, 0x00, 0x02, 0x0F}; //高空 
  //   radar_serial.write(radar_echo_mode,sizeof(radar_air_mode));
  // }else{
  //   unsigned char radar_ground_mode[11] = {0xAA, 0x55, 0x03, 0x00, 0x07, 0x06, 0x00, 0x00, 0x00, 0x01, 0x0E}; //低空
  //   radar_serial.write(radar_echo_mode,sizeof(radar_ground_mode));
  // }
  // ros::Duration(1.0).sleep();

  // 盲区设置
  // unsigned char radar_blind_dist[11] = {0xAA, 0x55, 0x03, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x52, 0xAF};    //盲区 
  // uint32_t blind_area_factor = blind_area/0.61; 
  // radar_blind_dist[9] = blind_area_factor&0xff;
  // radar_blind_dist[8] = (blind_area_factor>>8)&0xff;
  // radar_blind_dist[7] = (blind_area_factor>>16)&0xff;
  // radar_blind_dist[6] = (blind_area_factor>>24)&0xff;
  // // 计算累加 [3 ~9]
  // unsigned char checksum = 0;
  // for (int i = 3; i < 10; i++) {
  //   checksum += radar_blind_dist[i];
  // }
  // radar_blind_dist[10] = checksum;
  // radar_serial.write(radar_blind_dist, sizeof(radar_blind_dist));
  // ros::Duration(1.0).sleep();

  if (radar_serial.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }
  std::deque<uint8_t> circular_buffer;
  ros::Rate loop_rate(30); // 根据具体需求设置频率
  while (ros::ok()) {
    if (radar_serial.available()) {
      int len = radar_serial.available();
      std::vector<uint8_t> temp_buf(len);
      radar_serial.read(temp_buf, len);
      // printData(temp_buf);
      // 将接收到的数据添加到循环缓冲区
      std::copy(temp_buf.begin(), temp_buf.end(),
                std::back_inserter(circular_buffer));
      // 防止缓冲区过大，超过最大容量时移除前面的数据
      if (circular_buffer.size() > 2048) {
        circular_buffer.erase(circular_buffer.begin(),
                              circular_buffer.end() - 2048);
      }
      // 打印 circular_buffer
      // printData(circular_buffer);
      while (!circular_buffer.empty()) {

        uint8_t msg_update = serial_handler.parse_data(circular_buffer.front());
        circular_buffer.pop_front();
        if (msg_update) {
          last_data_time = ros::Time::now(); // 更新接收到数据的时间
          frame_handler(serial_handler.frame_type, serial_handler.data_content,
                        serial_handler.length + 3);
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
