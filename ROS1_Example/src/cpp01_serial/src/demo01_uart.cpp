#include "ros/console.h"
#include "ros/ros.h"
#include "serial/serial.h"
#include <atomic>
#include <clocale>
#include <cstdint>
#include <string>
#include <thread>
#include "cpp01_serial/serial_pack.h"


serial::Serial serial_driver_;
std::atomic<bool> stop_receive_(false);


extern std::vector<uint8_t> data_buffer;
void send_message_timer_callback(const ros::WallTimerEvent &)
{
    if (!serial_driver_.isOpen())
    {
        ROS_WARN("serial port is not open");
        return;
    }

    bool bool_buffer[] = {1, 0, 1, 0};
    int8_t int8_buffer[] = {0x11,0x22};
    int16_t int16_buffer[] = {2000,6666};
    int32_t int32_buffer[] = {305419896};
    fp32 fp32_buffer[] = {3.5f};

    const bool pack_ok = serial_pack_.tx.Data_Pack(
        0x01,
        bool_buffer, sizeof(bool_buffer) / sizeof(bool),
        int8_buffer, sizeof(int8_buffer) / sizeof(int8_t),
        int16_buffer, sizeof(int16_buffer) / sizeof(int16_t),
        int32_buffer, sizeof(int32_buffer) / sizeof(int32_t),
        fp32_buffer, sizeof(fp32_buffer) / sizeof(fp32)
    );

    if (!pack_ok) 
    {
      ROS_WARN("Data_Pack failed, skip sending");
      return;
    }

    if (data_buffer.empty()) 
    {
      ROS_WARN("No packed payload available, skip sending");
      return;
    }

    try
    {
        std::string buffer(reinterpret_cast<const char*>(data_buffer.data()), data_buffer.size());
        size_t bytes_size = serial_driver_.write(buffer);
        for (size_t i = 0; i < data_buffer.size(); ++i) 
        {
            ROS_INFO("0x%02X ", data_buffer[i]);  //以十六进制格式打印每个字节
        }
      ROS_INFO("(%zu bytes)", bytes_size);
    }
    catch(const std::exception &ex)
    {
      ROS_ERROR("Error Transmiting from serial port:%s",ex.what());
    }
}

void receive_task()
{
    while (ros::ok() && !stop_receive_.load())
    {
        try
        {
            std::string buffer = serial_driver_.read(1);
            if (!buffer.empty())
            {
                uint8_t rec_buffer = static_cast<uint8_t>(buffer[0]);
                const bool frame_ok = serial_pack_.rx.Data_Analysis(&rec_buffer, 0x01, 4, 2, 2, 0, 1);

                // 对接收到的数据进行处理（应用）
                if (frame_ok && serial_pack_.rx.data.cmd == 0x01) 
                {
                    ROS_INFO("Received integer: %d", serial_pack_.rx.data.int16_buffer[0]);
                }
            }
        }
        catch (const std::exception &e)
        {
            if (!stop_receive_.load())
            {
                ROS_ERROR("Error receiving data: %s", e.what());
            }
            break;
        }
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "serial_node_cpp");

    ros::NodeHandle nh;

    const std::string port_name = "/dev/ttyUSB0";
    const uint32_t baud_rate = 115200;

    ROS_INFO("Serial port Node Open!");

    try
    {
        serial_driver_.setPort(port_name);
        serial_driver_.setBaudrate(baud_rate);
        // timeout是设置超时时间，单位是毫秒
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_driver_.setTimeout(timeout);
        serial_driver_.open();

        if (!serial_driver_.isOpen())
        {
            ROS_ERROR("serial port open failed");
            return 1;
        }
        else
        {
            ROS_INFO("Serial port initialized successfully");
            ROS_INFO("Using device: %s",port_name.c_str());
            ROS_INFO("Baud_rate: %u",baud_rate);
        }

    }
    catch (const std::exception &e)
    {
        ROS_ERROR("open serial failed: %s", e.what());
        return 1;
    }

    auto transmit_timer_ = nh.createWallTimer(ros::WallDuration(1.0), send_message_timer_callback);
    auto receive_thread_ = std::thread(receive_task);

    ros::spin();

    stop_receive_.store(true);
    if (serial_driver_.isOpen())
    {
        serial_driver_.close();
    }

    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }

    return 0;
}
