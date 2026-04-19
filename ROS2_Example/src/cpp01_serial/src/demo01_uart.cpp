#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>
#include "cpp01_serial/serial_pack.h"


extern std::vector<uint8_t> data_buffer;

class Serial_Node : public rclcpp::Node
{
public:
  Serial_Node() : Node("serial_node_cpp")
  {
    // 等设备准备好再初始化
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 串口设备名（根据实际设备调整）
    const std::string device_name = "/dev/ttyUSB0";

    RCLCPP_INFO(this->get_logger(), "Serial port Node Open!");

    // 创建串口配置对象
    // 波特率115200；不开启流控制；无奇偶效验；停止位1。
    drivers::serial_driver::SerialPortConfig config(
        115200,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);
    
    // 初始化串口
    try
    {
      // 创建IO
      io_context_ = std::make_shared<drivers::common::IoContext>(1);
      // 初始化 serial_driver_
      serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
      serial_driver_->init_port(device_name, config);
      serial_driver_->port()->open();
      if (!serial_driver_->port()->is_open())
      {
        RCLCPP_ERROR(this->get_logger(), "serial port open failed");
        return;
      }
      else 
      {
        RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
        RCLCPP_INFO(this->get_logger(), "Baud_rate: %u", static_cast<unsigned int>(config.get_baud_rate()));
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
      return;
    }

    // 设置发送定时器
    transmit_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10000),
        std::bind(&Serial_Node::send_message_timer_callback, this));

    // 启动接收线程
    receive_thread_ = std::thread(&Serial_Node::receive_task, this);

  }

  ~Serial_Node()
  {
    // 停止接收线程
    stop_receive_.store(true);

    // 关闭串口以中断阻塞的 receive()
    try
    {
      if (serial_driver_) 
      {
        auto port = serial_driver_->port();
        if (port && port->is_open()) 
        {
          port->close();
        }
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to close serial port during shutdown: %s", ex.what());
    }

    // 阻塞主线程，等待接收线程结束
    if (receive_thread_.joinable()) 
    {
      receive_thread_.join();
    }
  }


private:

  void send_message_timer_callback()
  {
    bool bool_buffer[] = {1, 0, 1, 0};
    int8_t int8_buffer[] = {0x11,0x22};
    int16_t int16_buffer[] = {2000,6666};
    int32_t int32_buffer[] = {305419896};
    fp32 fp32_buffer[] = {3.5f};

    //由于ROS2中node为局部变量，所以只能在node中调用send函数，所以Serial_Transmit只负责处理data_buffer。
    const bool pack_ok = serial_pack_.tx.Data_Pack(
        0x01,
        bool_buffer, sizeof(bool_buffer) / sizeof(bool),
        int8_buffer, sizeof(int8_buffer) / sizeof(int8_t),
        int16_buffer, sizeof(int16_buffer) / sizeof(int16_t),
        int32_buffer, sizeof(int32_buffer) / sizeof(int32_t),
        fp32_buffer, sizeof(fp32_buffer) / sizeof(fp32));
    if (!pack_ok) 
    {
      RCLCPP_WARN(this->get_logger(), "Data_Pack failed, skip sending");
      return;
    }

    if (data_buffer.empty()) 
    {
      RCLCPP_WARN(this->get_logger(), "No packed payload available, skip sending");
      return;
    }

    auto port = serial_driver_->port();

    try
    {
      size_t bytes_size = port->send(data_buffer);
      RCLCPP_INFO(this->get_logger(), "Transmitted: ");
      for (size_t i = 0; i < data_buffer.size(); ++i) 
      {
        RCLCPP_INFO(this->get_logger(), "0x%02X ", data_buffer[i]);  //以十六进制格式打印每个字节
      }
      RCLCPP_INFO(this->get_logger(), "(%zu bytes)", bytes_size);
    }
    catch(const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
    }
  }

  // 创建一个新线程进行阻塞接收
  void receive_task()
  {
    auto port = serial_driver_->port();
    std::vector<uint8_t> buffer(256);

    while (!stop_receive_.load())
    {
      try 
      {
        // 阻塞接收
        size_t bytes_received = port->receive(buffer);

        if (bytes_received > 0) 
        {
          // 按字节喂给状态机，避免丢弃同一批次里后续字节
          for (size_t i = 0; i < bytes_received; ++i)
          {
            const bool frame_ok = serial_pack_.rx.Data_Analysis(&buffer[i], 0x01, 4, 2, 2, 0, 1);

            // 对接收到的数据进行处理（应用）
            if (frame_ok && serial_pack_.rx.data.cmd == 0x01) 
            {
              RCLCPP_INFO(this->get_logger(), "Received integer: %d", serial_pack_.rx.data.int16_buffer[0]);
            }
          }
        }
      }
      catch (const std::exception &ex) 
      {
        if (stop_receive_.load()) 
        {
          break;
        }
        RCLCPP_ERROR(this->get_logger(), "Error receiving data: %s", ex.what());
      }
    }
  }

  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;
  std::thread receive_thread_; // 用于接收数据的线程
  std::atomic<bool> stop_receive_{false};  // 用于停止接收线程
  std::vector<uint8_t> transmit_data_buffer = std::vector<uint8_t>(1024); // 发送缓冲区
  std::vector<uint8_t> receive_data_buffer = std::vector<uint8_t>(1024);  // 接收缓冲区
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Serial_Node>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
