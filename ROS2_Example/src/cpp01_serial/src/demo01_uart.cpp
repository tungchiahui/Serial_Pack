#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include <string>

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
        9600,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);
    
    // 初始化串口
    try
    {
      io_context_ = std::make_shared<drivers::common::IoContext>(1);
      // 初始化 serial_driver_
      serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
      serial_driver_->init_port(device_name, config);
      serial_driver_->port()->open();
      
      RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
      RCLCPP_INFO(this->get_logger(), "Using device: %s", serial_driver_->port().get()->device_name().c_str());
      RCLCPP_INFO(this->get_logger(), "Baud_rate: %d", config.get_baud_rate());
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
      return;
    }

        // 设置发送定时器
    transmit_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5000),
        std::bind(&Serial_Node::send_message_timer_callback, this));

    async_receive_message();   //进入异步接收
  }


private:

  void send_message_timer_callback()
  {
    // 发送一串字符串
    const std::string transmitted_message = "Hello from ROS 2!";
    transmit_data_buffer = std::vector<uint8_t>(transmitted_message.begin(), transmitted_message.end());
    // std::vector<uint8_t> hex_data = {0x48, 0x65, 0x6C, 0x6C, 0x6F}; // "Hello" in ASCII
    auto port = serial_driver_->port();

    try
    {
      size_t bytes_transmit_size = port->send(transmit_data_buffer);
      RCLCPP_INFO(this->get_logger(), "Transmitted: %s (%ld bytes)", transmitted_message.c_str(), bytes_transmit_size);
    }
    catch(const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port:%s",ex.what());
    }
  }

  void async_receive_message()  //创建一个函数更方便重新调用
  {
    auto port = serial_driver_->port();

    // 设置接收回调函数
    port->async_receive([this](const std::vector<uint8_t> &data,const size_t &size) 
    {
        if (size > 0)
        {
            // 处理接收到的数据
            std::string received_message(data.begin(), data.begin() + size);
            RCLCPP_INFO(this->get_logger(), "Received: %s (%ld bytes)", received_message.c_str(),size);
        }
        // 继续监听新的数据
        async_receive_message();
    }
    );
  }

  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::common::IoContext> io_context_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;
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