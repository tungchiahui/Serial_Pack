#include "ros/ros.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <cstddef>
#include <memory>
#include "cpp01_serial/struct_typedef.h"
#include "ros/wall_timer.h"

class SerialNode
{
public:
    SerialNode()
    {
        nh_ = std::make_shared<ros::NodeHandle>("~");
        ROS_INFO("串口节点开启!");

        std::string port = "/dev/ttyUSB0";  //设备名
        int32_t baud_rate = 9600;  //波特率
        int32_t character_size = 8;  // 数据位
        auto stop_bits = boost::asio::serial_port::stop_bits::one;  // 停止位
        auto parity = boost::asio::serial_port::parity::none;  //校验位
        auto flow_control = boost::asio::serial_port::flow_control::none;  //流控制

        //创建串口服务橘饼
        io_service_ = std::make_shared<boost::asio::io_service>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);
        read_buffer_ = std::make_shared<boost::array<uint8_t, 256>>();  // 接收缓冲区

        try
        {
            //设置设备名称
            serial_port_->open(port);
            //设置其他参数
            serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate));
            serial_port_->set_option(boost::asio::serial_port::character_size(character_size));
            serial_port_->set_option(boost::asio::serial_port::stop_bits(stop_bits));
            serial_port_->set_option(boost::asio::serial_port::parity(parity));
            serial_port_->set_option(boost::asio::serial_port::flow_control(flow_control));
            
            ROS_INFO("Serial port initialized successfully");
            ROS_INFO("Using device: %s",port.c_str()); // 使用传入的 port 变量作为设备名称
            ROS_INFO("Baud_rate: %d",baud_rate);
        }
        catch (const boost::system::system_error& e)
        {
            ROS_ERROR("配置失败: %s", e.what());
            ros::shutdown();
            return;
        }
        //定时器500ms发一次
        transmit_timer_ = nh_->createWallTimer(ros::WallDuration(0.5),
                                    &SerialNode::send_data_timer_callback, this);
        //异步接收
        asyncReceive();

        // 启动IO线程
        io_thread_ = std::make_shared<std::thread>(
        [this]()
        {
            ROS_INFO("IO服务线程启动");
            io_service_->run();
            ROS_INFO("IO服务线程退出");
        });
    }

    ~SerialNode()
    {
        transmit_timer_.stop(); // 停止定时器
    
        // 1. 取消所有异步操作
        if (serial_port_)
        {
            boost::system::error_code ec;
            serial_port_->cancel(ec); // 取消异步操作
            if (ec) // 检查错误码
            {
                ROS_ERROR("取消异步操作失败: %s", ec.message().c_str());
            }
        }

        // 2. 关闭串口
        if (serial_port_ && serial_port_->is_open())
        {
            boost::system::error_code ec;
            serial_port_->close(ec); // 关闭串口
            if (ec) 
            {
                ROS_ERROR("关闭串口失败: %s", ec.message().c_str());
            }
        }

        // 3. 停止IO服务
        if (io_service_)
            io_service_->stop();

        // 4. 等待线程退出
        if (io_thread_ && io_thread_->joinable())
        {
            io_thread_->join();
        }
    }

private:
    // ROS1 节点句柄
    std::shared_ptr<ros::NodeHandle> nh_;

    std::shared_ptr<boost::asio::io_service> io_service_;       // 基础服务
    std::shared_ptr<boost::asio::serial_port> serial_port_;     // 串口对象
    std::shared_ptr<std::thread> io_thread_;                    // IO服务线程

    std::shared_ptr<boost::array<uint8_t, 256>> read_buffer_;   // 接收缓冲区

    ros::WallTimer transmit_timer_;

    // 启动异步接收
    void asyncReceive()
    {
        if (!serial_port_->is_open())
        {
            ROS_ERROR("串口未打开，无法接收数据");
            return;
        }

        // 设置异步接收回调函数
        serial_port_->async_read_some(
            boost::asio::buffer(*read_buffer_),  // 使用共享缓冲区
            [this](const boost::system::error_code &ec, std::size_t size)
            {
                ROS_INFO("进入异步回调，状态码: %d", ec.value());
                if (ec) // 检查是否有错误
                {
                    ROS_ERROR("异步接收失败: %s", ec.message().c_str());
                    asyncReceive(); // 错误时重启接收
                    return;
                }

                // 处理接收到的数据
                if (size > 0)
                {
                    ROS_INFO("接收到数据大小: %zu", size);
                    for (std::size_t i = 0; i < size; ++i)
                    {
                        uint8_t data_byte = (*read_buffer_)[i];
                        ROS_INFO("接收字节: 0x%02X", data_byte);
                    }
                }

                // 继续监听新的数据
                asyncReceive();
            }
        );
    }




    void send_data_timer_callback(const ros::WallTimerEvent& event)
    {
        // 发送一串字符串
        const std::string transmitted_message = "Hello from ROS1 !";
        auto transmit_data_buffer = std::vector<uint8_t>(transmitted_message.begin(), transmitted_message.end());
        // std::vector<uint8_t> hex_data = {0x48, 0x65, 0x6C, 0x6C, 0x6F}; // "Hello" in ASCII

        if (!serial_port_->is_open())
        {
            ROS_ERROR("串口未打开，无法发送数据");
            return; // 或处理重连
        }

        this->asyncSend(transmit_data_buffer);
    }

    //阻塞式发送
    size_t syncSend(const std::vector<uint8_t>& data)
    {
        if(data.empty()) 
        { 
            // 空数据检查
            ROS_WARN("尝试发送空数据");
            return 0;
        }

        try
        {
            // 阻塞直到数据发送完成
            size_t bytes_transferred = boost::asio::write(
                *serial_port_,
                boost::asio::buffer(data)
            );
            return bytes_transferred; // 返回实际发送的字节数
        }
        catch (const boost::system::system_error& e) 
        {
            ROS_ERROR_STREAM("同步发送失败: " << e.what() 
                        << " [错误码: " << e.code() << "]");
            return 0; // 发送失败时返回0
        }
    }

    //异步发送
    void asyncSend(const std::vector<uint8_t>& data)
    {
        auto buf_ptr = std::make_shared<std::vector<uint8_t>>(data);
        boost::asio::async_write(
            *serial_port_,
            boost::asio::buffer(*buf_ptr),
            [this, buf_ptr](const boost::system::error_code& ec, size_t bytes)
            {
                if(ec)
                {
                    ROS_ERROR("异步发送失败: %s", ec.message().c_str());
                    return;
                }
                ROS_DEBUG("异步发送成功(%zu bytes)", bytes);
            }
        );
    }

};

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");  //设置编码
    ros::init(argc, argv, "SerialNode_node");  // 初始化节点并指定名称
    auto node = std::make_shared<SerialNode>();
    ros::spin();       // 主线程处理回调
    return 0;
}
