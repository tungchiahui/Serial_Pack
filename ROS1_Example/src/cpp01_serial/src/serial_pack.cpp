#include "cpp01_serial/serial_pack.h"
#include <cstdint>
#include <string.h> // For memcpy
#include <vector>



//包类对象
SERIAL serial_pack_;

//接收到的数据
uint8_t rx_buffer[1];


bool SERIAL::RX::Data_Apply(void)
{
/*  可使用的数据
	this->data.cmd;
	this->data.bool_buffer[i];
	this->data.int8_buffer[i];
	this->data.int16_buffer[i];
	this->data.int32_buffer[i];
	this->data.fp32_buffer[i];
*/
    //由于在ROS2中，node是局部变量，所以发布方只能在node类里，故这里不写任何东西，直接在接收回调函数里实现功能。
	
	return true;
}

std::vector<uint8_t> data_buffer;
bool SERIAL::TX::Serial_Transmit(uint8_t *pData, uint16_t Size)
{
    if (Size == 0)
    {
        data_buffer.clear();
        return true;
    }

    if (pData == nullptr)
    {
        return false;
    }

    // 每次发送都覆盖缓冲，避免历史帧不断累积。
    data_buffer.assign(pData, pData + Size);
    return true;
}


bool SERIAL::TX::Data_Pack(uint8_t cmd, 
                           bool *bool_buffer, int16_t bool_num,
                           int8_t *int8_buffer, int16_t int8_num,
                           int16_t *int16_buffer, int16_t int16_num,
                           int32_t *int32_buffer, int16_t int32_num,
                           fp32 *fp32_buffer, int16_t fp32_num)
{
    if (bool_num < 0 || int8_num < 0 || int16_num < 0 || int32_num < 0 || fp32_num < 0)
    {
        return false;
    }

    if ((bool_num > 0 && bool_buffer == nullptr) ||
        (int8_num > 0 && int8_buffer == nullptr) ||
        (int16_num > 0 && int16_buffer == nullptr) ||
        (int32_num > 0 && int32_buffer == nullptr) ||
        (fp32_num > 0 && fp32_buffer == nullptr))
    {
        return false;
    }

    const int16_t bool_byte_count = (bool_num + 7) / 8;
    const int32_t data_length = bool_byte_count +
                                int8_num +
                                (int16_num * 2) +
                                (int32_num * 4) +
                                (fp32_num * 4);

    // 长度字段只有 1 字节，且发送缓冲区固定大小。
    if (data_length > MAX_DATA_LENGTH || data_length > 0xFF)
    {
        return false;
    }

    constexpr int32_t frame_overhead = 7; // 2 字节包头 + 长度 + cmd + CRC16 + 包尾
    if ((data_length + frame_overhead) > static_cast<int32_t>(sizeof(this->data.buffer)))
    {
        return false;
    }

    uint8_t *ptr = this->data.buffer;

    // 包头
    *ptr++ = 0xA5;
    *ptr++ = 0x5A;

    // 有效值字节总长度
    *ptr++ = static_cast<uint8_t>(data_length);

    // 功能码
    *ptr++ = cmd;

    // 打包 bool 数据
    for (int i = 0; i < bool_byte_count; ++i) 
    {
        uint8_t byte = 0;
        for (int j = 0; j < 8; ++j) 
        {
            if (i * 8 + j < bool_num && bool_buffer[i * 8 + j]) 
            {
                byte |= (1 << j);
            }
        }
        *ptr++ = byte;
    }

    // 打包 int8_t 数据
    for (int i = 0; i < int8_num; ++i) 
    {
        *ptr++ = static_cast<uint8_t>(int8_buffer[i]);
    }

    // 打包 int16_t 数据
    for (int i = 0; i < int16_num; ++i) 
    {
        *ptr++ = static_cast<uint8_t>((int16_buffer[i] >> 8) & 0xFF);
        *ptr++ = static_cast<uint8_t>(int16_buffer[i] & 0xFF);
    }

    // 打包 int32_t 数据
    for (int i = 0; i < int32_num; ++i) 
    {
        *ptr++ = static_cast<uint8_t>((int32_buffer[i] >> 24) & 0xFF);
        *ptr++ = static_cast<uint8_t>((int32_buffer[i] >> 16) & 0xFF);
        *ptr++ = static_cast<uint8_t>((int32_buffer[i] >> 8) & 0xFF);
        *ptr++ = static_cast<uint8_t>(int32_buffer[i] & 0xFF);
    }

    // 打包 fp32 数据
    for (int i = 0; i < fp32_num; ++i) 
    {
        uint8_t *bytes = reinterpret_cast<uint8_t*>(&fp32_buffer[i]);
        *ptr++ = bytes[3];
        *ptr++ = bytes[2];
        *ptr++ = bytes[1];
        *ptr++ = bytes[0];
    }

    // 计算 CRC16 校验码
    //CRC16 只需要计算有效数据部分
    uint16_t crc = serial_pack_.checksum.__CRC16_Check(this->data.buffer + 4, ptr - this->data.buffer - 4);
    *ptr++ = static_cast<uint8_t>((crc >> 8) & 0xFF);
    *ptr++ = static_cast<uint8_t>(crc & 0xFF);

    // 包尾
    *ptr++ = 0xFF;

    // 计算总长度
    uint16_t total_length = ptr - this->data.buffer;

    // 发送数据
    return this->Serial_Transmit(this->data.buffer, total_length);
}




bool SERIAL::RX::Data_Analysis(uint8_t *msg_data,uint8_t cmd,int16_t bool_num,int16_t int8_num,int16_t int16_num,int16_t int32_num,int16_t fp32_num)
{
	static uint16_t CRC16_ = 0;
	static int16_t rx_cnt = 0;
	static uint8_t finded_flag = 0;
	static int16_t valid_data_len = 0;

    auto reset_state = [&]()
    {
        rx_cnt = 0;
        finded_flag = 0;
        valid_data_len = 0;
    };

    if (msg_data == nullptr)
    {
        reset_state();
        return false;
    }

    if (bool_num < 0 || int8_num < 0 || int16_num < 0 || int32_num < 0 || fp32_num < 0)
    {
        reset_state();
        return false;
    }

    const int16_t expected_data_len = ((bool_num + 7) / 8) +
                                      int8_num +
                                      (int16_num * 2) +
                                      (int32_num * 4) +
                                      (fp32_num * 4);

    constexpr int16_t frame_overhead = 7; // 2 字节包头 + 长度 + cmd + CRC16 + 包尾
    const int16_t max_frame_len = static_cast<int16_t>(sizeof(this->data.buffer));
    if (expected_data_len < 0 ||
        expected_data_len > MAX_SERIAL_RX_TOTAL_SIZE ||
        expected_data_len > 0xFF ||
        (expected_data_len + frame_overhead) > max_frame_len)
    {
        reset_state();
        return false;
    }

    const uint8_t current_byte = *msg_data;

    // 检测包头 0xA55A
    if (finded_flag == 0)
    {
        if (current_byte == 0xA5)
        {
            this->data.buffer[0] = current_byte;
            rx_cnt = 1;
            finded_flag = 1; // 标记检测到包头的第一个字节
        }
        return false;
    }

	if (finded_flag == 1)
    {
        if (current_byte == 0x5A)  // 检测到第二个包头字节
        {
            this->data.buffer[1] = current_byte;
            rx_cnt = 2;
            finded_flag = 2; // 标记检测到完整包头
        }
        else if (current_byte == 0xA5)
        {
            // 连续收到 0xA5 时，认为这是新的包头起点。
            this->data.buffer[0] = current_byte;
            rx_cnt = 1;
            finded_flag = 1;
        }
        else
        {
            reset_state();
        }
        return false;
    }

	// 如果包头已检测到，继续处理数据
    if (finded_flag == 2)
    {
        if (rx_cnt >= max_frame_len)
        {
            reset_state();
            return false;
        }

        // 将接收到的字节存入缓冲区
        this->data.buffer[rx_cnt++] = current_byte;

        // 在接收到长度字节后，解析有效数据长度
        if (rx_cnt == 3)  // 接收到了有效数据长度字节
        {
            valid_data_len = this->data.buffer[2];
            if (valid_data_len != expected_data_len ||
                (valid_data_len + frame_overhead) > max_frame_len)
            {
                reset_state();
                return false;
            }
        }

        if (rx_cnt > (valid_data_len + frame_overhead))
        {
            reset_state();
            return false;
        }

			// 如果数据接收完成（总长度 = 包头 + 数据长度 + CRC16 + 包尾）
        if (rx_cnt == (valid_data_len + frame_overhead))  // 2 字节包头 + 数据长度 + 1个功能码 + CRC16 + 包尾
        {
            //获取功能码
            this->data.cmd = this->data.buffer[3];
            if(cmd != this->data.cmd)
            {
                reset_state();
                return false;
            }

				// 提取接收到的 CRC16（倒数第三和倒数第二字节）
            //倒数第三个是高8位
            //倒数第二个是低8位
            const uint16_t received_crc = (static_cast<uint16_t>(this->data.buffer[valid_data_len + 4]) << 8) |
                                          static_cast<uint16_t>(this->data.buffer[valid_data_len + 5]);

            // 计算 CRC16 校验码（从有效数据长度开始计算）
            CRC16_ = serial_pack_.checksum.__CRC16_Check(this->data.buffer + 4, valid_data_len);

            // 校验 CRC16 和包尾
            const bool frame_ok = (received_crc == CRC16_) && (this->data.buffer[valid_data_len + 6] == 0xFF);
            if (frame_ok)  // 确认包尾是 0xFF
            {
                //数据分离
                const bool sep_ok = this->Buffer_Sep(bool_num,int8_num,int16_num,int32_num,fp32_num);
                // 数据处理
                const bool apply_ok = sep_ok && this->Data_Apply();  // 处理数据，赋予实际意义
                reset_state();
                return apply_ok;
            }

            reset_state();
            return false;
			}
		}

		// 当前包尚未接收完整，继续等待后续字节。
	    return false;

    reset_state();
	// 如果验证失败，返回 false
    return false;
}



bool SERIAL::RX::Buffer_Sep(int16_t bool_num,int16_t int8_num,int16_t int16_num,int16_t int32_num,int16_t fp32_num)
{
    if (bool_num < 0 || int8_num < 0 || int16_num < 0 || int32_num < 0 || fp32_num < 0)
    {
        return false;
    }

    if (bool_num > MAX_SERIAL_RX_BOOL_NUM ||
        int8_num > MAX_SERIAL_RX_INT8_NUM ||
        int16_num > MAX_SERIAL_RX_INT16_NUM ||
        int32_num > MAX_SERIAL_RX_INT32_NUM ||
        fp32_num > MAX_SERIAL_RX_FP32_NUM)
    {
        return false;
    }

    const int16_t expected_data_len = ((bool_num + 7) / 8) +
                                      int8_num +
                                      (int16_num * 2) +
                                      (int32_num * 4) +
                                      (fp32_num * 4);
    if (expected_data_len < 0 || expected_data_len > MAX_SERIAL_RX_TOTAL_SIZE)
    {
        return false;
    }

	// 假设 msg_data 指向接收到的数据缓冲区的有效数据值的起始位置。
    uint8_t *ptr = this->data.buffer + 4;

	// 解析 bool
    int bool_byte_count = (bool_num + 7) / 8; // 计算所需字节数（向上取整）
    for (int i = 0; i < bool_num; ++i) 
    {
        this->data.bool_buffer[i] = (ptr[i / 8] & (1 << (i % 8))) != 0;
    }
    ptr += bool_byte_count; // 移动指针到下一个数据段


	// 解析 int8_t
    for (int i = 0; i < int8_num; ++i) 
	{
        this->data.int8_buffer[i] = static_cast<int8_t>(*ptr);
        ptr += sizeof(int8_t);
    }
    
    // 解析 int16_t
    for (int i = 0; i < int16_num; ++i) 
	{
				uint8_t DH = *ptr++;
        uint8_t DL = *ptr++;
        this->data.int16_buffer[i] = serial_pack_.convert.Bytes2Short(DH, DL);
    }
    
    // 解析 int32_t
    for (int i = 0; i < int32_num; ++i) 
	{
        uint8_t DH = *ptr++;
        uint8_t D2 = *ptr++;
        uint8_t D3 = *ptr++;
        uint8_t DL = *ptr++;
        this->data.int32_buffer[i] = serial_pack_.convert.Bytes2Int(DH, D2, D3, DL);
    }
    
    // 解析 fp32
    for (int i = 0; i < fp32_num; ++i) 
	{
        uint8_t DH = *ptr++;
        uint8_t D2 = *ptr++;
        uint8_t D3 = *ptr++;
        uint8_t DL = *ptr++;
        this->data.fp32_buffer[i] = serial_pack_.convert.Bytes2Fp32(DH, D2, D3, DL);
    }

	return true;
}



uint8_t SERIAL::CHECKSUM::__SUMCRC(uint8_t *puchMsg, uint16_t usDataLen)
{
    int16_t i = 0;
	uint8_t uchSUMCRC = 0x00;
    for (; i < usDataLen; i++)
    {
			uchSUMCRC += puchMsg[i];
    }
    return uchSUMCRC;
}

uint16_t SERIAL::CHECKSUM::__CRC16_Check(uint8_t *puchMsg,uint16_t usDataLen)
{
    uint16_t uchCRC16 = 0xFFFF;
    uint8_t state,i,j;
    for(i = 0; i < usDataLen; i++ )
    {
        uchCRC16 ^= puchMsg[i];
        for( j = 0; j < 8; j++)
        {
            state = uchCRC16 & 0x01;
            uchCRC16 >>= 1;
            if(state)
            {
                uchCRC16 ^= 0xA001;
            }
        }
    }
    return uchCRC16;
}


int16_t SERIAL::CONVERT::Bytes2Short(uint8_t DH,uint8_t DL)
{
	int16_t result = (int16_t)((int16_t)DH << 8 | DL);
	return result;
}

int32_t SERIAL::CONVERT::Bytes2Int(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL)
{
    int32_t result = (int32_t)((int32_t)DH << 24 | (int32_t)D2 << 16 | (int32_t)D3 << 8 | DL);
    return result;
}

fp32 SERIAL::CONVERT::Bytes2Fp32(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL)
{
    uint8_t bytes[4] = {DL, D3, D2, DH};
    fp32 result;
    memcpy(&result, bytes, sizeof(result));
    return result;
}
