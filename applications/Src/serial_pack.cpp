#include "serial_pack.h"
#include "usart.h"
#include <cstdint>
#include <string.h> // For memcpy
#include "cmsis_os.h"


//包类对象
SERIAL serial_pack_;

//接收到的数据
uint8_t rx_buffer[1];

//单片机串口发送数据的线程任务
extern "C"
void Transmit_task(void const * argument)
{

	
	for(;;)
	{
		osDelay(500);
	}
}

//单片机串口接收中断回调函数	
extern "C"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		serial_pack_.rx.Data_Analysis(rx_buffer);
	}
}

uint8_t buffer[SERIAL_RX_TOTAL_SIZE + 7];

bool SERIAL::RX::Data_Apply(void)
{
/*  可使用的数据
	this->data.bool_buffer[i];
	this->data.int8_buffer[i];
	this->data.int16_buffer[i];
	this->data.int32_buffer[i];
	this->data.fp32_buffer[i];
*/

	for(int16_t i = 0;i < (SERIAL_RX_TOTAL_SIZE + 7);i++)
	{
		buffer[i] = this->data.buffer[i];
	}
	
	return true;
}


bool SERIAL::RX::Data_Analysis(uint8_t *msg_data,int16_t bool_num,int16_t int8_num,int16_t int16_num,int16_t int32_num,int16_t fp32_num,int16_t total_size)
{
	static uint16_t CRC16_ = 0;
	static int16_t rx_cnt = 0;
	static uint8_t finded_flag = 0;
	static int16_t valid_data_len = 0;
	static uint8_t cmd = 0;

    // 检测包头 0xA55A
    if (finded_flag == 0 && *msg_data == 0xA5)  // 检测到第一个包头字节
    {
        this->data.buffer[rx_cnt++] = *msg_data;
        finded_flag = 1; // 标记检测到包头的第一个字节
        return false;
    }
	if (finded_flag == 1 && *msg_data == 0x5A)  // 检测到第二个包头字节
    {
        this->data.buffer[rx_cnt++] = *msg_data;
        finded_flag = 2; // 标记检测到完整包头
        return false;
    }

	// 如果包头已检测到，继续处理数据
    if (finded_flag == 2)
    {
        // 将接收到的字节存入缓冲区
        this->data.buffer[rx_cnt++] = *msg_data;

        // 在接收到长度字节后，解析有效数据长度
        if (rx_cnt == 3)  // 接收到了有效数据长度字节
        {
            valid_data_len = this->data.buffer[2];
        }

		// 如果数据接收完成（总长度 = 包头 + 数据长度 + CRC16 + 包尾）
        if (rx_cnt == (valid_data_len + 7))  // 2 字节包头 + 数据长度 + 1个功能码 + CRC16 + 包尾
        {
            finded_flag = 0;  // 重置包头标志
            rx_cnt = 0;       // 重置接收计数器

            //获取功能码
            cmd = this->data.buffer[3];
			// 提取接收到的 CRC16（倒数第三和倒数第二字节）
            //倒数第三个是高8位
            //倒数第二个是低8位
            uint16_t received_crc = (this->data.buffer[valid_data_len + 4] << 8) | this->data.buffer[valid_data_len + 5];

            // 计算 CRC16 校验码（从有效数据长度开始计算）
            CRC16_ = serial_pack_.checksum.__CRC16_Check(this->data.buffer + 4, valid_data_len);

            // 校验 CRC16 和包尾
            if (received_crc == CRC16_ && this->data.buffer[valid_data_len + 6] == 0xFF)  // 确认包尾是 0xFF
            {
                // 数据处理
                this->Data_Apply();  // 处理数据，赋予实际意义
                return true;
            }
		}
	}

	// 如果验证失败，返回 false
    return false;
}



bool SERIAL::RX::Buffer_Sep(void)
{
	// 假设 msg_data 指向接收到的数据缓冲区的起始位置。
    uint8_t *ptr = this->data.buffer;

	// 解析 bool
		for (int i = 0; i < SERIAL_RX_BOOL_NUM; ++i) 
		{
			this->data.bool_buffer[i] = (*ptr & (1 << (i % 8))) != 0;
			if (i % 8 == 7 || i == SERIAL_RX_BOOL_NUM - 1) 
				{
					++ptr; // 每8个布尔值或到最后一个布尔值时，移动到下一个字节
				}
		}


	// 解析 int8_t
    for (int i = 0; i < SERIAL_RX_INT8_NUM; ++i) 
	{
        this->data.int8_buffer[i] = static_cast<int8_t>(*ptr);
        ptr += sizeof(int8_t);
    }
    
    // 解析 int16_t
    for (int i = 0; i < SERIAL_RX_INT16_NUM; ++i) 
	{
		    uint8_t DL = *ptr++;
        uint8_t DH = *ptr++;
        this->data.int16_buffer[i] = serial_pack_.convert.Bytes2Short(DH, DL);
    }
    
    // 解析 int32_t
    for (int i = 0; i < SERIAL_RX_INT32_NUM; ++i) 
	{
        uint8_t DL = *ptr++;
        uint8_t D3 = *ptr++;
        uint8_t D2 = *ptr++;
        uint8_t DH = *ptr++;
        this->data.int32_buffer[i] = serial_pack_.convert.Bytes2Int(DH, D2, D3, DL);
    }
    
    // 解析 fp32
    for (int i = 0; i < SERIAL_RX_FP32_NUM; ++i) 
	{
        uint8_t DL = *ptr++;
        uint8_t D3 = *ptr++;
        uint8_t D2 = *ptr++;
        uint8_t DH = *ptr++;
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