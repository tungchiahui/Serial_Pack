#ifndef __SERIAL_PACK_H_
#define __SERIAL_PACK_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "struct_typedef.h"

/*接收数据数量*/
#define SERIAL_RX_BOOL_NUM 	0
#define SERIAL_RX_INT8_NUM 	4
#define SERIAL_RX_INT16_NUM 	0
#define SERIAL_RX_INT32_NUM 	0
#define SERIAL_RX_FP32_NUM 	0
#define SERIAL_RX_TOTAL_SIZE  (((SERIAL_RX_BOOL_NUM + 7) / 8) + \
                        (SERIAL_RX_INT8_NUM * 1) + \
                        (SERIAL_RX_INT16_NUM * 2) + \
                        (SERIAL_RX_INT32_NUM * 4) + \
                        (SERIAL_RX_FP32_NUM * 4))

/*发送数据数量*/
#define SERIAL_TX_BOOL_NUM      0
#define SERIAL_TX_INT8_NUM      0
#define SERIAL_TX_INT16_NUM     0
#define SERIAL_TX_INT32_NUM     0
#define SERIAL_TX_FP32_NUM      0
#define SERIAL_TX_TOTAL_SIZE    (((SERIAL_TX_BOOL_NUM + 7) / 8) + \
                            (SERIAL_TX_INT8_NUM * 1) + \
                            (SERIAL_TX_INT16_NUM * 2) + \
                            (SERIAL_TX_INT32_NUM * 4) + \
                            (SERIAL_TX_FP32_NUM * 4))


typedef struct
{
	int16_t ch[4];
	bool s[2];
}Udp_rc_t;


class SERIAL
{
	public:
	class RX
	{
		public:
		class DATA
		{
			public:
			uint8_t buffer[SERIAL_RX_TOTAL_SIZE + 7];
/****************************接受到的数据类型*******************************/
			bool bool_buffer[SERIAL_RX_BOOL_NUM];
			int8_t int8_buffer[SERIAL_RX_INT8_NUM];
			int16_t int16_buffer[SERIAL_RX_INT16_NUM];
			int32_t int32_buffer[SERIAL_RX_INT32_NUM];
			fp32 fp32_buffer[SERIAL_RX_FP32_NUM];
		}data;

		bool Data_Analysis(uint8_t *msg_data,int16_t bool_num = SERIAL_RX_BOOL_NUM,int16_t int8_num = SERIAL_RX_INT8_NUM,int16_t int16_num = SERIAL_RX_INT16_NUM,int16_t int32_num = SERIAL_RX_INT32_NUM,int16_t fp32_num = SERIAL_RX_FP32_NUM,int16_t total_size = SERIAL_RX_TOTAL_SIZE);
		bool Buffer_Sep(void);
		bool Data_Apply(void);
	}rx;

	class TX
	{
		public:
		class DATA
		{
			public:
			uint8_t buffer[SERIAL_TX_TOTAL_SIZE + 7];
/****************************准备发送的数据类型*******************************/
			bool bool_buffer[SERIAL_TX_BOOL_NUM];
			int8_t int8_buffer[SERIAL_TX_INT8_NUM];
			int16_t int16_buffer[SERIAL_TX_INT16_NUM];
			int32_t int32_buffer[SERIAL_TX_INT32_NUM];
			fp32 fp32_buffer[SERIAL_TX_FP32_NUM];
		}data;

	}tx;

	class CHECKSUM
	{
		public:
		uint8_t __SUMCRC(uint8_t *puchMsg, uint16_t usDataLen);
		uint16_t __CRC16_Check(uint8_t *puchMsg,uint16_t usDataLen);
	}checksum;

	class CONVERT
	{
		public:
		int16_t Bytes2Short(uint8_t DH,uint8_t DL);
		int32_t Bytes2Int(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL);
		fp32 Bytes2Fp32(uint8_t DH, uint8_t D2, uint8_t D3, uint8_t DL);
	}convert;

};

extern SERIAL serial_pack_;

#ifdef __cplusplus
}
#endif
	
#endif
