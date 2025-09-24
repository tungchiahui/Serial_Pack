#include "startup_main.h"
#include "usart.h"

extern uint8_t rx_buffer[1];

void startup_main(void)
{
    //�������ڽ����ж�
    HAL_UART_Receive_IT(&huart2,rx_buffer,1);

    #if isRTOS==0    	//������������
    for(;;)  //��ͬ��while(true)
    {

    }
    #endif
}
