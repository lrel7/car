#include "stm32f4xx_hal.h"
#include "delay.h"
// #include "usart.h"
#include "motor.h"

#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart1;

int main() {
    HAL_Init();
    
    // ???LED????????????
    Delay_Init();
    Motor_Init();
    //USART_init(9600);
	
		// ???UART1??
    //huart1.Instance = USART1;
    //huart1.Init.BaudRate = 9600;
    //huart1.Init.WordLength = UART_WORDLENGTH_8B;
    //huart1.Init.StopBits = UART_STOPBITS_1;
    //huart1.Init.Parity = UART_PARITY_NONE;
    //huart1.Init.Mode = UART_MODE_TX_RX;
    //huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    //huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    while(1) {
				Go();
			
        //uint8_t RX_Data;
        
        // ??????????
        //while(HAL_UART_Receive(&huart1, &RX_Data, 1, HAL_MAX_DELAY) != HAL_OK);
        
        // ???????????????
        //if(RX_Data == '1') {
            
        //} else if(RX_Data == '2') {
        //    Back();
        //} else if(RX_Data == '3') {
        //    Left();
        //} else if(RX_Data == '4') {
        //    Right();
        //} else {
        //    Stop();
        //}
    }
}