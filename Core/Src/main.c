#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor.h"

int main() {
    HAL_Init();
    motor_config();
    
    while(1) {
			HAL_Delay(500);	//??500ms+
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//??PC13??
			motor_set_speed(1.0, 1.0);
    }
}
