#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor.h"

#include "stm32f4xx_hal.h"

int main() {
    HAL_Init();
    Delay_Init();
    Motor_Init();
    
    while(1) {
      motor_set_direction(MOTOR_DIRECTION_FORWARD);
    }
}