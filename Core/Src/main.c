#include "stm32f4xx_hal.h"
#include "delay.h"
#include "motor.h"

int main() {
    HAL_Init();
    motor_config();
    
    while(1) {
      motor_set_direction(MOTOR_DIRECTION_FORWARD);
    }
}
