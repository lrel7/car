/**
@file       motor.c
@brief      实现与电机控制有关的函数
@author     Zev
*/

#include "motor.h"
#include "delay.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"

#define ENCODER_PERIOD 334                                      ///<未经过减速器的电机转动一圈产生的脉冲数
#define REDUCTION_RATIO 43                                      ///<电机的减速比
#define PULSE_PER_CIRCLE 21400                                  ///<理论上PULSE_PER_CIRCLE==ENCODER_PERIOD*REDUCTION_RATIO，但是因为没有电机的参考资料，所以没用ENCODER_PERIOD*REDUCTION_RATIO，PULSE_PER_CIRCLE是实际测量的
#define SPEED_TO_CIRCLE(v) (v / (6.4f * PI) * PULSE_PER_CIRCLE) ///<将速度转换为PID采样周期100ms要达到的脉冲数，速度单位为dm/s，车轮的直径为6.4cm

static void motor_rcc_config(void);
static void motor_gpio_config(void);
static void motor_tim_config(void);
static void motor_nvic_config(void);

/**
@brief      配置电机控制所需的时钟，GPIO口，定时器，中断
@param      None
@retval     None
*/
void motor_config(void)
{
    motor_rcc_config();
    motor_gpio_config();
    motor_tim_config();
    motor_nvic_config();
}
/**
@brief      设定左右电机的速度
@param      speed0 设定右电机的速度
            speed1 设定左电机的速度
@retval     None
@note       速度单位为dm/s，速度的正负可以表示方向
*/
void motor_set_speed(float speed0, float speed1)
{
    if (speed0 < 0)
    {
        motor_set_direction(MOTOR_DIRECTION_BACKWARD);
        speed0 = -speed0;
    }
    else
    {
        motor_set_direction(MOTOR_DIRECTION_FORWARD);
    }
    if (speed1 < 0)
    {
        motor_set_direction(MOTOR_DIRECTION_BACKWARD);
        speed1 = -speed1;
    }
    else
    {
        motor_set_direction(MOTOR_DIRECTION_FORWARD);
    }

    // pid_set_point(SPEED_TO_CIRCLE(speed0), SPEED_TO_CIRCLE(speed1)); //PID控制接受的参数是采样周期100ms内所要达到的脉冲数，因此需要对速度进行转换
}
/**
@brief      设定电机的转动方向
@param      dir 可选MOTOR_DIRECTION_FORWARD,MOTOR_DIRECTION_STOP,MOTOR_DIRECTION_BACKWARD,MOTOR_DIRECTION_KEEP
            motor 可选MOTOR_0，MOTOR_1或者两者的组合MOTOR_0|MOTOR_1
@retval     None
*/
void motor_set_direction(MOTOR_DIRECTION dir)
{
    uint16_t pin1 = GPIO_PIN_6, pin2 = GPIO_PIN_7, pin3 = GPIO_PIN_4, pin4 = GPIO_PIN_5;

    switch (dir)
    {
    case MOTOR_DIRECTION_STOP:
        // GPIO_ResetBits(GPIOA, pin0 | pin1);
        // 将四个引脚全部设为低电平
        HAL_GPIO_WritePin(GPIOA, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin4, GPIO_PIN_RESET);
        break;
    case MOTOR_DIRECTION_FORWARD:
        // GPIO_ResetBits(GPIOA, pin0);
        // GPIO_SetBits(GPIOA, pin1);
        // 将pin1, pin3设置为高电平，将pin2, pin4设置为低电平
        HAL_GPIO_WritePin(GPIOA, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, pin2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, pin4, GPIO_PIN_RESET);
        break;
    case MOTOR_DIRECTION_BACKWARD:
        // 将pin1, pin3设置为低电平，将pin2, pin4设置为高电平
        HAL_GPIO_WritePin(GPIOA, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, pin3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin4, GPIO_PIN_SET);
        break;
    case MOTOR_DIRECTION_LEFT:
        // 将pin1, pin4设置为高电平，将pin2, pin3设置为低电平
        HAL_GPIO_WritePin(GPIOA, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, pin2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin4, GPIO_PIN_SET);
        delay_ms(TURN_DELAY_TIME); // 延迟一定时间使它转到位
        break;
    case MOTOR_DIRECTION_RIGHT:
        // 将pin1, pin4设置为低电平，将pin2, pin3设置为高电平
        HAL_GPIO_WritePin(GPIOA, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, pin2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, pin3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, pin4, GPIO_PIN_RESET);
        delay_ms(TURN_DELAY_TIME); // 延迟一定时间使它转到位
        break;
    default:
        break;
    }
}
/**
@brief      获得单个电机的转动方向
@param      motor 可选MOTOR_0，MOTOR_1
@retval     返回电机motor的转动方向
*/
/*MOTOR_DIRECTION motor_get_direction(uint16_t motor)
{
    uint16_t pin0 = 0, pin1 = 0;

    if (motor & MOTOR_0)
    {
        pin0 = GPIO_Pin_6;
        pin1 = GPIO_Pin_7;
    }
    else if (motor & MOTOR_1)
    {
        pin0 = GPIO_Pin_4;
        pin1 = GPIO_Pin_5;
    }

    if (HAL_GPIO_ReadPin(GPIOA, pin0) == RESET && HAL_GPIO_ReadPin(GPIOA, pin1) == SET)
        return MOTOR_DIRECTION_FORWARD;
    else if (HAL_GPIO_ReadPin(GPIOA, pin0) == SET && HAL_GPIO_ReadPin(GPIOA, pin1) == RESET)
        return MOTOR_DIRECTION_BACKWARD;
    else
        return MOTOR_DIRECTION_STOP;
}*/
/**
@brief      配置电机控制所需的时钟
@param      None
@retval     None
*/

static void motor_rcc_config(void)
{/*
    // GPIO
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // TIM
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();*/
}


/**
@brief      配置电机控制所需的GPIO口
@param      None
@retval     None
@note    
*/

static void motor_gpio_config(void){
/*
    GPIO_InitTypeDef gpio_init;
    GPIO_StructInit(&gpio_init);

    //PA0--MSP1;PA1--MSP2
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.OutputType = GPIO_OUTPUT_TYPE_PP;
    gpio_init.Pin = GPIO_Pin_0 | GPIO_Pin_1; //PA0和PA1
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH; 
    gpio_init.Alternate = GPIO_AF_TIM5; // 设置复用功能为TIM5
    HAL_GPIO_Init(GPIOA, &gpio_init);
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); //必须开启复用功能
    // GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

    // PA6, PA7, PA4, PA5
    gpio_init.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5; // PA6, PA7, PA4, PA5
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP; // 设置为推挽输出模式
    gpio_init.Pull = GPIO_NOPULL; // 不使用上下拉电阻
    HAL_GPIO_Init(GPIOA, &gpio_init); // 初始化GPIOA引脚

    // PA8, PA9
    gpio_init.Pin = GPIO_PIN_8 | GPIO_PIN_9; // PA8, PA9
    gpio_init.Mode = GPIO_MODE_AF_PP; // 设置为复用功能推挽输出模式
    gpio_init.Pull = GPIO_NOPULL; // 不使用上下拉电阻
    gpio_init.Alternate = GPIO_AF_TIM1; // 设置复用功能为TIM1
    HAL_GPIO_Init(GPIOA, &gpio_init); // 初始化GPIOA引脚
    // GPIO_PinAFConfig(GPIOA, GPIO_PIN_8, GPIO_AF_TIM1); // 配置引脚复用功能为TIM1
    // GPIO_PinAFConfig(GPIOA, GPIO_PIN_9, GPIO_AF_TIM1); // 配置引脚复用功能为TIM1

    gpio_init.Pin = GPIO_PIN_6 | GPIO_PIN_7; // PC6, PC7
    gpio_init.Mode = GPIO_MODE_AF_PP; // 设置为复用功能推挽输出模式
    gpio_init.Pull = GPIO_NOPULL; // 不使用上下拉电阻
    gpio_init.Alternate = GPIO_AF_TIM8; // 设置复用功能为TIM8
    HAL_GPIO_Init(GPIOC, &gpio_init); // 初始化GPIOC引脚
    // GPIO_PinAFConfig(GPIOC, GPIO_PIN_6, GPIO_AF_TIM8); // 配置引脚复用功能为TIM8
    // GPIO_PinAFConfig(GPIOC, GPIO_PIN_7, GPIO_AF_TIM8); // 配置引脚复用功能为TIM8
*/
}
/**
@brief      配置电机控制所需的定时器
@param      None
@retval     None
@note       
*/

static void motor_tim_config(void)
{
/*
    TIM_TimeBaseInitTypeDef tim_base_init;
    TIM_OCInitTypeDef tim_oc_init;
    TIM_ICInitTypeDef tim_ic_init;

    //TIM5_OC1--PA0--MSP1;TIM5_OC2--PA1--MSP2
    TIM_DeInit(TIM5);
    tim_base_init.TIM_ClockDivision = 0;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base_init.TIM_Period = TIM5_PERIOD;
    tim_base_init.TIM_Prescaler = 0;
    tim_base_init.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM5, &tim_base_init);

    TIM_OCStructInit(&tim_oc_init);
    tim_oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init.TIM_Pulse = 0; //0~TIM5_PERIOD+1
    TIM_OC1Init(TIM5, &tim_oc_init);
    TIM_OC2Init(TIM5, &tim_oc_init);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable); //取消预加载，为了在motor_set_action函数中对CCR1的修改能够立即生效
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM5, ENABLE); //使能预加载，防止溢出

    TIM_Cmd(TIM5, ENABLE);
    //TIM1_TI1--PA8--MP11,TIM1_TI2--PA9--MP12;TIM8_TI1--PC6--MP21,TIM8_TI2--PC7--MP22
    TIM_DeInit(TIM1);
    TIM_DeInit(TIM8);
    tim_base_init.TIM_Period = TIM1_PERIOD;
    TIM_TimeBaseInit(TIM1, &tim_base_init);
    TIM_TimeBaseInit(TIM8, &tim_base_init);

    TIM_ICStructInit(&tim_ic_init);
    TIM_ICInit(TIM1, &tim_ic_init);
    TIM_ICInit(TIM8, &tim_ic_init);
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling); //将TIM8_TI2修改为TIM_ICPolarity_Falling为了使TIM1和TIM8计数方向相同

    tim_ic_init.TIM_Channel = TIM_Channel_3;
    tim_ic_init.TIM_ICSelection = TIM_ICSelection_TRC;
    TIM_ICInit(TIM1, &tim_ic_init);
    TIM_ICInit(TIM8, &tim_ic_init);
    TIM_SelectInputTrigger(TIM1, TIM_TS_ITR1);
    TIM_SelectInputTrigger(TIM8, TIM_TS_ITR1);

    TIM_Cmd(TIM1, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
    TIM_ClearFlag(TIM1, TIM_FLAG_Update | TIM_FLAG_CC3);
    TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_CC3, ENABLE);
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
*/
}

/*static void motor_tim_config(void)
{
    TIM_HandleTypeDef tim_handle;
    TIM_OC_InitTypeDef tim_oc_init;
    TIM_Encoder_InitTypeDef tim_encoder_init;

    //TIM5_OC1--PA0--MSP1;TIM5_OC2--PA1--MSP2
    HAL_TIM_Base_DeInit(&htim5);
    tim_handle.Instance = TIM5;
    tim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    tim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    tim_handle.Init.Period = TIM5_PERIOD;
    tim_handle.Init.Prescaler = 0;
    tim_handle.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&tim_handle);

    HAL_TIM_OC_Init(&tim_handle);
    tim_oc_init.OCMode = TIM_OCMODE_PWM1;
    tim_oc_init.Pulse = 0; //0~TIM5_PERIOD+1
    tim_oc_init.OCPolarity = TIM_OCPOLARITY_HIGH;
    tim_oc_init.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(&tim_handle, &tim_oc_init, TIM_CHANNEL_1);
    HAL_TIM_OC_ConfigChannel(&tim_handle, &tim_oc_init, TIM_CHANNEL_2);
    HAL_TIM_OC_PreloadConfig(&tim_handle, TIM_OCPreload_Disable, TIM_CHANNEL_1); //取消预加载，为了在motor_set_action函数中对CCR1的修改能够立即生效
    HAL_TIM_OC_PreloadConfig(&tim_handle, TIM_OCPreload_Disable, TIM_CHANNEL_2);
    HAL_TIM_Base_Start(&tim_handle);

    //TIM1_TI1--PA8--MP11,TIM1_TI2--PA9--MP12;TIM8_TI1--PC6--MP21,TIM8_TI2--PC7--MP22
    HAL_TIM_Base_DeInit(&htim1);
    HAL_TIM_Base_DeInit(&htim8);
    tim_handle.Instance = TIM1;
    tim_handle.Init.Period = TIM1_PERIOD;
    HAL_TIM_Base_Init(&tim_handle);
    HAL_TIM_Base_Start(&tim_handle);

    tim_handle.Instance = TIM8;
    HAL_TIM_Base_Init(&tim_handle);
    HAL_TIM_Base_Start(&tim_handle);

    HAL_TIM_Encoder_Init(&tim_handle, &tim_encoder_init);
    tim_encoder_init.EncoderMode = TIM_ENCODERMODE_TI1;
    tim_encoder_init.IC1Polarity = TIM_ICPOLARITY_RISING;
    tim_encoder_init.IC2Polarity = TIM_ICPOLARITY_RISING;
    HAL_TIM_Encoder_ConfigChannel(&tim_handle, &tim_encoder_init, TIM_CHANNEL_1);
    HAL_TIM_Encoder_ConfigChannel(&tim_handle, &tim_encoder_init, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&tim_handle, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&tim_handle, TIM_CHANNEL_2);

    HAL_TIM_Encoder_MspInit(&tim_handle);

    __HAL_TIM_CLEAR_FLAG(&tim_handle, TIM_FLAG_UPDATE | TIM_FLAG_CC3);
    HAL_TIM_Base_Start_IT(&tim_handle);
}*/


/**
@brief      配置电机控制所需的中断
@param      None
@retval     None
@note       
*/

/*
static void motor_nvic_config(void)
{
    NVIC_InitTypeDef nvic_init;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    nvic_init.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&nvic_init);
    nvic_init.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
    NVIC_Init(&nvic_init);

    nvic_init.NVIC_IRQChannel = TIM1_CC_IRQn;
    nvic_init.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&nvic_init);
}*/

static void motor_nvic_config(void)
{
    /*NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);*/
}
