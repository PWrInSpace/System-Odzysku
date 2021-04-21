#ifndef MOTOR_CTRL
#define MOTOR_CTRL

#include "gpio.h"
#include "tim.h"

#define MAX_PWM 1000

struct Motor {
	GPIO_TypeDef * input_1_port;
	uint16_t input_1_pin;

	GPIO_TypeDef * input_2_port;
	uint16_t input_2_pin;

	uint16_t speed_pwm;
};

void Motor_Init(struct Motor * motor,
		GPIO_TypeDef * in_1_port,
		uint16_t in_1_pin,
		GPIO_TypeDef * in_2_port,
		uint16_t in_2_pin,
		uint16_t pwm);
void Motor_Init_Default(struct Motor * motor,
		GPIO_TypeDef * in_1_port,
		uint16_t in_1_pin,
		GPIO_TypeDef * in_2_port,
		uint16_t in_2_pin);
void Motor_Open(struct Motor * motor);
void Motor_Close(struct Motor * motor);
void Motor_Stop(struct Motor * motor);

#endif
