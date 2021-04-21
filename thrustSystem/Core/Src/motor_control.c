#include "motor_control.h"

void Motor_Init(struct Motor * motor,
		GPIO_TypeDef * in_1_port,
		uint16_t in_1_pin,
		GPIO_TypeDef * in_2_port,
		uint16_t in_2_pin,
		uint16_t pwm) {
	motor->input_1_port = in_1_port;
	motor->input_1_pin = in_1_pin;
	motor->input_2_port = in_2_port;
	motor->input_2_pin = in_2_pin;
	motor->speed_pwm = pwm;
}

void Motor_Init_Default(struct Motor * motor,
		GPIO_TypeDef * in_1_port,
		uint16_t in_1_pin,
		GPIO_TypeDef * in_2_port,
		uint16_t in_2_pin) {
	motor->input_1_port = in_1_port;
	motor->input_1_pin = in_1_pin;
	motor->input_2_port = in_2_port;
	motor->input_2_pin = in_2_pin;
	motor->speed_pwm = MAX_PWM;
}

void Motor_Open(struct Motor * motor) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor->speed_pwm);
	HAL_GPIO_WritePin(motor->input_1_port, motor->input_1_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->input_2_port, motor->input_2_pin, GPIO_PIN_RESET);
}

void Motor_Close(struct Motor * motor) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, motor->speed_pwm);
	HAL_GPIO_WritePin(motor->input_1_port, motor->input_1_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->input_2_port, motor->input_2_pin, GPIO_PIN_SET);
}

void Motor_Stop(struct Motor * motor) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	HAL_GPIO_WritePin(motor->input_1_port, motor->input_1_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->input_2_port, motor->input_2_pin, GPIO_PIN_SET);
}
