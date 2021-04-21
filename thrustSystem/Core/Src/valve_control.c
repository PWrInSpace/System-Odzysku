#include "valve_control.h"

void Valve_Init(struct Valve * valve, struct Motor * motor, int8_t max_state, int8_t encoder_tick) {
	valve->max_state = max_state;
	valve->encoder_tick = encoder_tick;
	valve->state = 0;
	valve->is_closed = False;
	valve->is_opened = False;
	valve->motor = motor;
}

void Valve_Open_Precisely(struct Valve * valve, int8_t step) {
	int8_t limit = valve->state + valve->encoder_tick*step;
	if (limit > valve->max_state) {
		limit = valve->max_state;
	}
	while (valve->state <= limit && valve->state <= valve->max_state){
		Motor_Open(valve->motor);
		Valve_Get_Currnet_Position(valve);
	}
}

void Valve_Close_Precisely(struct Valve * valve, int8_t step) {
	int8_t limit = valve->state - valve->encoder_tick*step;
	if (limit < 0) {
		limit = 0;
	}
	while (valve->state <= limit && valve->state <= valve->max_state){
		Motor_Close(valve->motor);
		Valve_Get_Currnet_Position(valve);
	}
}

void Valve_Force_Close(struct Valve * valve) {
	while (HAL_GPIO_ReadPin(Limit_Switch_Close_GPIO_Port, Limit_Switch_Close_Pin)) {
			Motor_Close(valve->motor);
	}
}

void Valve_Set_End_Position(struct Valve * valve, uint8_t is_opened, uint8_t is_closed) {
	valve->is_opened = is_opened;
	valve->is_closed = is_closed;
}

void Valve_Get_Currnet_Position(struct Valve * valve) {
	int8_t encoder_state = TIM4->CNT;
	valve->state = encoder_state;
}

void Valve_Open(struct Valve * valve) {
	Motor_Open(valve->motor);
}

void Valve_Close(struct Valve * valve) {
	Motor_Close(valve->motor);
}
