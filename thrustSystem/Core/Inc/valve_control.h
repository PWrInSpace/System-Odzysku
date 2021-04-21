#ifndef VALVE_CTRL
#define VALVE_CTRL

#include "gpio.h"
#include "tim.h"

#include "motor_control.h"

#define True  1
#define False 0

struct Valve {

	// max warosc encodera to: 12
	// czyli wychodzi 6 wykrywalnych stanow posrednich
	// czyli obracamy zaworem co 15 stopni (90 stopni / 6 stanow = 15 stopni/stan)

	int8_t state;
	int8_t max_state;
	uint8_t is_closed;
	uint8_t is_opened;
	int8_t encoder_tick;

	struct Motor * motor;
};

void Valve_Init(struct Valve * valve, struct Motor * motor, int8_t set_max_state, int8_t encoder_tick);
void Valve_Open_Precisely(struct Valve * valve, int8_t step);
void Valve_Close_Precisely(struct Valve * valve, int8_t step);
void Valve_Force_Close(struct Valve * valve);
void Valve_Set_End_Position(struct Valve * valve, uint8_t is_opened, uint8_t is_closed);
void Valve_Get_Currnet_Position(struct Valve * valve);

// tymczasowe funkcje
void Valve_Open(struct Valve * valve);
void Valve_Close(struct Valve * valve);

#endif
