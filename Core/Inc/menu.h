/*
 * menu.h  —  simple app state and PID settings
 */

#ifndef INC_MENU_H_
#define INC_MENU_H_

#include <stdint.h>

typedef enum {
	SCR_MAIN = 0,
	SCR_RUNNING,
	SCR_PID,
	SCR_CALIBRATE,
	SCR_SENSOR_DEBUG,
} AppScreen;

typedef struct {
	float   Kp;
	float   Kd;
	uint8_t baseSpeed;
	uint8_t turnSpeed;
} PIDConfig;

extern PIDConfig pid;
extern AppScreen currentScreen;
extern uint8_t   calibrated;
extern uint8_t   mainCursor;
extern uint8_t   pidCursor;
extern uint8_t   pidEdit;

void App_Init(void);
void App_Update(void);

#endif /* INC_MENU_H_ */
