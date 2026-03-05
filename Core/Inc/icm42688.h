/*
 * icm42688.h  —  ICM-42688-P 6-axis IMU driver (I2C3)
 *
 * I2C address: 0x68 (AD0 low) → 0xD0 shifted
 * SCL: PA8  SDA: PB4   Speed: 400 kHz
 *
 * Registers used:
 *   0x11 DEVICE_CONFIG   — soft reset
 *   0x4E PWR_MGMT0       — 0x0F: accel + gyro low-noise mode
 *   0x50 ACCEL_CONFIG0   — FSR + ODR (0x46 = ±4 g @ 200 Hz)
 *   0x1F ACCEL_DATA_X1   — MSB of X accel (6 bytes: X,Y,Z each 16-bit)
 *   0x75 WHO_AM_I        — 0x47 expected
 *
 * Accel scale (±4 g): 8192 LSB/g.
 *
 * This driver also maintains the ball-physics state used by the
 * gyro visualisation screen in menu.c:
 *   ballX / ballY  — pixel position of the rolling ball (0-127, 0-63)
 *   ballVx / ballVy — velocity (pixels per frame)
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_

#include "main.h"
#include <stdint.h>

typedef struct {
    float ax;   /* accel X in g (positive = tilt right)   */
    float ay;   /* accel Y in g (positive = tilt forward) */
    float az;   /* accel Z in g (positive = face up)      */
} ImuData;

extern ImuData  imu;         /* latest calibrated accelerometer reading */
extern uint8_t  icmReady;    /* 1 after successful WHO_AM_I check       */
extern float    ballX;       /* ball pixel X (2…125)  */
extern float    ballY;       /* ball pixel Y (10…53)  */
extern float    ballVx;      /* ball velocity X       */
extern float    ballVy;      /* ball velocity Y       */

/* Initialise the ICM-42688-P.  Call once after hi2c3 is ready. */
void ICM_Init(void);

/*
 * Read a fresh accelerometer sample, update imu.ax/ay, and advance
 * the ball physics simulation.  Call at ~20 Hz (every 50 ms).
 */
void ICM_ReadAccel(void);

/*
 * Record accelerometer bias over ~1 second (200 samples × 5 ms).
 * Hold the board flat and still before calling.
 * Resets ball position to centre when done.
 */
void ICM_Calibrate(void);

#endif /* INC_ICM42688_H_ */
