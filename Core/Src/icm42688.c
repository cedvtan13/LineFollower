/*
 * icm42688.c  —  ICM-42688-P 6-axis IMU driver (I2C3)
 */

#include "icm42688.h"

extern I2C_HandleTypeDef hi2c3;

#define ICM_ADDR        0xD0u   /* 0x68 << 1 */
#define ICM_ACCEL_SCALE  4.0f   /* pixels of ball force per g    */
#define BALL_DAMP        0.88f  /* velocity multiplier per frame */

ImuData imu      = {0};
uint8_t icmReady = 0;
float   ballX    = 63.0f;
float   ballY    = 31.0f;
float   ballVx   = 0.0f;
float   ballVy   = 0.0f;

static float icmOffAx = 0.0f;
static float icmOffAy = 0.0f;

/* ---- Low-level register access ---- */

static void ICM_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t pkt[2] = {reg, val};
    HAL_I2C_Master_Transmit(&hi2c3, ICM_ADDR, pkt, 2, 10);
}

static void ICM_ReadRegs(uint8_t reg, uint8_t *out, uint8_t len)
{
    HAL_I2C_Master_Transmit(&hi2c3, ICM_ADDR, &reg, 1, 10);
    HAL_I2C_Master_Receive (&hi2c3, ICM_ADDR, out, len, 10);
}

/* ---- Public API ---- */

void ICM_Init(void)
{
    /* Soft reset */
    ICM_WriteReg(0x11, 0x01);
    HAL_Delay(100);

    /* Verify WHO_AM_I — ICM-42688-P always reads 0x47 */
    uint8_t who = 0;
    ICM_ReadRegs(0x75, &who, 1);
    if (who != 0x47) {
        icmReady = 0;
        return;
    }

    /* Accel + gyro low-noise mode */
    ICM_WriteReg(0x4E, 0x0F);
    HAL_Delay(5);

    /*
     * ACCEL_CONFIG0 = 0x46:
     *   bits [7:5] 010 = ±4 g  (sensitivity 8192 LSB/g)
     *   bits [3:0] 0110 = 200 Hz ODR
     */
    ICM_WriteReg(0x50, 0x46);

    icmReady = 1;
}

void ICM_ReadAccel(void)
{
    if (!icmReady) return;

    uint8_t raw[6];
    ICM_ReadRegs(0x1F, raw, 6);

    int16_t rawX = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t rawY = (int16_t)((raw[2] << 8) | raw[3]);

    /* Convert to g and subtract calibration bias */
    imu.ax = (float)rawX / 8192.0f - icmOffAx;
    imu.ay = (float)rawY / 8192.0f - icmOffAy;

    /*
     * Ball physics:
     *   Tilt right (ax > 0) → ball drifts right (+Vx)
     *   Tilt forward (ay > 0) → ball drifts up (–Vy because Y=0 is top)
     *   Damping prevents run-away velocity.
     *   Walls bounce at 50% energy loss.
     */
    ballVx = ballVx * BALL_DAMP + imu.ax * ICM_ACCEL_SCALE;
    ballVy = ballVy * BALL_DAMP - imu.ay * ICM_ACCEL_SCALE;

    ballX += ballVx;
    ballY += ballVy;

    /* Arena wall bouncing (ball centre = 2..125, 10..53) */
    if (ballX <   2.0f) { ballX =   2.0f; ballVx = -ballVx * 0.5f; }
    if (ballX > 125.0f) { ballX = 125.0f; ballVx = -ballVx * 0.5f; }
    if (ballY <  10.0f) { ballY =  10.0f; ballVy = -ballVy * 0.5f; }
    if (ballY >  53.0f) { ballY =  53.0f; ballVy = -ballVy * 0.5f; }
}

void ICM_Calibrate(void)
{
    if (!icmReady) return;

    float sumAx = 0.0f, sumAy = 0.0f;
    uint8_t raw[6];

    for (int i = 0; i < 200; i++) {
        ICM_ReadRegs(0x1F, raw, 6);
        int16_t rx = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ry = (int16_t)((raw[2] << 8) | raw[3]);
        sumAx += (float)rx / 8192.0f;
        sumAy += (float)ry / 8192.0f;
        HAL_Delay(5);
    }

    icmOffAx = sumAx / 200.0f;
    icmOffAy = sumAy / 200.0f;

    /* Re-centre the ball */
    ballX = 63.0f;
    ballY = 31.0f;
    ballVx = 0.0f;
    ballVy = 0.0f;
}
