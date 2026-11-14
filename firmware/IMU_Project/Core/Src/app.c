/*
 * app.c
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 */

#include "main.h"
#include "app.h"
#include "imu.h"

static uint8_t imu_ok = 0;
static I2C_HandleTypeDef *imu_bus = NULL;   /* bus memorise pour App_Run */
static volatile IMU_Raw_t   imu_raw;        /* dernieres mesures brutes   */
static volatile IMU_donne_t imu_donne;      /* mesures en g, deg/s, degC  */

void App_Init(I2C_HandleTypeDef *bus)
{
    imu_bus = bus;

    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin | LD5_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);                          /* demarrage du capteur */

    if (IMU_Init(bus) == HAL_OK &&
        (IMU_GetWhoAmI() == 0x68 || IMU_GetWhoAmI() == 0x70))
        imu_ok = 1;                          /* MPU-6050 (0x68) ou MPU-6500 (0x70) */
}

void App_Run(void)
{
    if (imu_ok && IMU_ReadRaw(imu_bus, (IMU_Raw_t *)&imu_raw) == HAL_OK)
    {
        /* Conversion : brut -> unites physiques */
        imu_donne.ax   = imu_raw.ax / IMU_ACCEL_LSB_PER_G;
        imu_donne.ay   = imu_raw.ay / IMU_ACCEL_LSB_PER_G;
        imu_donne.az   = imu_raw.az / IMU_ACCEL_LSB_PER_G;
        imu_donne.temp = imu_raw.temp / IMU_TEMP_LSB_PER_DEGC + IMU_TEMP_OFFSET_DEGC;
        imu_donne.gx   = imu_raw.gx / IMU_GYRO_LSB_PER_DPS;
        imu_donne.gy   = imu_raw.gy / IMU_GYRO_LSB_PER_DPS;
        imu_donne.gz   = imu_raw.gz / IMU_GYRO_LSB_PER_DPS;

        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);   /* verte : mesure lue */
    }
    else
    {
        HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);   /* rouge : erreur */
    }

    HAL_Delay(50);                                    /* ~20 lectures par seconde */
}
