/*
 * imu.c
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 *
 *  Pilote du capteur inertiel MPU-6050 / MPU-6500 (I2C).
 */

#include "imu.h"

static volatile uint8_t imu_who_am_i   = 0;
static volatile uint8_t imu_data_ready = 0;   /* 1 = nouvelle mesure disponible */

/* Ecriture d'un registre du capteur (fonction privee) */
static HAL_StatusTypeDef imu_write(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(bus, IMU_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *bus)
{
    HAL_StatusTypeDef status;

    if (bus == NULL)
        return HAL_ERROR;

    /* 1) Identification : lecture du WHO_AM_I */
    status = HAL_I2C_Mem_Read(bus, IMU_I2C_ADDR, IMU_REG_WHO_AM_I,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)&imu_who_am_i, 1, 100);
    if (status != HAL_OK)
        return status;

    /* 2) Reveil */
    status = imu_write(bus, IMU_REG_PWR_MGMT_1, 0x01);     /* SLEEP = 0, horloge PLL gyro X */
    if (status != HAL_OK)
        return status;

    /* 3) Configuration (cahier des charges) */
    status = imu_write(bus, IMU_REG_CONFIG, 0x05);         /* DLPF ~10 Hz, gyro a 1 kHz */
    if (status != HAL_OK)
        return status;

    status = imu_write(bus, IMU_REG_SMPLRT_DIV, 49);       /* 1 kHz / (1 + 49) = 20 Hz  */
    if (status != HAL_OK)
        return status;

    status = imu_write(bus, IMU_REG_GYRO_CONFIG, 0x00);    /* +-250 deg/s */
    if (status != HAL_OK)
        return status;

    status = imu_write(bus, IMU_REG_ACCEL_CONFIG, 0x00);   /* +-2 g */
    if (status != HAL_OK)
        return status;

    /* 4) Interruption : impulsion sur INT a chaque nouvelle mesure */
    status = imu_write(bus, IMU_REG_INT_ENABLE, 0x01);     /* DATA_RDY_EN = 1 */
    if (status != HAL_OK)
        return status;

    return HAL_OK;
}

uint8_t IMU_GetWhoAmI(void)
{
    return imu_who_am_i;
}

/* Lecture des 7 mesures brutes (14 octets) en une seule transaction */
HAL_StatusTypeDef IMU_ReadRaw(I2C_HandleTypeDef *bus, IMU_Raw_t *raw)
{
    uint8_t b[14];
    HAL_StatusTypeDef status;

    if (bus == NULL || raw == NULL)
        return HAL_ERROR;

    status = HAL_I2C_Mem_Read(bus, IMU_I2C_ADDR, IMU_REG_ACCEL_XOUT_H,
                              I2C_MEMADD_SIZE_8BIT, b, 14, 100);
    if (status != HAL_OK)
        return status;

    raw->ax   = (int16_t)(b[0]  << 8 | b[1]);
    raw->ay   = (int16_t)(b[2]  << 8 | b[3]);
    raw->az   = (int16_t)(b[4]  << 8 | b[5]);
    raw->temp = (int16_t)(b[6]  << 8 | b[7]);
    raw->gx   = (int16_t)(b[8]  << 8 | b[9]);
    raw->gy   = (int16_t)(b[10] << 8 | b[11]);
    raw->gz   = (int16_t)(b[12] << 8 | b[13]);

    return HAL_OK;
}

/* Appelee depuis l'interruption EXTI2 (broche INT du capteur) :
   une nouvelle mesure est prete */
void IMU_DataReadyCallback(void)
{
    imu_data_ready = 1;
}
