/*
 * imu.h
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 *
 *  Pilote du capteur inertiel MPU-6050 / MPU-6500 (I2C).
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

/* Adresse I2C du capteur (7 bits, AD0 = GND), decalee pour la HAL */
#define IMU_I2C_ADDR           (0x68 << 1)

/* Registres, par ordre d'adresse (Register Map) */
#define IMU_REG_SMPLRT_DIV     0x19   /* diviseur d'echantillonnage        */
#define IMU_REG_CONFIG         0x1A   /* filtre passe-bas (DLPF_CFG)       */
#define IMU_REG_GYRO_CONFIG    0x1B   /* pleine echelle gyroscope          */
#define IMU_REG_ACCEL_CONFIG   0x1C   /* pleine echelle accelerometre      */
#define IMU_REG_INT_ENABLE     0x38   /* activation de l'interruption INT  */
#define IMU_REG_ACCEL_XOUT_H   0x3B   /* debut des 14 octets de mesures    */
#define IMU_REG_PWR_MGMT_1     0x6B   /* alimentation (bit 6 = SLEEP)      */
#define IMU_REG_WHO_AM_I       0x75   /* identifiant                       */

/* Facteurs de conversion (configuration actuelle) */
#define IMU_ACCEL_LSB_PER_G     16384.0f  /* +-2 g      : 16384 LSB par g   */
#define IMU_GYRO_LSB_PER_DPS    131.0f    /* +-250 deg/s : 131 LSB par deg/s */
#define IMU_TEMP_LSB_PER_DEGC   333.87f   /* MPU-6500 : 333,87 LSB par degC  */
#define IMU_TEMP_OFFSET_DEGC    21.0f     /* MPU-6500 : 0 LSB = 21 degC      */

/* Mesures brutes lues dans le capteur */
typedef struct {
    int16_t ax, ay, az;     /* accelerometre */
    int16_t temp;           /* temperature   */
    int16_t gx, gy, gz;     /* gyroscope     */
} IMU_Raw_t;

/* Mesures converties en unites physiques */
typedef struct {
    float ax, ay, az;       /* acceleration en g        */
    float temp;             /* temperature en degC      */
    float gx, gy, gz;       /* vitesse angulaire en deg/s */
} IMU_donne_t;

HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *bus);
uint8_t           IMU_GetWhoAmI(void);
HAL_StatusTypeDef IMU_ReadRaw(I2C_HandleTypeDef *bus, IMU_Raw_t *raw);
void              IMU_DataReadyCallback(void);

#endif /* INC_IMU_H_ */
