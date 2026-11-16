/*
 * imu.h
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 *
 *  Pilote du capteur inertiel MPU-6050 / MPU-6500 sur bus I2C.
 *  Chaine de traitement : capteur -> IMU_Raw_t -> IMU_donne_t -> IMU_milli_t
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

/* ---------- Bus I2C ---------- */
#define IMU_I2C_ADDR           (0x68 << 1)  /* adresse 7 bits (AD0 = GND), decalee pour la HAL */
#define IMU_I2C_TIMEOUT_MS     100          /* attente maximale d'une transaction I2C          */

/* ---------- Identifiants possibles (registre WHO_AM_I) ---------- */
#define IMU_ID_MPU6050         0x68
#define IMU_ID_MPU6500         0x70

/* ---------- Registres utilises (Register Map) ---------- */
#define IMU_REG_SMPLRT_DIV     0x19   /* diviseur d'echantillonnage        */
#define IMU_REG_CONFIG         0x1A   /* filtre passe-bas (DLPF)           */
#define IMU_REG_GYRO_CONFIG    0x1B   /* pleine echelle du gyroscope       */
#define IMU_REG_ACCEL_CONFIG   0x1C   /* pleine echelle de l'accelerometre */
#define IMU_REG_INT_ENABLE     0x38   /* activation de la sortie INT       */
#define IMU_REG_ACCEL_XOUT_H   0x3B   /* debut des 14 octets de mesures    */
#define IMU_REG_PWR_MGMT_1     0x6B   /* alimentation et horloge           */
#define IMU_REG_WHO_AM_I       0x75   /* identifiant du capteur            */

/* ---------- Valeurs ecrites dans ces registres (cahier des charges) ---------- */
#define IMU_PWR_WAKE_PLL       0x01   /* SLEEP = 0, horloge = PLL du gyro X  */
#define IMU_DLPF_10HZ          0x05   /* filtre ~10 Hz, gyro echantillonne a 1 kHz */
#define IMU_SMPLRT_20HZ        49     /* 1 kHz / (1 + 49) = 20 Hz            */
#define IMU_GYRO_250DPS        0x00   /* +-250 deg/s                         */
#define IMU_ACCEL_2G           0x00   /* +-2 g                               */
#define IMU_INT_DATA_RDY       0x01   /* impulsion INT a chaque mesure       */

/* ---------- Facteurs de conversion (lies aux echelles ci-dessus) ---------- */
#define IMU_ACCEL_LSB_PER_G    16384.0f  /* +-2 g       : 16384 LSB par g      */
#define IMU_GYRO_LSB_PER_DPS   131.0f    /* +-250 deg/s : 131 LSB par deg/s    */
#define IMU_TEMP_LSB_PER_DEGC  333.87f   /* MPU-6500    : 333,87 LSB par degC  */
#define IMU_TEMP_OFFSET_DEGC   21.0f     /* MPU-6500    : 0 LSB = 21 degC      */

/* ---------- Types ---------- */

/* 1) Mesures brutes, telles que lues dans le capteur */
typedef struct {
    int16_t ax, ay, az;     /* accelerometre */
    int16_t temp;           /* temperature   */
    int16_t gx, gy, gz;     /* gyroscope     */
} IMU_Raw_t;

/* 2) Mesures en unites physiques (pour les calculs) */
typedef struct {
    float ax, ay, az;       /* acceleration en g          */
    float temp;             /* temperature en degC        */
    float gx, gy, gz;       /* vitesse angulaire en deg/s */
} IMU_donne_t;

/* 3) Memes mesures x1000 en entiers (pour l'affichage et l'envoi) */
typedef struct {
    int32_t ax, ay, az;     /* acceleration en mg (milli-g)     */
    int32_t temp;           /* temperature en milli-degC        */
    int32_t gx, gy, gz;     /* vitesse angulaire en milli-deg/s */
} IMU_milli_t;

/* ---------- Fonctions ---------- */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *bus);
uint8_t           IMU_GetWhoAmI(void);
HAL_StatusTypeDef IMU_ReadRaw(I2C_HandleTypeDef *bus, IMU_Raw_t *raw);
void              IMU_Convert(const IMU_Raw_t *raw, IMU_donne_t *donne);
void              IMU_ToMilli(const IMU_donne_t *donne, IMU_milli_t *milli);

/* Interruption "donnee prete" (broche INT -> PC2 -> EXTI2) */
void              IMU_DataReadyCallback(void);
uint8_t           IMU_IsDataReady(void);

#endif /* INC_IMU_H_ */
