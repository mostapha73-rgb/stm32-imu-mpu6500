/*
 * imu.c
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 *
 *  Pilote du capteur inertiel MPU-6050 / MPU-6500 sur bus I2C.
 *
 *  Sequence d'initialisation :
 *    1) identification  (WHO_AM_I)
 *    2) reset logiciel  (repartir d'un etat connu)
 *    3) configuration   (tableau imu_config)
 *    4) verification    (relecture de chaque registre ecrit)
 */

#include "imu.h"

/* ---------- Constantes privees ---------- */
#define IMU_PWR_DEVICE_RESET   0x80   /* PWR_MGMT_1 bit 7 : remet tous les registres a leur valeur par defaut */
#define IMU_RESET_DELAY_MS     100    /* temps de redemarrage apres un reset (datasheet)                     */

/* ---------- Variables privees (invisibles hors de ce fichier) ---------- */
static uint8_t          imu_who_am_i   = 0;   /* identifiant lu a l'initialisation              */
static volatile uint8_t imu_data_ready = 0;   /* 1 = nouvelle mesure (ecrit par l'interruption) */

/* Configuration appliquee dans l'ordre par IMU_Init : { registre, valeur } */
static const uint8_t imu_config[][2] = {
    { IMU_REG_PWR_MGMT_1,   IMU_PWR_WAKE_PLL },   /* 1) reveil (toujours en premier) */
    { IMU_REG_CONFIG,       IMU_DLPF_10HZ    },   /* 2) filtre passe-bas             */
    { IMU_REG_SMPLRT_DIV,   IMU_SMPLRT_20HZ  },   /* 3) frequence d'echantillonnage  */
    { IMU_REG_GYRO_CONFIG,  IMU_GYRO_250DPS  },   /* 4) echelle du gyroscope         */
    { IMU_REG_ACCEL_CONFIG, IMU_ACCEL_2G     },   /* 5) echelle de l'accelerometre   */
    { IMU_REG_INT_ENABLE,   IMU_INT_DATA_RDY },   /* 6) interruption "donnee prete"  */
};

#define IMU_CONFIG_COUNT  (sizeof(imu_config) / sizeof(imu_config[0]))

/* ---------- Fonctions privees ---------- */

/* Ecrit un octet dans un registre du capteur */
static HAL_StatusTypeDef imu_write(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(bus, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, IMU_I2C_TIMEOUT_MS);
}

/* Lit un ou plusieurs registres consecutifs (auto-increment du capteur) */
static HAL_StatusTypeDef imu_read(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(bus, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, IMU_I2C_TIMEOUT_MS);
}

/* Ecrit un registre puis le relit pour verifier que la valeur a bien ete prise */
static HAL_StatusTypeDef imu_write_verify(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t val)
{
    uint8_t lu = 0;
    HAL_StatusTypeDef status;

    status = imu_write(bus, reg, val);
    if (status != HAL_OK)
        return status;

    status = imu_read(bus, reg, &lu, 1);
    if (status != HAL_OK)
        return status;

    return (lu == val) ? HAL_OK : HAL_ERROR;
}

/* Assemble deux octets (poids fort en premier) en un entier signe 16 bits */
static int16_t imu_to_int16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

/* ---------- Fonctions publiques ---------- */

/* Identifie, reinitialise, configure et verifie le capteur.
   Renvoie HAL_OK si tout s'est bien passe, sinon le premier code d'erreur rencontre. */
HAL_StatusTypeDef IMU_Init(I2C_HandleTypeDef *bus)
{
    HAL_StatusTypeDef status;

    if (bus == NULL)
        return HAL_ERROR;

    /* 1) Identification : le capteur doit repondre et etre reconnu */
    status = imu_read(bus, IMU_REG_WHO_AM_I, &imu_who_am_i, 1);
    if (status != HAL_OK)
        return status;

    if (imu_who_am_i != IMU_ID_MPU6050 && imu_who_am_i != IMU_ID_MPU6500)
        return HAL_ERROR;

    /* 2) Reset logiciel : apres un reset de la carte, le capteur (toujours alimente)
          garde son ancienne configuration. On le remet a zero pour partir d'un etat connu. */
    status = imu_write(bus, IMU_REG_PWR_MGMT_1, IMU_PWR_DEVICE_RESET);
    if (status != HAL_OK)
        return status;

    HAL_Delay(IMU_RESET_DELAY_MS);

    /* 3) et 4) Configuration + verification de chaque registre */
    for (uint32_t i = 0; i < IMU_CONFIG_COUNT; i++)
    {
        status = imu_write_verify(bus, imu_config[i][0], imu_config[i][1]);
        if (status != HAL_OK)
            return status;
    }

    return HAL_OK;
}

uint8_t IMU_GetWhoAmI(void)
{
    return imu_who_am_i;
}

/* Lit les 7 mesures (14 octets a partir de 0x3B) en une seule transaction :
   toutes les mesures correspondent au meme instant. */
HAL_StatusTypeDef IMU_ReadRaw(I2C_HandleTypeDef *bus, IMU_Raw_t *raw)
{
    uint8_t b[14];
    HAL_StatusTypeDef status;

    if (bus == NULL || raw == NULL)
        return HAL_ERROR;

    status = imu_read(bus, IMU_REG_ACCEL_XOUT_H, b, sizeof(b));
    if (status != HAL_OK)
        return status;

    raw->ax   = imu_to_int16(&b[0]);
    raw->ay   = imu_to_int16(&b[2]);
    raw->az   = imu_to_int16(&b[4]);
    raw->temp = imu_to_int16(&b[6]);
    raw->gx   = imu_to_int16(&b[8]);
    raw->gy   = imu_to_int16(&b[10]);
    raw->gz   = imu_to_int16(&b[12]);

    return HAL_OK;
}

/* Brut -> unites physiques (g, degC, deg/s) */
void IMU_Convert(const IMU_Raw_t *raw, IMU_donne_t *donne)
{
    if (raw == NULL || donne == NULL)
        return;

    donne->ax   = raw->ax   / IMU_ACCEL_LSB_PER_G;
    donne->ay   = raw->ay   / IMU_ACCEL_LSB_PER_G;
    donne->az   = raw->az   / IMU_ACCEL_LSB_PER_G;
    donne->temp = raw->temp / IMU_TEMP_LSB_PER_DEGC + IMU_TEMP_OFFSET_DEGC;
    donne->gx   = raw->gx   / IMU_GYRO_LSB_PER_DPS;
    donne->gy   = raw->gy   / IMU_GYRO_LSB_PER_DPS;
    donne->gz   = raw->gz   / IMU_GYRO_LSB_PER_DPS;
}

/* Unites physiques -> entiers x1000 (exemple : 1,009 g -> 1009 mg) */
void IMU_ToMilli(const IMU_donne_t *donne, IMU_milli_t *milli)
{
    if (donne == NULL || milli == NULL)
        return;

    milli->ax   = (int32_t)(donne->ax   * 1000.0f);
    milli->ay   = (int32_t)(donne->ay   * 1000.0f);
    milli->az   = (int32_t)(donne->az   * 1000.0f);
    milli->temp = (int32_t)(donne->temp * 1000.0f);
    milli->gx   = (int32_t)(donne->gx   * 1000.0f);
    milli->gy   = (int32_t)(donne->gy   * 1000.0f);
    milli->gz   = (int32_t)(donne->gz   * 1000.0f);
}

/* Appelee depuis l'interruption EXTI2 : une nouvelle mesure est prete.
   Une routine d'interruption doit rester tres courte : on leve seulement un drapeau. */
void IMU_DataReadyCallback(void)
{
    imu_data_ready = 1;
}

/* Renvoie 1 une seule fois par nouvelle mesure, puis remet le drapeau a 0.
   Section critique : on bloque les interruptions pendant le "lire puis effacer",
   sinon une interruption arrivant entre les deux serait perdue. */
uint8_t IMU_IsDataReady(void)
{
    uint8_t pret;

    __disable_irq();
    pret = imu_data_ready;
    imu_data_ready = 0;
    __enable_irq();

    return pret;
}
