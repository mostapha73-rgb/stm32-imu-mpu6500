/*
 * app.c
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 *
 *  Application : lit le capteur toutes les APP_PERIOD_MS et signale l'etat par les LEDs.
 *
 *    LD4 (verte) clignote      : mesures lues normalement
 *    LD5 (rouge) allumee fixe  : capteur absent ou mal configure au demarrage
 *    LD5 (rouge) clignote      : erreurs de lecture en cours de fonctionnement
 */

#include "app.h"
#include "imu.h"

/* ---------- Reglages ---------- */
#define APP_PERIOD_MS       50      /* une lecture toutes les 50 ms (20 par seconde) */
#define APP_BOOT_DELAY_MS   100     /* attente du demarrage du capteur               */

/* ---------- LEDs (noms fonctionnels plutot que numeros) ---------- */
#define LED_OK_PORT         LD4_GPIO_Port   /* verte */
#define LED_OK_PIN          LD4_Pin
#define LED_ERR_PORT        LD5_GPIO_Port   /* rouge */
#define LED_ERR_PIN         LD5_Pin

/* ---------- Etat interne de l'application ---------- */
static I2C_HandleTypeDef *imu_bus       = NULL;   /* bus I2C memorise par App_Init      */
static uint8_t            imu_ok        = 0;      /* 1 = capteur detecte et configure   */
static uint32_t           app_last_tick = 0;      /* instant de la derniere lecture (ms) */

/* ---------- A observer dans Live Expressions ---------- */
static volatile IMU_Raw_t   imu_raw;              /* brut                   */
static volatile IMU_donne_t imu_donne;            /* g, degC, deg/s         */
static volatile IMU_milli_t imu_milli;            /* mg, milli-degC, mdeg/s */
static volatile uint32_t    app_nb_mesures = 0;   /* lectures reussies      */
static volatile uint32_t    app_nb_erreurs = 0;   /* lectures echouees      */

void App_Init(I2C_HandleTypeDef *bus)
{
    imu_bus = bus;

    HAL_GPIO_WritePin(LED_OK_PORT,  LED_OK_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_ERR_PORT, LED_ERR_PIN, GPIO_PIN_RESET);

    HAL_Delay(APP_BOOT_DELAY_MS);

    imu_ok = (IMU_Init(imu_bus) == HAL_OK);

    if (!imu_ok)
        HAL_GPIO_WritePin(LED_ERR_PORT, LED_ERR_PIN, GPIO_PIN_SET);   /* rouge fixe */

    app_last_tick = HAL_GetTick();
}

void App_Run(void)
{
    IMU_Raw_t   raw;       /* copies de travail locales */
    IMU_donne_t donne;
    IMU_milli_t milli;

    /* 1) Capteur absent : rien a faire (la LED rouge reste allumee) */
    if (!imu_ok)
        return;

    /* 2) Cadencement non bloquant : on ne lit que toutes les APP_PERIOD_MS.
          Contrairement a HAL_Delay, la boucle de main.c continue de tourner. */
    if ((HAL_GetTick() - app_last_tick) < APP_PERIOD_MS)
        return;
    app_last_tick = HAL_GetTick();

    /* 3) Lecture */
    if (IMU_ReadRaw(imu_bus, &raw) != HAL_OK)
    {
        app_nb_erreurs++;
        HAL_GPIO_TogglePin(LED_ERR_PORT, LED_ERR_PIN);   /* rouge clignote : erreur */
        return;
    }

    /* 4) Conversions */
    IMU_Convert(&raw, &donne);         /* brut -> g, degC, deg/s */
    IMU_ToMilli(&donne, &milli);       /* -> entiers x1000       */

    /* 5) Publication pour le debogueur */
    imu_raw   = raw;
    imu_donne = donne;
    imu_milli = milli;
    app_nb_mesures++;

    HAL_GPIO_TogglePin(LED_OK_PORT, LED_OK_PIN);         /* verte clignote : OK */
}
