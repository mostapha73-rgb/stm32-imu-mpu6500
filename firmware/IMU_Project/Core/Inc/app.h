/*
 * app.h
 *
 *  Created on: 3 nov. 2026
 *      Author: mostapha
 *
 *  Couche application de la centrale inertielle.
 *  Utilisation depuis main.c :
 *    - App_Init(&hi2c1) une seule fois, dans USER CODE BEGIN 2
 *    - App_Run()        en boucle,      dans USER CODE BEGIN 3
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"   /* pour I2C_HandleTypeDef */

/**
 * @brief  Initialise l'application : memorise le bus, eteint les LEDs,
 *         puis detecte et configure le capteur.
 * @param  bus  bus I2C sur lequel est branche le capteur (ex. &hi2c1)
 */
void App_Init(I2C_HandleTypeDef *bus);

/**
 * @brief  Un cycle de l'application : lit le capteur, convertit les mesures,
 *         signale l'etat par les LEDs (LD4 = OK, LD5 = erreur).
 *         A appeler en boucle dans le while(1) de main.c.
 */
void App_Run(void);

#endif /* INC_APP_H_ */
