/*
 * FinalRtosProtocolos.h
 *
 *  Created on: 10/4/2019
 *      Author: nelsonf
 */

#ifndef MIS_PROYECTOS_FINALRTOSPROTOCOLOS_INC_FINALRTOSPROTOCOLOS_H_
#define MIS_PROYECTOS_FINALRTOSPROTOCOLOS_INC_FINALRTOSPROTOCOLOS_H_

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "stdio.h"
#include "mpu9250_RTOS_Compatible.h"
#include "task.h"
#include "semphr.h"
#include "sapi.h"
// sAPI header
//#include "sapi.h"

// ---- Definicion de constantes
#define SAFE_TEMP_MI 60
#define SAFE_TEMP_MD 60
#define SAFE_TEMP_BMS 60

// --- Estructura que almacena todas las variables y estados del vehiculo
typedef enum {PARADO, ACELERANDO, FRENANDO, LISTO, ALARMA} stateCar;
typedef struct{
	stateCar estado;
	uint16_t aceleradorIn;		//--- Entrada del acelerador por medio del ADC
	uint16_t frenoIn;			//--- Entrada del freno por medio del ADC

	double giroscopoX;		//--- Posteriormente se analizara el acelerometro y magnetometro
	double giroscopoY;		//--- Para obtener tambien informacion de velocidad en el vehiculo
	double giroscopoZ;

	uint8_t start;		//--- Indica si el vehiculo esta listo para ser utilizado
	uint8_t alarma;		//--- Indica si se activo el boton de parada de emergencia

	float tempMI;			//--- Registro de temperatura del motor izquierdo
	float tempMD;			//--- Registro de temperatura del motor derecho
	float tempBMS;			//--- Registro de temperatura del pack de baterias, puede ser sustituido posteriormente
							//--- por la temperatura que provea el BMS (Battery Management System)
	float aceleradorOutMI; 	//--- Aceleracion diferencial
	float aceleradorOutMD;
	float frenoOutMI; 		//--- Freno diferencial (por determinar)
	float frenoOutMD;
}globalCar;

// --- Estados posibles de la maquina de estados
typedef enum {BUTTON_UP, BUTTON_FALLING, BUTTON_DOWN, BUTTON_RAISING} fsmDebounce_t;
/* Estructura de datos necesarios para la MEF antirebote*/
typedef struct{
	gpioMap_t tecla;
	fsmDebounce_t state;
	uint32_t delay;
	TickType_t tiempo_inicio_ciclo;
	TickType_t tiempo_fin_ciclo;
	TickType_t tiempo_presionado;
}debounceData_t;



#endif /* MIS_PROYECTOS_FINALRTOSPROTOCOLOS_INC_FINALRTOSPROTOCOLOS_H_ */
