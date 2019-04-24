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
#include "queue.h"
#include "sapi.h"
#include <string.h>
#include "stringManipulation.h"
#include "board.h"
//#include "IRQConfigAndHandlers.h"

// ---- Definicion de constantes
#define SAFE_TEMP_MI 60
#define SAFE_TEMP_MD 60
#define SAFE_TEMP_BMS 60

#define 	taskLedVariablePriority 		( tskIDLE_PRIORITY + 1)
#define  	taskProcessorPriority 			( tskIDLE_PRIORITY + 5)
#define  	taskAceleradorFrenoPriority		( tskIDLE_PRIORITY + 4)
#define  	taskEnvioDatosPriority			( tskIDLE_PRIORITY + 2)
#define  	taskGiroscopoPriority			( tskIDLE_PRIORITY + 4)
#define  	taskUartConnectionPriority		( tskIDLE_PRIORITY + 2)
#define		taskAntirreboteTecXPriority		( tskIDLE_PRIORITY + 3)

// --- Estructura que almacena todas las variables y estados del vehiculo
typedef enum {PARADO, ACELERANDO, FRENANDO, LISTO, ALARMA} stateCar;
typedef struct{
	stateCar estado;
	uint16_t aceleradorIn;		//--- Entrada del acelerador por medio del ADC
	uint16_t frenoIn;			//--- Entrada del freno por medio del ADC

	float giroscopoX;		//--- Posteriormente se analizara el acelerometro y magnetometro
	float giroscopoY;		//--- Para obtener tambien informacion de velocidad en el vehiculo
	float giroscopoZ;		//--- Y realizar un factor de corrección del giro de ser necesario

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

typedef enum {UP, DOWN, FALLING, RAISING} stateBTN;
typedef struct{
	stateBTN state;         //-- Estado actual
	TickType_t tic;			//-- Inicio de cuenta
	TickType_t toc;			//-- Fin de cuenta
	gpioMap_t tec;            //-- Indica donde se encuentra fisicamente conectada la tecla
}btnStruct;



#endif /* MIS_PROYECTOS_FINALRTOSPROTOCOLOS_INC_FINALRTOSPROTOCOLOS_H_ */
