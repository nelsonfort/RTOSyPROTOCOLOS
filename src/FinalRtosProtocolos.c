/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inclusiones]============================================*/
#include "FinalRtosProtocolos.h"

/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

DEBUG_PRINT_ENABLE;
/*==================[declaraciones de variables globales]====================*/
globalCar vehiculo; //--- Estructura que almacena todas las variables globales y estado del veh[iculo.


uint8_t delayInicial = 30;

//Sincronizacion y proteccion de datos
SemaphoreHandle_t SemBinTec1RiseEdge = NULL;
SemaphoreHandle_t SemBinTec1FallEdge = NULL;
SemaphoreHandle_t SemBinTec2RiseEdge = NULL;
SemaphoreHandle_t SemBinTec2FallEdge = NULL;
SemaphoreHandle_t SemBinTec3RiseEdge = NULL;
SemaphoreHandle_t SemBinTec3FallEdge = NULL;
SemaphoreHandle_t SemBinTec4RiseEdge = NULL;
SemaphoreHandle_t SemBinTec4FallEdge = NULL;
xSemaphoreHandle mutexSem = NULL;
//Cola de mensajes para enviar variables por la UART
QueueHandle_t colaMsg;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/
//-- Funciones
void configInterrupts(void);

//-- Handlers de interrupciones
void GPIO0_IRQHandler(void); 	//-- handler que atiende eventos de la tecla 1
void GPIO1_IRQHandler(void);	//-- handler que atiende eventos de la tecla 2
void GPIO2_IRQHandler(void);	//-- handler que atiende eventos de la tecla 3
void GPIO3_IRQHandler(void);	//-- handler que atiende eventos de la tecla 4

// Prototipo de funcion de la tarea
void taskLedVariable( void* taskParmPtr );
void taskAntirreboteTec1( void* taskParmPtr );
void taskAntirreboteTec2( void* taskParmPtr );
void taskAntirreboteTec3( void* taskParmPtr );
void taskAntirreboteTec4( void* taskParmPtr );

//--- Tareas propias del vehiculo
void taskProcessor(void* taskParmPtr);
void taskAceleradorFreno(void* taskParmPtr );
void taskEnvioDatos(void* taskParmPtr );
void taskGiroscopo(void* taskParmPtr);
void taskUartConnection( void* taskParmPtr );
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();
   configInterrupts();
   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );

   // Crear tarea en freeRTOS
   xTaskCreate(
		   taskLedVariable,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskLedVariable",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   taskLedVariablePriority,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
   );

   /*xTaskCreate(
		   taskProcessor,
   		   (const char *)"taskProcessor",
   		   configMINIMAL_STACK_SIZE*2,
   		   0,
		   taskProcessorPriority,         // Prioridad de la tarea
   		   0
       	);
   xTaskCreate(
		   taskAceleradorFreno,
		   (const char *)"taskAceleradorFreno",
		   configMINIMAL_STACK_SIZE*2,
		   0,
		   taskAceleradorFrenoPriority,         // Prioridad de la tarea
		   0
    	);
   xTaskCreate(
		   taskEnvioDatos,
		   (const char *)"taskEnvioDatos",
		   configMINIMAL_STACK_SIZE*6,
		   0,
		   taskEnvioDatosPriority,         // Prioridad de la tarea
		   0
    	);
   xTaskCreate(
		   taskGiroscopo,
		   (const char *)"taskGiroscopo",
		   configMINIMAL_STACK_SIZE*2,
		   0,
		   taskGiroscopoPriority,         // Prioridad de la tarea
		   0
    	);
   xTaskCreate(
		   taskUartConnection,
   		   (const char *)"taskUartConnection",
   		   configMINIMAL_STACK_SIZE*2,
   		   0,
		   taskUartConnectionPriority,         // Prioridad de la tarea
   		   0
       	);*/
   xTaskCreate(
   		   taskAntirreboteTec1,
   		   (const char *)"taskAntirreboteTec1",
   		   configMINIMAL_STACK_SIZE*1,
   		   0,
		   taskAntirreboteTecXPriority,
   		   0
      );
   xTaskCreate(
   		   taskAntirreboteTec2,
   		   (const char *)"taskAntirreboteTec2",
   		   configMINIMAL_STACK_SIZE*1,
   		   0,
		   taskAntirreboteTecXPriority,
   		   0
      );
   xTaskCreate(
   		   taskAntirreboteTec3,
   		   (const char *)"taskAntirreboteTec3",
   		   configMINIMAL_STACK_SIZE*1,
   		   0,
		   taskAntirreboteTecXPriority,
   		   0
      );
   xTaskCreate(
   		   taskAntirreboteTec4,
   		   (const char *)"taskAntirreboteTec4",
   		   configMINIMAL_STACK_SIZE*1,
   		   0,
		   taskAntirreboteTecXPriority,
   		   0
      );
   // Iniciar scheduler
   vTaskStartScheduler();

   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   return 0;
}
void configInterrupts(void){
	Chip_PININT_Init(LPC_GPIO_PIN_INT);
	//--- Mapeo entre el pin fisico y el canal del handler que atendera la interrupcion
	Chip_SCU_GPIOIntPinSel(0, 0, 4); //TEC1
	Chip_SCU_GPIOIntPinSel(1, 0, 8); //TEC2
	Chip_SCU_GPIOIntPinSel(2, 0, 9); //TEC3
	Chip_SCU_GPIOIntPinSel(3, 1, 9); //TEC4

	//-- Configuro interrupcion TEC1
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);//Configuracion de interrupcion por flanco
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);//Flanco descendente
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH0);//Flanco ascendente
	NVIC_SetPriority( PIN_INT0_IRQn, 5 );
	NVIC_EnableIRQ(PIN_INT0_IRQn);


	//-- Configuro interrupcion TEC2
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);//Configuracion de interrupcion por flanco
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH1);//Flanco descendente
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH1);//Flanco ascendente
	NVIC_SetPriority( PIN_INT1_IRQn, 5 );
	NVIC_EnableIRQ(PIN_INT1_IRQn);

	//-- Configuro interrupcion TEC3
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH2);//Configuracion de interrupcion por flanco
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH2);//Flanco descendente
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH2);//Flanco ascendente
	NVIC_SetPriority( PIN_INT2_IRQn, 5 );
	NVIC_EnableIRQ(PIN_INT2_IRQn);

	//-- Configuro interrupcion TEC4
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH3);//Configuracion de interrupcion por flanco
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH3);//Flanco descendente
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH3);//Flanco ascendente
	NVIC_SetPriority( PIN_INT3_IRQn, 5 );
	NVIC_EnableIRQ(PIN_INT3_IRQn);


	//-- Creo los semaforos que voy a utilizar en cada tecla para cada flanco
	SemBinTec1RiseEdge = xSemaphoreCreateBinary();
	SemBinTec1FallEdge = xSemaphoreCreateBinary();
	SemBinTec2RiseEdge = xSemaphoreCreateBinary();
	SemBinTec2FallEdge = xSemaphoreCreateBinary();
	SemBinTec3RiseEdge = xSemaphoreCreateBinary();
	SemBinTec3FallEdge = xSemaphoreCreateBinary();
	SemBinTec4RiseEdge = xSemaphoreCreateBinary();
	SemBinTec4FallEdge = xSemaphoreCreateBinary();
	//-- Por defecto inician tomados por lo cual estan listos para ser liberados por los handlers.
}
void GPIO0_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpioToggle(LEDG);//Debug
	if( Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH0);
		xSemaphoreGiveFromISR(SemBinTec1FallEdge, &xHigherPriorityTaskWoken);
	}
	else if( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH0){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH0);
		xSemaphoreGiveFromISR(SemBinTec1RiseEdge, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void GPIO1_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpioToggle(LED2);//Debug
	if( Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH1){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH1);
		xSemaphoreGiveFromISR(SemBinTec2FallEdge, &xHigherPriorityTaskWoken);
	}
	else if( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH1){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH1);
		xSemaphoreGiveFromISR(SemBinTec2RiseEdge, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void GPIO2_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpioToggle(LED3);//Debug
	if( Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH2){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH2);
		xSemaphoreGiveFromISR(SemBinTec3FallEdge, &xHigherPriorityTaskWoken);
	}
	else if( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH2){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH2);
		xSemaphoreGiveFromISR(SemBinTec3RiseEdge, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
void GPIO3_IRQHandler(void){
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	gpioToggle(LEDB); //Debug
	if( Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH3){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH3);
		xSemaphoreGiveFromISR(SemBinTec4FallEdge, &xHigherPriorityTaskWoken);
	}
	else if( Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH3){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH3);
		xSemaphoreGiveFromISR(SemBinTec4RiseEdge, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void taskAntirreboteTec1( void* taskParmPtr ){
	//-- TEC1 <=> START
	btnStruct 	tecla1;
	vTaskDelay(3000/portTICK_RATE_MS);
	tecla1.state = UP;
	tecla1.tec = TEC1;
	tecla1.tic = 0;
	tecla1.toc = 0;
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();

	while(1){
		if( tecla1.state == UP ){
			if(xSemaphoreTake(SemBinTec1FallEdge,( TickType_t ) 1) ){ //Sucede un flanco descendente timeout de 1ms
				tecla1.tic = xTaskGetTickCount();
				tecla1.state = FALLING;
			}
		}
		else if( tecla1.state == FALLING ){
			tecla1.toc = xTaskGetTickCount();

			if(tecla1.toc - tecla1.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla1.tec) == 0){
					tecla1.state = DOWN;
				}
			}
		}
		else if( tecla1.state == DOWN ){
			if(xSemaphoreTake(SemBinTec1RiseEdge,( TickType_t ) 1)){ //Sucede un flanco ascendente  timeout de 1ms
				tecla1.tic = xTaskGetTickCount();
				tecla1.state = RAISING;
			}
		}
		else if( tecla1.state == RAISING ){
			tecla1.toc = xTaskGetTickCount();

			if(tecla1.toc - tecla1.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla1.tec) == 1){
					tecla1.state = UP;
					//vehiculo.start = ON;
					/*
					//-- En este momento se cumplio todo el ciclo del antirebote
					//-- Antes de dejarlo en el estado inicial se activa la variable deseada.
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, portMAX_DELAY ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							vehiculo.start = ON;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
						else
						{
							// No se pudo ingresar al mutex
						}
					}*/
					gpioToggle(LEDR);
				}
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,5/portTICK_RATE_MS);
	}
}

void taskAntirreboteTec2( void* taskParmPtr ){
	//-- TEC2 <=> STOP
	btnStruct 	tecla2;
	vTaskDelay(3000/portTICK_RATE_MS);
	tecla2.state = UP;
	tecla2.tec = TEC2;
	tecla2.tic = 0;
	tecla2.toc = 0;
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();

	while(1){
		if( tecla2.state == UP ){
			if(xSemaphoreTake(SemBinTec2FallEdge,( TickType_t ) 1) ){ //Sucede un flanco descendente timeout de 1ms
				tecla2.tic = xTaskGetTickCount();
				tecla2.state = FALLING;
			}
		}
		else if( tecla2.state == FALLING ){
			tecla2.toc = xTaskGetTickCount();

			if(tecla2.toc - tecla2.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla2.tec) == 0){
					tecla2.state = DOWN;
				}
			}
		}
		else if( tecla2.state == DOWN ){
			if(xSemaphoreTake(SemBinTec2RiseEdge,( TickType_t ) 1)){ //Sucede un flanco ascendente  timeout de 1ms
				tecla2.tic = xTaskGetTickCount();
				tecla2.state = RAISING;
			}
		}
		else if( tecla2.state == RAISING ){
			tecla2.toc = xTaskGetTickCount();

			if(tecla2.toc - tecla2.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla2.tec) == 1){
					tecla2.state = UP;
					//vehiculo.start = OFF;
					/*
					//-- En este momento se cumplio todo el ciclo del antirebote
					//-- Antes de dejarlo en el estado inicial se activa la variable deseada.
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem,portMAX_DELAY  ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							vehiculo.start = OFF;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
						else
						{
							// No se pudo ingresar al mutex
						}
					}*/
					gpioToggle(LEDR);
				}
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,5/portTICK_RATE_MS);
	}
}

void taskAntirreboteTec3( void* taskParmPtr ){
	//-- TEC3 <=> P.EMERGENCIA
	btnStruct 	tecla3;
	vTaskDelay(3000/portTICK_RATE_MS);
	tecla3.state = UP;
	tecla3.tec = TEC3;
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();

	while(1){
		if( tecla3.state == UP ){
			if(xSemaphoreTake(SemBinTec3FallEdge,( TickType_t ) 1) ){ //Sucede un flanco descendente timeout de 1ms
				tecla3.tic = xTaskGetTickCount();
				tecla3.state = FALLING;
			}
		}
		else if( tecla3.state == FALLING ){
			tecla3.toc = xTaskGetTickCount();

			if(tecla3.toc - tecla3.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla3.tec) == 0){
					tecla3.state = DOWN;
				}
			}
		}
		else if( tecla3.state == DOWN ){
			if(xSemaphoreTake(SemBinTec3RiseEdge,( TickType_t ) 1)){ //Sucede un flanco ascendente  timeout de 1ms
				tecla3.tic = xTaskGetTickCount();
				tecla3.state = RAISING;
			}
		}
		else if( tecla3.state == RAISING ){
			tecla3.toc = xTaskGetTickCount();

			if(tecla3.toc - tecla3.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla3.tec) == 1){
					tecla3.state = UP;
					//vehiculo.alarma = ON;
					/*
					//-- En este momento se cumplio todo el ciclo del antirebote
					//-- Antes de dejarlo en el estado inicial se activa la variable deseada.
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, portMAX_DELAY ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							vehiculo.alarma = ON;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
						else
						{
							// No se pudo ingresar al mutex
						}
					}*/
					gpioToggle(LEDR);
				}
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,5/portTICK_RATE_MS);
	}
}

void taskAntirreboteTec4( void* taskParmPtr ){
	//-- TEC4 <=> RESETALARMA
	btnStruct 	tecla4;
	vTaskDelay(3000/portTICK_RATE_MS);
	tecla4.state = UP;
	tecla4.tec = TEC4;
	tecla4.tic = 0;
	tecla4.toc = 0;
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();

	while(1){
		if( tecla4.state == UP ){
			if(xSemaphoreTake(SemBinTec4FallEdge,( TickType_t ) 1) ){ //Sucede un flanco descendente timeout de 1ms
				tecla4.tic = xTaskGetTickCount();
				tecla4.state = FALLING;
			}
		}
		else if( tecla4.state == FALLING ){
			tecla4.toc = xTaskGetTickCount();

			if(tecla4.toc - tecla4.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla4.tec) == 0){
					tecla4.state = DOWN;
				}
			}
		}
		else if( tecla4.state == DOWN ){
			if(xSemaphoreTake(SemBinTec4RiseEdge,( TickType_t ) 1)){ //Sucede un flanco ascendente  timeout de 1ms
				tecla4.tic = xTaskGetTickCount();
				tecla4.state = RAISING;
			}
		}
		else if( tecla4.state == RAISING ){
			tecla4.toc = xTaskGetTickCount();

			if(tecla4.toc - tecla4.tic > ( TickType_t ) 20/portTICK_RATE_MS ){
				if(gpioRead(tecla4.tec) == 1){
					tecla4.state = UP;
					//vehiculo.alarma = OFF;
					/*
					//-- En este momento se cumplio todo el ciclo del antirebote
					//-- Antes de dejarlo en el estado inicial se activa la variable deseada.
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, portMAX_DELAY ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							vehiculo.alarma = OFF;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
						else
						{
							// No se pudo ingresar al mutex
						}
					}*/
					gpioToggle(LEDR);
				}
			}
		}

		vTaskDelayUntil(&tiempo_inicio_ciclo,5/portTICK_RATE_MS);
	}
}

void taskLedVariable( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	uint8_t delay_on =100;

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
      // Intercambia el estado del LEDB
      gpioWrite( LED1, HIGH );

      // Envia la tarea al estado bloqueado duranta el tiempo delay_on
      vTaskDelay( delay_on / portTICK_RATE_MS );
      gpioWrite( LED1, LOW);
      //delay_on += 100;
      //if( delay_on == 1000) delay_on = 0;

      vTaskDelayUntil(&tiempo_inicio_ciclo,(delay_on*2)/ portTICK_RATE_MS);


   }
}

/* Estructura para ingresar a variables compartidas
 *
if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
{
	// --- Mutex lock
	if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
	{
		//--- Lectura o Escritura de variables compartidas

		//--- Mutex unlock
		xSemaphoreGive( mutexSem );
	}
	else
	{
		// No se pudo ingresar al mutex
	}
}
*/

void taskProcessor(void* taskParmPtr){
	//-- Setup tarea
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	mutexSem =  xSemaphoreCreateMutex();
	globalCar localVehiculo;

	if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
	{
		// --- Mutex lock
		if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
		{
			//--- Lectura o Escritura de variables compartidas

			// --- Inicializacion del estado del vehiculo y variables de salida
			vehiculo.start = 0;
			vehiculo.alarma = 0;
			vehiculo.estado = PARADO;
			vehiculo.frenoOutMD = 0;
			vehiculo.frenoOutMI = 0;
			vehiculo.aceleradorOutMD = 0;
			vehiculo.aceleradorOutMI = 0;

			// --- Inicializacion de variables para debug no implementadas en otras tareas
			vehiculo.start = 1;
			vehiculo.tempBMS= 30;
			vehiculo.tempMD=45;
			vehiculo.tempMI=50;

			//--- Mutex unlock
			xSemaphoreGive( mutexSem );
		}
		else
		{
			// No se pudo ingresar al mutex
		}
	}

	//-- Loop tarea
	while(1){

		//--- Leemos variables de entrada
		if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
		{
			// --- Mutex lock
			if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
			{
				//--- Lectura o Escritura de variables compartidas
				localVehiculo.aceleradorIn = vehiculo.aceleradorIn;
				localVehiculo.frenoIn = vehiculo.frenoIn;
				localVehiculo.giroscopoX = vehiculo.giroscopoX;
				localVehiculo.giroscopoY = vehiculo.giroscopoY;
				localVehiculo.giroscopoZ = vehiculo.giroscopoZ;
				localVehiculo.start = vehiculo.start;
				localVehiculo.alarma = vehiculo.alarma;
				localVehiculo.tempMI = vehiculo.tempMI;
				localVehiculo.tempMD = vehiculo.tempMD;
				localVehiculo.tempBMS = vehiculo.tempBMS;

				//--- Mutex unlock
				xSemaphoreGive( mutexSem );
			}
			else
			{
				// No se pudo ingresar al mutex
			}
		}


		if(vehiculo.alarma ==ON){
			//-- Se presiono el boton de parada de emergencia
			//--- debugPrintlnString( "--- ALARMA --- Se presiono el boton de parada de emergencia");
			localVehiculo.estado = ALARMA;
			//--- Nos aseguramos que las salidas queden desactivadas

			localVehiculo.aceleradorOutMD = 0;
			localVehiculo.aceleradorOutMI = 0;
			localVehiculo.frenoOutMD = 0;
			localVehiculo.frenoOutMI = 0;
		}
		else if ((vehiculo.tempBMS > SAFE_TEMP_BMS) || (vehiculo.tempMI > SAFE_TEMP_MI) || (vehiculo.tempMD > SAFE_TEMP_MD)){
			//--- Algun motor o el pack de baterias sobrepaso el limite seguro de temperatura
			localVehiculo.estado = ALARMA;
			//--- Nos aseguramos que las salidas queden desactivadas
			localVehiculo.aceleradorOutMD = 0;
			localVehiculo.aceleradorOutMI = 0;
			localVehiculo.frenoOutMD = 0;
			localVehiculo.frenoOutMI = 0;

			//--- Se indica quien origino la alarma
			if(localVehiculo.tempBMS > SAFE_TEMP_BMS){
				debugPrintlnString( "--- ALARMA --- El pack de baterias sobrepaso la temperatura limite");
			}
			else if(localVehiculo.tempMI > SAFE_TEMP_MI){
				debugPrintlnString( "--- ALARMA --- El motor izquierdo sobrepaso la temperatura limite");
			}
			else{
				debugPrintlnString( "--- ALARMA --- El motor derecho sobrepaso la temperatura limite");
			}
		}
		else{ //--- Ninguna condicion de alarma fue activada
			if(localVehiculo.start ==ON){
				if( (localVehiculo.frenoIn ==0) && (localVehiculo.aceleradorIn ==0) ){
					//--- El vehiculo se encuentra en condiciones de ser manipulado
					localVehiculo.estado = LISTO;
					//debugPrintlnString("Vehiculo LISTO");
					//--- Nos aseguramos que las salidas se encuentran desactivadas
					localVehiculo.aceleradorOutMD = 0;
					localVehiculo.aceleradorOutMI = 0;
					localVehiculo.frenoOutMD = 0;
					localVehiculo.frenoOutMI = 0;
				}
				else if(localVehiculo.frenoIn >0){
					//--- El freno tiene mayor prioridad que el acelerador
					//--- Al momento del frenado se desestima la entrada del acelerador
					localVehiculo.estado = FRENANDO;
					//debugPrintlnString("Vehiculo FRENANDO");
					localVehiculo.aceleradorOutMD = 0;
					localVehiculo.aceleradorOutMI = 0;
					localVehiculo.frenoOutMD = (localVehiculo.frenoIn*3.3)/1024;
					localVehiculo.frenoOutMI = (localVehiculo.frenoIn*3.3)/1024;
				}
				else if(localVehiculo.aceleradorIn > 0){
					localVehiculo.estado = ACELERANDO;
					//debugPrintlnString("Vehiculo ACELERANDO");
					localVehiculo.aceleradorOutMD = (localVehiculo.aceleradorIn * 3.3)/1024;
					localVehiculo.aceleradorOutMI = (localVehiculo.aceleradorIn * 3.3)/1024;
					localVehiculo.frenoOutMD = 0;
					localVehiculo.frenoOutMI = 0;
				}
			}
			else{
				//--- El vehiculo se encuentra parado.
				localVehiculo.estado = PARADO;
				//debugPrintlnString("Vehiculo PARADO");
				//--- Nos aseguramos que las salidas queden desactivadas
				localVehiculo.aceleradorOutMD = 0;
				localVehiculo.aceleradorOutMI = 0;
				localVehiculo.frenoOutMD = 0;
				localVehiculo.frenoOutMI = 0;
			}

		}
		//-- Actualizo las variables globales que modifica el taskProcessor
		if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
		{
			// --- Mutex lock
			if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
			{
				//--- Lectura o Escritura de variables compartidas
				localVehiculo.estado = vehiculo.estado;
				localVehiculo.aceleradorOutMD = vehiculo.aceleradorOutMD;
				localVehiculo.aceleradorOutMI = vehiculo.aceleradorOutMI;
				localVehiculo.frenoOutMD = vehiculo.frenoOutMD;
				localVehiculo.frenoOutMI = vehiculo.frenoOutMI;
				//--- Mutex unlock
				xSemaphoreGive( mutexSem );
			}
			else
			{
				// No se pudo ingresar al mutex
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,10/portTICK_RATE_MS);
	}
}

void taskAceleradorFreno(void* taskParmPtr ){
	//-- Setup tarea
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	adcConfig( ADC_ENABLE ); /* ADC */
	uint16_t acelerador = 0;
	uint16_t freno = 0;
	//vTaskDelay( delayInicial / portTICK_RATE_MS );
	//-- Loop tarea
	while(1){

		acelerador= adcRead( CH1 );
		freno = adcRead( CH2 );

		if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
		{
			// --- Mutex lock
			if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
			{
				//--- Lectura o Escritura de variables compartidas
				vehiculo.aceleradorIn = acelerador;
				vehiculo.frenoIn = freno;
				//--- Mutex unlock
				xSemaphoreGive( mutexSem );
			}
			else
			{
				// No se pudo ingresar al mutex
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,60/portTICK_RATE_MS);
	}


}

void taskGiroscopo(void* taskParmPtr ){
	//-- Setup tarea
	int8_t status;
	char buffer[50];

	float giroX;
	float giroY;
	float giroZ;

	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	//-- Configuracion de la direccion I2C
	MPU9250_address_t addr = 0x68; // If MPU9250 AD0 pin is connected to GND
	status = mpu9250Init_RTOS( addr );
	//debugPrintInt(status);
	if( status < 0 ){

	      while(1){
	    	  gpioToggle(LED1);
	    	  gpioToggle(LED2);
	    	  gpioToggle(LED3);
	    	  vTaskDelayUntil(&tiempo_inicio_ciclo,1000/portTICK_RATE_MS);
	      }
	   }

	//-- Loop tarea
	while(1){

		//Leer el sensor y guardar en estructura de control
		mpu9250Read_RTOS();

		giroX = mpu9250GetGyroX_rads_RTOS()*(180/3.14159);
		giroY = mpu9250GetGyroY_rads_RTOS()*(180/3.14159);
		giroZ = mpu9250GetGyroZ_rads_RTOS()*(180/3.14159);
		if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
		{
			// --- Mutex lock
			if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
			{
				//--- Lectura o Escritura de variables compartidas
				vehiculo.giroscopoX = giroX;
				vehiculo.giroscopoY = giroY;
				vehiculo.giroscopoZ = giroZ;
				//--- Mutex unlock
				xSemaphoreGive( mutexSem );
			}
			else
			{
				// No se pudo ingresar al mutex
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,80/portTICK_RATE_MS);
	}
}

void taskEnvioDatos(void* taskParmPtr ){
	//-- Setup tarea
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	globalCar localVehiculo;
	char strEnvio[30];
	char strDato[15];
	char* ptrStrEnvio;
	int i;
	BaseType_t returnValue;
	//-- Se crea una cola de mensajes para enviar un puntero al string generado
	//-- con esto se reduce en gran medida el tamanio de la cola generada
	//-- debe tenerse especial cuidado de que la variable no cambie de valor hasta
	//-- que sea leida por el otro proceso.
	colaMsg = xQueueCreate(1,sizeof(char*));

	//-- Loop tarea
	while(1){

		// ---- Se envía un dato por vez para que la tarea no se quede con el control del cpu por mucho tiempo
		// ---- Los nombres que se asigna a cada variable es el nombre actual de cada variable
		for (i=1 ; i<=15 ;i++){
			switch(i){
				case 1:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.aceleradorIn = vehiculo.aceleradorIn;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del aceleradorIn (int)
					strcpy(strEnvio, "aceleradorIn:" );
					strcpy(strDato, integerToString(localVehiculo.aceleradorIn,strDato,10));
					break;
				case 2:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.aceleradorOutMD = vehiculo.aceleradorOutMD;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío estado del aceleradorOutMD (float)
					strcpy(strEnvio, "aceleradorOutMD:" );
					strcpy(strDato ,floatToString(localVehiculo.aceleradorOutMD,strDato));
					break;
				case 3:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.aceleradorOutMI = vehiculo.aceleradorOutMI;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío estado del aceleradorOutMI (float)
					strcpy(strEnvio, "aceleradorOutMI:" );
					strcpy(strDato ,floatToString(localVehiculo.aceleradorOutMI,strDato));
					break;
				case 4:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.alarma = vehiculo.alarma;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado de la alarma
					strcpy(strEnvio, "alarma:" );
					strcpy(strDato ,integerToString(localVehiculo.alarma,strDato,10));
					break;
				case 5:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.estado = vehiculo.estado;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del vehiculo
					strcpy(strEnvio, "estado:" );
					strcpy(strDato ,integerToString(localVehiculo.estado,strDato,10));
					break;
				case 6:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.frenoIn = vehiculo.frenoIn;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío estado del frenoIn (int)
					strcpy(strEnvio, "frenoIn:" );
					strcpy(strDato ,integerToString(localVehiculo.frenoIn,strDato,10));
					break;
				case 7:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.frenoOutMD = vehiculo.frenoOutMD;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del frenoOutMD (float)
					strcpy(strEnvio, "frenoOutMD:" );
					strcpy(strDato ,floatToString(localVehiculo.frenoOutMD,strDato));
					break;
				case 8:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.frenoOutMI = vehiculo.frenoOutMI;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del frenoOutMI (float)
					strcpy(strEnvio, "frenoOutMI:" );
					strcpy(strDato ,floatToString(localVehiculo.frenoOutMI,strDato));
					break;
				case 9:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.giroscopoX = vehiculo.giroscopoX;;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del giroscopoX (float)
					strcpy(strEnvio, "giroscopoX:" );
					strcpy(strDato ,floatToString(localVehiculo.giroscopoX,strDato));
					break;
				case 10:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.giroscopoY = vehiculo.giroscopoY;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del giroscopoY (float)
					strcpy(strEnvio, "giroscopoY:" );
					strcpy(strDato ,floatToString(localVehiculo.giroscopoY,strDato));
					break;
				case 11:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.giroscopoZ = vehiculo.giroscopoZ;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}
					//-- Envío estado del giroscopoZ (float)
					strcpy(strEnvio, "giroscopoZ:" );
					strcpy(strDato ,floatToString(localVehiculo.giroscopoZ,strDato));
					break;
				case 12:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.start = vehiculo.start;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío estado de la condicion de start (int)
					strcpy(strEnvio, "start:" );
					strcpy(strDato ,integerToString(localVehiculo.start,strDato,10));
					break;
				case 13:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.tempBMS = vehiculo.tempBMS;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío temperatura del BMS (float)
					strcpy(strEnvio, "tempBMS:" );
					strcpy(strDato ,floatToString(localVehiculo.tempBMS,strDato));
					break;
				case 14:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.tempMD = vehiculo.tempMD;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío temperatura del MD (float)
					strcpy(strEnvio, "tempMD:" );
					strcpy(strDato ,floatToString(localVehiculo.tempMD,strDato));
					break;
				case 15:
					if( mutexSem!= NULL ) //-- Si ya se encuentra creado el mutex
					{
						// --- Mutex lock
						if( xSemaphoreTake( mutexSem, ( TickType_t ) 10 ) == pdTRUE )
						{
							//--- Lectura o Escritura de variables compartidas
							localVehiculo.tempMI = vehiculo.tempMI;
							//--- Mutex unlock
							xSemaphoreGive( mutexSem );
						}
					}

					//-- Envío temperatura del MI (float)
					strcpy(strEnvio, "tempMI:" );
					strcpy(strDato ,floatToString(localVehiculo.tempMI,strDato));
					break;
				default:
					//-- Do nothing
					break;
			}
			strcat(strEnvio, strDato);
			strcat(strEnvio, "\r\n");

			ptrStrEnvio = strEnvio;
			returnValue = xQueueSend(colaMsg, &ptrStrEnvio,portMAX_DELAY);
			if(returnValue){
				//gpioWrite(LEDB,ON);
			}
			else{
				//gpioWrite(LED2,ON);
			}
			vTaskDelayUntil(&tiempo_inicio_ciclo,20/portTICK_RATE_MS);
		}

	}
}

void taskUartConnection( void* taskParmPtr ){
	//-- Setup tarea
	char* ptrStrRecibido;
	char StrRecibido[30];
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	uartConfig(UART_GPIO , 9600);
	uartWriteString(UART_GPIO ,"Scheduler inicializado 11\r\n");
	BaseType_t returnValue;
	//-- Loop tarea
	while(1){
		returnValue = xQueueReceive(colaMsg,&ptrStrRecibido,(TickType_t) 500 );
		strcpy(StrRecibido,ptrStrRecibido);
		if(returnValue){
			uartWriteString(UART_GPIO ,StrRecibido);
			//gpioWrite(LEDR,ON);
		}
		else{
			uartWriteString(UART_GPIO ,"Nada recibido desde la cola");
			//gpioWrite(LED3,ON);
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,250/portTICK_RATE_MS);
	}
}

/*==================[fin del archivo]========================================*/
