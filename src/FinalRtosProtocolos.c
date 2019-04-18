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

debounceData_t datosAntirrebote[4]; //Variable que guarda el estado del antirrebote
uint8_t delayInicial = 30;

//Sincronizacion y proteccion de datos
xSemaphoreHandle SemBin[4];

uint16_t acelerador = 0;
uint16_t freno = 0;
/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void taskLedVariable( void* taskParmPtr );
void taskMefAntirrebote( void* taskParmPtr );
void taskSenialTecla1( void* taskParmPtr );
void taskSenialTecla2( void* taskParmPtr );
void taskSenialTecla3( void* taskParmPtr );
void taskSenialTecla4( void* taskParmPtr );

//--- Tareas propias del vehiculo
void taskProcessor(void* taskParmPtr);
void taskAceleradorFreno(void* taskParmPtr );
//void taskEnvioDatos(void* taskParmPtr );
void taskGiroscopo(void* taskParmPtr);
void taskUartConnection( void* taskParmPtr );
/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );

   // Crear tarea en freeRTOS
   xTaskCreate(
		   taskLedVariable,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskLedVariable",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
   );

   xTaskCreate(
		   taskMefAntirrebote,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskMefAntirrebote",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
    	);
   xTaskCreate(
		   taskSenialTecla1,                     // Funcion de la tarea a ejecutar
   		   (const char *)"taskSenialTecla1",     // Nombre de la tarea como String amigable para el usuario
   		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
   		   0,                          // Parametros de tarea
   		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
   		   0                           // Puntero a la tarea creada en el sistema
    	);
   xTaskCreate(
   		   taskSenialTecla2,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskSenialTecla2",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
   	   );
   xTaskCreate(
   		   taskSenialTecla3,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskSenialTecla3",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
    	);
   xTaskCreate(
   		   taskSenialTecla4,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskSenialTecla4",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
    	);

   xTaskCreate(
		   taskProcessor,                     // Funcion de la tarea a ejecutar
   		   (const char *)"taskProcessor",     // Nombre de la tarea como String amigable para el usuario
   		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
   		   0,                          // Parametros de tarea
   		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
   		   0                           // Puntero a la tarea creada en el sistema
       	);
   xTaskCreate(
		   taskAceleradorFreno,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskAceleradorFreno",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
    	);
   /*xTaskCreate(
		   taskEnvioDatos,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskEnvioDatos",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
    	);*/
   xTaskCreate(
		   taskGiroscopo,                     // Funcion de la tarea a ejecutar
		   (const char *)"taskGiroscopo",     // Nombre de la tarea como String amigable para el usuario
		   configMINIMAL_STACK_SIZE*4, // Cantidad de stack de la tarea
		   0,                          // Parametros de tarea
		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
		   0                           // Puntero a la tarea creada en el sistema
    	);
   /*xTaskCreate(
		   taskUartConnection,                     // Funcion de la tarea a ejecutar
   		   (const char *)"taskUartConnection",     // Nombre de la tarea como String amigable para el usuario
   		   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
   		   0,                          // Parametros de tarea
   		   tskIDLE_PRIORITY+1,         // Prioridad de la tarea
   		   0                           // Puntero a la tarea creada en el sistema
       	);*/

   // Iniciar scheduler
   debugPrintlnString( "Iniciando scheduler" );
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// ----------------- CON vTaskDelayUntil----------------------------
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

void taskMefAntirrebote( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	int index = 0; //Recorre y analiza el estado de cada tecla
	uint8_t giveRealizado[4]; //Avisa que tarea fue liberada para poder tomarla nuevamente
	// --- Inicializando datos antirrebote
	datosAntirrebote[0].tecla = TEC1;
	datosAntirrebote[0].state = BUTTON_UP;
	datosAntirrebote[0].delay = 50;

	datosAntirrebote[1].tecla = TEC2;
	datosAntirrebote[1].state = BUTTON_UP;
	datosAntirrebote[1].delay = 50;

	datosAntirrebote[2].tecla = TEC3;
	datosAntirrebote[2].state = BUTTON_UP;
	datosAntirrebote[2].delay = 50;

	datosAntirrebote[3].tecla = TEC4;
	datosAntirrebote[3].state = BUTTON_UP;
	datosAntirrebote[3].delay = 50;

	//debugPrintlnString( "Creando semaforo...\r\n" );
	SemBin[0] = xSemaphoreCreateBinary();
	SemBin[1] = xSemaphoreCreateBinary();
	SemBin[2] = xSemaphoreCreateBinary();
	SemBin[3] = xSemaphoreCreateBinary();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
	   switch(datosAntirrebote[index].state){
	   	   case BUTTON_UP:
	   		   	   //-- Si la tecla es presionada se pasa al estado BUTTON_FALLING
	   		   	   if(!gpioRead(datosAntirrebote[index].tecla)){
	   		   		   //debugPrintlnString( "Se detecto tecla presionada/r/n" );
	   		   		   datosAntirrebote[index].state = BUTTON_FALLING;
	   		   	   }
	   		   	   break;

	   	   case BUTTON_FALLING:
	   		   	   //-- Bloqueamos el estado hasta que se cumpla el tiempo del antirebote
	   		   	   vTaskDelay( datosAntirrebote[index].delay / portTICK_RATE_MS );

	   		   	   if(!gpioRead(datosAntirrebote[index].tecla)) {
	   		   		   //-- El BOTON SIGUE PRESIONADO --
	   		   		   //-- Se guarda el tiempo inicial de tecla 1
	   		   		   datosAntirrebote[index].tiempo_inicio_ciclo = xTaskGetTickCount();

	   		   		   //-- Se pasa al estado DOWN
	   		   		   datosAntirrebote[index].state = BUTTON_DOWN;
	   		   	   }
	   		   	   else{
	   		   		   //-- El BOTON NO ESTA PRESIONADO --
	   		   		   //-- Se vuelve al estado UP
	   		   		   datosAntirrebote[index].state = BUTTON_UP;
	   		   	   }

	   		   	   break;

	   	   case BUTTON_DOWN:
	   		   	   //-- Se encuentra en este estado hasta que se deje de presionar la tecla
	   		   	   if(gpioRead(datosAntirrebote[index].tecla))
	   		   		   datosAntirrebote[index].state = BUTTON_RAISING;
	   		   	   break;

	   	   case BUTTON_RAISING:

	   		   	   if(gpioRead(datosAntirrebote[index].tecla)) {
	   		   		   //-- EL BOTON ACABA DE SER SOLTADO --
	   		   		   //-- el delay se utiliza si el antirebote es por flanco ascendente
	   		   		   //-- vTaskDelay( delayAntirrebote / portTICK_RATE_MS );

	   		   		   //Cuando se suelta el boton termino de contar el tiempo
					   datosAntirrebote[index].tiempo_presionado = xTaskGetTickCount() - datosAntirrebote[index].tiempo_inicio_ciclo;
					   xSemaphoreGive(SemBin[index]);
					   giveRealizado[index] = 1;
					   //vTaskDelay( 5/ portTICK_RATE_MS );
					   //xSemaphoreTake(SemBin[index],portMAX_DELAY);

					   datosAntirrebote[index].state = BUTTON_UP;
	   		   	   }
	   		   	   else
	   		   		datosAntirrebote[index].state = BUTTON_DOWN;
	   		   	   break;

	   	   default:
	   		   	   debugPrintlnString( "MEF Default\r\n" );
	   		   	   break;
	   }
	   index++;
	   if (index ==4){
		   index =0;
		   int i;
		   for(i=0;i<4;i++){
			   if(giveRealizado[i] ==1){
				   vTaskDelay( 1/ portTICK_RATE_MS );
				   xSemaphoreTake(SemBin[i],portMAX_DELAY);
				   giveRealizado[i] = 0;
			   }
		   }


/*
		   xSemaphoreTake(SemBin[1],portMAX_DELAY);
		   xSemaphoreTake(SemBin[2],portMAX_DELAY);
		   xSemaphoreTake(SemBin[3],portMAX_DELAY);*/
	   }
   }
}

void taskSenialTecla1( void* taskParmPtr )
{
      // ---------- CONFIGURACIONES ------------------------------
	   gpioWrite( LEDG, LOW);
	   vTaskDelay( delayInicial / portTICK_RATE_MS );
      // ---------- REPETIR POR SIEMPRE --------------------------
      while(TRUE) {
    	  //debugPrintlnString( "Tarea led retardado\r\n" );
          // Intercambia el estado del LEDB
    	  if( xSemaphoreTake(SemBin[0] ,portMAX_DELAY ) == pdTRUE){
    		  //debugPrintlnString( "Se toma el semaforo por parte del LED!!\r\n" );
    		  gpioWrite( LEDG, HIGH );
    	  	  //debugPrintlnString( "Blink!" );
    	  	  // Envia la tarea al estado bloqueado durante 500ms
    	  	  vTaskDelay( datosAntirrebote[0].tiempo_presionado / portTICK_RATE_MS );
    	  	  gpioWrite( LEDG, LOW);
    	  	  debugPrintlnString( "Tiempo encendido:  ");
    	  	  debugPrintlnInt( datosAntirrebote[0].tiempo_presionado );
    	  	  //debugPrintlnString( "\r\nTiempo tics\n");
    	  	  //debugPrintlnInt( xTaskGetTickCount() );

    	  	  xSemaphoreGive(SemBin[0]);
    	  	  vTaskDelay( 10 / portTICK_RATE_MS );
    	  }
    	  else
    		  vTaskDelay( 50 / portTICK_RATE_MS );

      }

}
void taskSenialTecla2( void* taskParmPtr )
{
      // ---------- CONFIGURACIONES ------------------------------
	   gpioWrite( LEDB, LOW);
	   vTaskDelay( delayInicial / portTICK_RATE_MS );
      // ---------- REPETIR POR SIEMPRE --------------------------
      while(TRUE) {
    	  //debugPrintlnString( "Tarea led retardado\r\n" );
          // Intercambia el estado del LEDB
    	  if( xSemaphoreTake(SemBin[1] ,portMAX_DELAY ) == pdTRUE){
    		  //debugPrintlnString( "Se toma el semaforo por parte del LED!!\r\n" );
    		  gpioWrite( LEDB, HIGH );
    	  	  //debugPrintlnString( "Blink!" );
    	  	  // Envia la tarea al estado bloqueado durante 500ms
    	  	  vTaskDelay( datosAntirrebote[1].tiempo_presionado / portTICK_RATE_MS );
    	  	  gpioWrite( LEDB, LOW);
    	  	  debugPrintlnString( "Tiempo encendido:  ");
    	  	  debugPrintlnInt( datosAntirrebote[1].tiempo_presionado );
    	  	  //debugPrintlnString( "\r\nTiempo tics\n");
    	  	  //debugPrintlnInt( xTaskGetTickCount() );

    	  	  xSemaphoreGive(SemBin[1]);
    	  	  vTaskDelay( 10 / portTICK_RATE_MS );
    	  }
    	  else
    		  vTaskDelay( 50 / portTICK_RATE_MS );

      }

}
void taskSenialTecla3( void* taskParmPtr )
{
      // ---------- CONFIGURACIONES ------------------------------
	   gpioWrite( LEDR, LOW);
	   vTaskDelay( delayInicial / portTICK_RATE_MS );
      // ---------- REPETIR POR SIEMPRE --------------------------
      while(TRUE) {
    	  //debugPrintlnString( "Tarea led retardado\r\n" );
          // Intercambia el estado del LEDB
    	  if( xSemaphoreTake(SemBin[2] ,portMAX_DELAY ) == pdTRUE){
    		  //debugPrintlnString( "Se toma el semaforo por parte del LED!!\r\n" );
    		  gpioWrite( LEDR, HIGH );
    	  	  //debugPrintlnString( "Blink!" );
    	  	  // Envia la tarea al estado bloqueado durante 500ms
    	  	  vTaskDelay( datosAntirrebote[2].tiempo_presionado / portTICK_RATE_MS );
    	  	  gpioWrite( LEDR, LOW);
    	  	  debugPrintlnString( "Tiempo encendido:  ");
    	  	  debugPrintlnInt( datosAntirrebote[2].tiempo_presionado );
    	  	  //debugPrintlnString( "\r\nTiempo tics\n");
    	  	  //debugPrintlnInt( xTaskGetTickCount() );

    	  	  xSemaphoreGive(SemBin[2]);
    	  	  vTaskDelay( 10 / portTICK_RATE_MS );
    	  }
    	  else
    		  vTaskDelay( 50 / portTICK_RATE_MS );

      }

}
void taskSenialTecla4( void* taskParmPtr )
{
      // ---------- CONFIGURACIONES ------------------------------
	   gpioWrite( LED2, LOW);
	   vTaskDelay( delayInicial / portTICK_RATE_MS );
      // ---------- REPETIR POR SIEMPRE --------------------------
      while(TRUE) {
    	  //debugPrintlnString( "Tarea led retardado\r\n" );
          // Intercambia el estado del LEDB
    	  if( xSemaphoreTake(SemBin[3] ,portMAX_DELAY ) == pdTRUE){
    		  //debugPrintlnString( "Se toma el semaforo por parte del LED!!\r\n" );
    		  gpioWrite( LED2, HIGH );
    	  	  //debugPrintlnString( "Blink!" );
    	  	  // Envia la tarea al estado bloqueado durante 500ms
    	  	  vTaskDelay( datosAntirrebote[3].tiempo_presionado / portTICK_RATE_MS );
    	  	  gpioWrite( LED2, LOW);
    	  	  debugPrintlnString( "Tiempo encendido:  ");
    	  	  debugPrintlnInt( datosAntirrebote[3].tiempo_presionado );
    	  	  //debugPrintlnString( "\r\nTiempo tics\n");
    	  	  //debugPrintlnInt( xTaskGetTickCount() );

    	  	  xSemaphoreGive(SemBin[3]);
    	  	  vTaskDelay( 10 / portTICK_RATE_MS );
    	  }
    	  else
    		  vTaskDelay( 50 / portTICK_RATE_MS );

      }

}
void taskProcessor(void* taskParmPtr){
	//-- Setup tarea
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
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

	//vTaskDelay( delayInicial*3 / portTICK_RATE_MS );

	//-- Loop tarea
	while(1){
		// ---
		// ---REEMPLAZAR TODAS las variables globales por variables locales utilizando mutexes
		// ---

		// ---
		// --- REEMPLAZAR TODOS los "debugPrintlnString" por una cola de mensajes
		// --- y una tarea que envie el estado por la UART_USB
		// ---



		if(vehiculo.alarma ==ON){
			//-- Se presiono el boton de parada de emergencia
			debugPrintlnString( "--- ALARMA --- Se presiono el boton de parada de emergencia");
			vehiculo.estado = ALARMA;
			//--- Nos aseguramos que las salidas queden desactivadas
			vehiculo.aceleradorOutMD = 0;
			vehiculo.aceleradorOutMI = 0;
			vehiculo.frenoOutMD = 0;
			vehiculo.frenoOutMI = 0;
		}
		else if ((vehiculo.tempBMS > SAFE_TEMP_BMS) || (vehiculo.tempMI > SAFE_TEMP_MI) || (vehiculo.tempMD > SAFE_TEMP_MD)){
			//--- Algun motor o el pack de baterias sobrepaso el limite seguro de temperatura
			vehiculo.estado = ALARMA;
			//--- Nos aseguramos que las salidas queden desactivadas
			vehiculo.aceleradorOutMD = 0;
			vehiculo.aceleradorOutMI = 0;
			vehiculo.frenoOutMD = 0;
			vehiculo.frenoOutMI = 0;

			//--- Se indica quien origino la alarma
			if(vehiculo.tempBMS > SAFE_TEMP_BMS){
				debugPrintlnString( "--- ALARMA --- El pack de baterias sobrepaso la temperatura limite");
			}
			else if(vehiculo.tempMI > SAFE_TEMP_MI){
				debugPrintlnString( "--- ALARMA --- El motor izquierdo sobrepaso la temperatura limite");
			}
			else{
				debugPrintlnString( "--- ALARMA --- El motor derecho sobrepaso la temperatura limite");
			}
		}
		else{ //--- Ninguna condicion de alarma fue activada
			if(vehiculo.start ==ON){
				if( (vehiculo.frenoIn ==0) && (vehiculo.aceleradorIn ==0) ){
					//--- El vehiculo se encuentra en condiciones de ser manipulado
					vehiculo.estado = LISTO;
					debugPrintlnString("Vehiculo LISTO");
					//--- Nos aseguramos que las salidas se encuentran desactivadas
					vehiculo.aceleradorOutMD = 0;
					vehiculo.aceleradorOutMI = 0;
					vehiculo.frenoOutMD = 0;
					vehiculo.frenoOutMI = 0;
				}
				else if(vehiculo.frenoIn >0){
					//--- El freno tiene mayor prioridad que el acelerador
					//--- Al momento del frenado se desestima la entrada del acelerador
					vehiculo.estado = FRENANDO;
					debugPrintlnString("Vehiculo FRENANDO");
					vehiculo.aceleradorOutMD = 0;
					vehiculo.aceleradorOutMI = 0;
					vehiculo.frenoOutMD = vehiculo.frenoIn;
					vehiculo.frenoOutMI = vehiculo.frenoIn;
				}
				else if(vehiculo.aceleradorIn > 0){
					vehiculo.estado = ACELERANDO;
					debugPrintlnString("Vehiculo ACELERANDO");
					vehiculo.aceleradorOutMD = vehiculo.aceleradorIn;
					vehiculo.aceleradorOutMI = vehiculo.aceleradorIn;
					vehiculo.frenoOutMD = 0;
					vehiculo.frenoOutMI = 0;
				}
			}
			else{
				//--- El vehiculo se encuentra parado.
				vehiculo.estado = PARADO;
				debugPrintlnString("Vehiculo PARADO");
				//--- Nos aseguramos que las salidas queden desactivadas
				vehiculo.aceleradorOutMD = 0;
				vehiculo.aceleradorOutMI = 0;
				vehiculo.frenoOutMD = 0;
				vehiculo.frenoOutMI = 0;
			}
		}
		vTaskDelayUntil(&tiempo_inicio_ciclo,10/portTICK_RATE_MS);
	}
}
void taskAceleradorFreno(void* taskParmPtr ){
	//-- Setup tarea
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	adcConfig( ADC_ENABLE ); /* ADC */
	//vTaskDelay( delayInicial / portTICK_RATE_MS );
	//-- Loop tarea
	while(1){
		vehiculo.aceleradorIn = adcRead( CH1 );
		acelerador = (vehiculo.aceleradorIn*100) /1024; 	//--- Porcentaje aceleracion
		vehiculo.frenoIn = adcRead( CH2 );
		freno = (adcRead( vehiculo.frenoIn )*100) /1024 ;	//--- Porcentaje frenado

		debugPrintlnString( "Acelerador:  ");
		debugPrintlnInt( acelerador );
		debugPrintlnString( "Freno:  ");
		debugPrintlnInt( freno);
		vTaskDelayUntil(&tiempo_inicio_ciclo,60/portTICK_RATE_MS);
	}


}
/*void taskEnvioDatos(void* taskParmPtr ){
	//-- Setup tarea

	//-- Loop tarea
	while(1){

	}
}*/
void taskGiroscopo(void* taskParmPtr ){

	//-- Setup tarea

	int8_t status;
	char buffer[50];
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	//-- Configuracion de la direccion I2C
	MPU9250_address_t addr = 0x68; // If MPU9250 AD0 pin is connected to GND
	status = mpu9250Init_RTOS( addr );
	debugPrintInt(status);
	if( status < 0 ){

	      while(1){
	    	  debugPrintlnString( "Error al inicializar el modulo MPU9250");
	    	  vTaskDelayUntil(&tiempo_inicio_ciclo,1000/portTICK_RATE_MS);
	      }
	   }

	//-- Loop tarea
	while(1){

		//Leer el sensor y guardar en estructura de control
		mpu9250Read_RTOS();

		// Imprimir resultados
		debugPrintlnString("Giroscopo: ");
		debugPrintString("   X: ");
		debugPrintInt(mpu9250GetGyroX_rads_RTOS()*(180/3.14159));
		debugPrintString("   Y: ");
		debugPrintInt(mpu9250GetGyroY_rads_RTOS()*(180/3.14159));
		debugPrintString("   Z: ");
		debugPrintInt(mpu9250GetGyroZ_rads_RTOS()*(180/3.14159));
		debugPrintlnString("");
		vTaskDelayUntil(&tiempo_inicio_ciclo,80/portTICK_RATE_MS);
	}
}

void taskUartConnection( void* taskParmPtr ){
	TickType_t tiempo_inicio_ciclo = xTaskGetTickCount();
	vTaskDelayUntil(&tiempo_inicio_ciclo,1000/portTICK_RATE_MS);
	/*//-- Setup tarea
	uartConfig( UART_GPIO, 9600 );
	//-- Loop tarea
	while(1){
		uartWriteString( UART_GPIO, "\r\nGo Go Go!\r\n" );

	}*/
}
/*==================[fin del archivo]========================================*/
