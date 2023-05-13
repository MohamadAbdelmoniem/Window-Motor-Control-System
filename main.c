/*
Project: Window Motor Control System
Description: This project implements a task-based system for controlling window motors in a vehicle.
It utilizes FreeRTOS to manage concurrent execution of multiple tasks, synchronization between tasks using semaphores, 
and GPIO interactions for controlling the motors.

Author: Team 8
Date: 12/5/2023

--- Project Structure ---
- main.c: Contains the main function and task creation
- tasks.c: Defines the task functions for window control, obstacle detection, and system control
- gpio.c: Implements GPIO handling functions
- delay.c: Provides a delay function for introducing delays in milliseconds

--- Tasks ---
1. initTask: Initializes the system and creates necessary semaphores and mutexes.
2. WindowUp_Driver: Controls the upward movement of the driver-side window.
3. WindowDown_Driver: Controls the downward movement of the driver-side window.
4. WindowUp_Passenger: Controls the upward movement of the passenger-side window.
5. WindowDown_Passenger: Controls the downward movement of the passenger-side window.
6. OnOff: Monitors the On/Off state of the system for the passenger control.
7. Obstacle: Handles obstacle detection and performs necessary actions.

--- Semaphores and Mutexes ---
- WindowUp_driverS: Semaphore for signaling the WindowUp_Driver task.
- WindowDown_driverS: Semaphore for signaling the WindowDown_Driver task.
- WindowUp_passengerS: Semaphore for signaling the WindowUp_Passenger task.
- WindowDown_passengerS: Semaphore for signaling the WindowDown_Passenger task.
- ObstacleS: Semaphore for signaling the Obstacle task.
- OnOffS: Semaphore for signaling the OnOff task.
- MotorMutex: Mutex for controlling access to the motor.

--- GPIO Interaction ---
- GPIOA_Handler: Interrupt handler for GPIO Port A.
- GPIOF_Handler: Interrupt handler for GPIO Port F.
- GPIO interactions are used to read inputs for obstacle detection and control the window motors.

Note: Make sure to configure the necessary hardware peripherals and pin assignments as required.
*/
#include "FreeRTOS.h"
#include "tm4c123gh6pm.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "semphr.h"
#include "driverlib/gpio.h"
#include "Team8Macro.h"

SemaphoreHandle_t WindowUp_driverS;
SemaphoreHandle_t WindowDown_driverS;

SemaphoreHandle_t WindowUp_passengerS;
SemaphoreHandle_t WindowDown_passengerS;
SemaphoreHandle_t ObstacleS;
SemaphoreHandle_t OnOffS;

SemaphoreHandle_t MotorMutex;			// Mutex for Motor

void initTask(void *params);
void WindowUp_Driver (void *params);
void WindowDown_Driver (void *params);
void OnOff (void *params);
void WindowUp_Passenger(void *params);
void WindowDown_Passenger(void *params);
void Obstacle ();


void GPIOA_Handler();
void GPIOF_Handler();
void GPIOC_Handler();

xQueueHandle xQueue;


void delayMs(uint32_t n)
{
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++)
    {}
  }
}


int main()
{
	WindowUp_driverS = xSemaphoreCreateBinary();
	WindowDown_driverS = xSemaphoreCreateBinary();
	OnOffS = xSemaphoreCreateBinary();
	WindowUp_passengerS = xSemaphoreCreateBinary();
	WindowDown_passengerS = xSemaphoreCreateBinary();
	ObstacleS = xSemaphoreCreateBinary();
	
	MotorMutex = xSemaphoreCreateMutex();
	
	
	xTaskCreate(initTask, "init", 80, NULL, 4, NULL);
	xTaskCreate(WindowUp_Driver, "WindowUp_Driver", 80, NULL, 2, NULL);
	xTaskCreate(WindowDown_Driver, "WindowDown_Driver", 80, NULL, 2, NULL);
	xTaskCreate(WindowUp_Passenger, "WindowUp_Passenger", 80, NULL, 1, NULL);
	xTaskCreate(WindowDown_Passenger, "WindowDown_Passenger", 80, NULL, 1, NULL);
	xTaskCreate(OnOff, "OnOff", 80, NULL, 2, NULL);
	
	xQueue = xQueueCreate( 1, sizeof(int));

	vTaskStartScheduler();
	while (1)
	{}
}

void initTask(void *params)
{
	for (;;)
	{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
			;

		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3);
		GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3);
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);

		GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_Handler);
		IntPrioritySet(INT_GPIOA, 0XE0);
		
		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 );
		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

		
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
		GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		
		GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);
		GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		
		GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_5);
		GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
		
		GPIOIntRegister(GPIO_PORTC_BASE, GPIOC_Handler);
		IntPrioritySet(INT_GPIOC, 0XE0);
		
			GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4 );
		GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
		
		GPIOIntRegister(GPIO_PORTF_BASE, GPIOF_Handler);
		IntPrioritySet(INT_GPIOF, 0XE0);


		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
		GPIOUnlockPin(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
		vTaskSuspend(NULL);
	}
}

void GPIOA_Handler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2))
	{
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
		xSemaphoreGiveFromISR(WindowUp_driverS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	else if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
		xSemaphoreGiveFromISR(WindowDown_driverS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		
	}
	else if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
		xSemaphoreGiveFromISR(WindowUp_passengerS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	else if (!GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7))
	{
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
		xSemaphoreGiveFromISR(WindowDown_passengerS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	else{
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_3);
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_7);
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_6);
	}
}




void GPIOC_Handler()
{
			GPIOIntClear(GPIO_PORTC_BASE, GPIO_PIN_5);
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			int flag=1;
			xQueueSendFromISR(xQueue,&flag,&xHigherPriorityTaskWoken);
}

void GPIOF_Handler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if (!GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4))
	{
			GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
		xSemaphoreGiveFromISR(OnOffS, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	else{
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);

	}
}


void WindowUp_Driver(void *params)
{
	int out=0;
	xSemaphoreTake(WindowUp_driverS, 0);
	while (1)
	{
		xSemaphoreTake(WindowUp_driverS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
	if (xQueueReceive( xQueue, &out, 0 )== pdFALSE)
	{
			if (!GPIOPinRead(BUTTONS_PORT, WINDOW_UP_DRIVER_PIN))
		{
			delayMs(50);	
			Move_Up;
			delayMs(1000);
		if(GPIOPinRead(BUTTONS_PORT, WINDOW_UP_DRIVER_PIN))
		{
			//automatic		
			while(GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_6))
			{
				if (xQueueReceive( xQueue, &out, 0 )== pdTRUE)
				{
					if(out==1)
						Obstacle();
					break;
				}
				if (!GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_DRIVER_PIN) || !GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_PASSENGER_PIN))
				{
					break;
				}
			}
		}
		//MANUAL
			else
				{
					while (!(GPIOPinRead(BUTTONS_PORT, WINDOW_UP_DRIVER_PIN)) && GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_6))
					{
						if (xQueueReceive( xQueue, &out, 0 )== pdTRUE)
				{
					if(out==1)
						Obstacle();
					break;
				}
						
					}
						Stop_Motor;
			}
		}
	}
	else
	{
		__WFI();
	}

		Stop_Motor;
		out=0;
		xSemaphoreGive(MotorMutex);
	}

}


void WindowDown_Driver(void *params)
{
	xSemaphoreTake(WindowDown_driverS, 0);
	while (1)
	{
		xSemaphoreTake(WindowDown_driverS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
			if (!GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_DRIVER_PIN))
		{

		delayMs(50);
		Move_Down;
		delayMs(1000);
		if(GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_DRIVER_PIN))
		{
			//automatic
			while(GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_7))
			{
				if (!GPIOPinRead(BUTTONS_PORT, GPIO_PIN_2) || !GPIOPinRead(BUTTONS_PORT, GPIO_PIN_6))
				{
					break;
				}
				
			}
		}
			else
				//manual
				{			
					while (!(GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_DRIVER_PIN)) && GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_7))
					{
						
					}
						Stop_Motor;
			}
	}		
	Stop_Motor;
	xSemaphoreGive(MotorMutex);
	
}
	}

void WindowUp_Passenger(void *params)
{
	int out=0;
	xSemaphoreTake(WindowUp_passengerS, 0);
	while (1)
	{
		xSemaphoreTake(WindowUp_passengerS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		
	if (xQueueReceive( xQueue, &out, 0 )== pdFALSE)
	{
		if (!GPIOPinRead(BUTTONS_PORT, WINDOW_UP_PASSENGER_PIN))
		{
			delayMs(50);
		
		Move_Up;
		delayMs(1000);
		if(GPIOPinRead(BUTTONS_PORT, WINDOW_UP_PASSENGER_PIN))
		{
			
			//automatic
			while(GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_6))
			{
				if (xQueueReceive( xQueue, &out, 0 )== pdTRUE)
				{
					if(out==1)
						Obstacle();
					break;
				}
				if (!GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_DRIVER_PIN) || !GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_PASSENGER_PIN))
				{
					break;
				}
				}
		
			}
		
			else 
				{
					while (!(GPIOPinRead(BUTTONS_PORT, WINDOW_UP_PASSENGER_PIN)) && GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_6) )
					{
						if (xQueueReceive( xQueue, &out, 0 )== pdTRUE)
				{
					if(out==1)
						Obstacle();
					break;
				}
						
					}
						Stop_Motor;
			}
		}
	}
	else
		{
			__WFI();
	}
		out=0;
		Stop_Motor;
		xSemaphoreGive(MotorMutex);
	}
	}

void WindowDown_Passenger(void *params)
{
	xSemaphoreTake(WindowDown_passengerS, 0);
	while (1)
	{
		xSemaphoreTake(WindowDown_passengerS, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		
		if (!GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_PASSENGER_PIN))
		{

		delayMs(50);
		Move_Down;
		delayMs(1000);
		if(GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_PASSENGER_PIN))
		{
			//automatic
			
			while(GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_7))
			{
				if (!GPIOPinRead(BUTTONS_PORT, WINDOW_UP_DRIVER_PIN) || !GPIOPinRead(BUTTONS_PORT, WINDOW_UP_PASSENGER_PIN))
				{
					break;
				}
			}
		}
			else
				{				
					//manual
					while (!(GPIOPinRead(BUTTONS_PORT, WINDOW_DOWN_PASSENGER_PIN)) && GPIOPinRead(LIMIT_SWITCHES_PORT, GPIO_PIN_7))
					{
						
					}
						Stop_Motor;
			}
		}
		Stop_Motor;
		xSemaphoreGive(MotorMutex);
	}
}

void OnOff(void *params)
{
	xSemaphoreTake(OnOffS, 0);
	while (1)
	{
		xSemaphoreTake(OnOffS, portMAX_DELAY);	
			
		while (!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4)))
		{
			GPIOIntDisable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7 );
		__WFI();
		}
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7 );
	}
}

void Obstacle ()
{
		Stop_Motor;
		delayMs(200);
		Move_Down;
		delayMs(500);
		Stop_Motor;	
}