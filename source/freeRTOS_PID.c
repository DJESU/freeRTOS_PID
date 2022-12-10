/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Practica5_PID.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "fsl_adc16.h"
#include "fsl_pit.h"
#include "fsl_uart.h"
#include "fsl_dac.h"

/* TODO: insert other definitions and declarations here. */
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LED_INIT() LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE() LED_RED_TOGGLE()
/*******************************************************************************
 * Variables
 ******************************************************************************/
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;

dac_config_t dacConfigStruct;

static QueueHandle_t queueADCaControl = NULL;
static QueueHandle_t queueControltoSerialTx = NULL;
static QueueHandle_t queueControlaDAC = NULL;

static SemaphoreHandle_t mutex_PITaADC;

static TaskHandle_t xTaskToNotify;
static SemaphoreHandle_t rxMutex;

volatile bool pitIsrFlag = false;
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue;
volatile int32_t g_DacValue;
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */
volatile uint16_t rx_dato = 0;

double Kp, Td, Ti, r, y, e, m, a, b, c, y1, m1, e1, e2, TS = 0.1;

volatile bool flagKP;
volatile bool flagTD;
volatile bool flagTI;
volatile bool flagR;

uint8_t g_tipString[] =
    "Mandame uint16 :\r\n";
uint8_t demoRingBuffer[16];

struct MSG{
	int16_t counter;
	char label;
};

/*
 * @brief   Application entry point.
 */

/*******************************************************************************
 * Code
 ******************************************************************************/
void DAC(void){
	DAC_GetDefaultConfig(&dacConfigStruct);
	DAC_Init(DAC0, &dacConfigStruct);
	DAC_Enable(DAC0, true);             /* Enable output. */
	DAC_SetBufferReadPointer(DAC0, 0U); /* Make sure the read pointer to the start. */

	g_DacValue = 0;

	struct MSG messageReceived;
	struct MSG myMessage;
	myMessage.label = 'R';

	while(1){

		xQueueReceive(queueControlaDAC, &messageReceived, portMAX_DELAY);
		g_DacValue	= messageReceived.counter;

		if (g_DacValue > 4095){
            g_DacValue = 4095;
        }
        if (g_DacValue < 0){
        	g_DacValue = 0;
        }

        DAC_SetBufferValue(DAC0, 0U, g_DacValue);

	     myMessage.counter = r;
	     xQueueSend(queueControltoSerialTx, &myMessage, 0);

	}
}

void UART0_IRQHandler(void){
	uint8_t data;

	/* If new data arrived. */
	if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(UART0))
	{
		data = UART_ReadByte(UART0);
	    demoRingBuffer[rxIndex] = data;

	    if(flagKP){
	    	Kp = data;
	    	flagKP = false;
	    }
	    if(flagTD){
	    	Td = data;
	    	flagTD = false;
	    }
	    if(flagTI){
	       	Ti = data;
	       	flagTI = false;
	    }

	    if(flagR){
	       	r = (data*4095)/100;
	       	flagR = false;
	    }

	    if((demoRingBuffer[rxIndex]) == 11){
	    	flagKP = true;
	    }
	    if((demoRingBuffer[rxIndex]) == 12){
	    	flagTD = true;
	    }
	    if((demoRingBuffer[rxIndex]) == 13){
	    	flagTI = true;
	    }
	    if((demoRingBuffer[rxIndex]) == 14){
	    	flagR = true;
	    }

	    rxIndex++;
	    /* If ring buffer is full, or message finished. */
	    if (rxIndex >= 16)
	    {
	    	rxIndex = 0;
	    }
	    if (data == '\r') //Fin del msg
	    {
	    	rxIndex = 0;
	    	xSemaphoreGiveFromISR(rxMutex,xTaskToNotify);
	    }
	}
}

static void UART_RX(void *pvParameters)
{

    while (1)
    {
     xSemaphoreTake(rxMutex,portMAX_DELAY); //Intenta tomar el Mutex
     // procesa msg
     if (demoRingBuffer[2] == '\r'){ // el msg se ve bien
    	 rx_dato = *((uint16_t*) (&demoRingBuffer[0])); //Cast de los primeros dos bytes a uint16
     }
    }
}

static void UART_TX(void){
	struct MSG messageReceived;
	while(1){
	xQueueReceive(queueControltoSerialTx, &messageReceived, portMAX_DELAY);
	UART_WriteBlocking(UART0, &messageReceived.label, 1);
	UART_WriteBlocking(UART0, &messageReceived.counter, 4);
	UART_WriteBlocking(UART0, "\r\n", 2);
	//vTaskDelay(5);
	}
}

static void Control(void){
	struct MSG messageReceived, myMessage;
	m = 0;
	m1 = 0;
	e = 0;
	e1 = 0;
	e2 = 0;
	Kp = 1;
	Ti = 255;
	Td = 0;

	while(1){
		//We calculate PID constants
		a = Kp * (1 + (TS / Ti) + (Td/TS));
		b = Kp*(-1-2*(Td/TS));
		c = Kp*(Td/TS);

		xQueueReceive(queueADCaControl, &messageReceived, portMAX_DELAY);
	    //PRINTF("ADC %c Value: %d\r\n",messageReceived.label, messageReceived.counter);
		y = messageReceived.counter;
		//y = (messageReceived.counter*100)/4095;
		e = r - y;

		m = m1 +(a*e)+(b*e1)+(c*e2);
		m1 = m;
		e2 = e1;
		e1 = e;
		y1 = y;

		if (m > 4095){
            m = 4095;
        }
        if (m < 0){
        	m = 0;
        }

    	myMessage.label = 'M';
		myMessage.counter = m;
		xQueueSend(queueControlaDAC, &myMessage, 0);
		xQueueSend(queueControltoSerialTx, &myMessage, 0);

		/*
    	myMessage.label = 'Y';
		myMessage.counter = y;
		xQueueSend(queueControltoSerialTx, &myMessage, 0);
		*/
	}

}

/*

void ADC0_IRQHandler(void){
	g_Adc16ConversionDoneFlag = true;
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC0, 0U);
}

void ADCTask (void *arg){
    vTaskDelay(25);
	}
}

*/

void PIT_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

    struct MSG myMessage;
	myMessage.counter = 0;

	int signal[] = {0, 0, 0};
	int sum = 0;
	int idx = 0;

    ADC16_SetChannelConfig(ADC0, 0U, &adc16ChannelConfigStruct);

    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(ADC0, 0U))){
    }

    signal[1] = ADC16_GetChannelConversionValue(ADC0, 0U);
    /*
    for (idx = 0; idx < 3; ++idx) {
    	sum = (signal[idx] + sum);
    }
     */
    myMessage.counter = signal[1];
    xQueueSend(queueADCaControl, &myMessage, 0);
    sum = 0;
    idx = (idx+1)%3;
    pitIsrFlag = true;
    xSemaphoreGiveFromISR(mutex_PITaADC, xTaskToNotify);
}

void TimerIRQ(void){
    /* Structure of initialize PIT */
    pit_config_t pitConfig;
    /*
     * pitConfig.enableRunInDebug = false;
     */
    LED_INIT();
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(5000U, CLOCK_GetFreq(kCLOCK_BusClk)));
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT_IRQn);

    /* Start channel 0 */
    //PRINTF("\r\nStarting channel No.0 ...");
    PIT_StartTimer(PIT, kPIT_Chnl_0);

    while (1){
    		xSemaphoreTake(mutex_PITaADC,portMAX_DELAY); //Intenta tomar el Mutex
            /* Check whether occur interupt and toggle LED */
            if (true == pitIsrFlag){
                LED_TOGGLE();
                pitIsrFlag = false;

            }
        }
}

int main(void) {
	uart_config_t config;

	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
    BOARD_InitPins();
    BOARD_BootClockRUN();

	//EnableIRQ(ADC0_IRQn);
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	//adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	ADC16_Init(ADC0, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
	ADC16_DoAutoCalibration(ADC0);

	adc16ChannelConfigStruct.channelNumber = 0U;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false; /* Enable the interrupt. */

	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = true;
	config.enableRx = true;
	UART_Init(UART0, &config, 115200);

	//UART_WriteBlocking(UART0, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
	//Enable RX interrupt.
	UART_EnableInterrupts(UART0, kUART_RxDataRegFullInterruptEnable |kUART_RxOverrunInterruptEnable);
	EnableIRQ(UART0_IRQn);
	rxMutex =  xSemaphoreCreateBinary();

    mutex_PITaADC = xSemaphoreCreateBinary();

    queueADCaControl = xQueueCreate(10, sizeof(struct MSG));
    queueControltoSerialTx = xQueueCreate(10, sizeof(struct MSG));
    queueControlaDAC = xQueueCreate(10, sizeof(struct MSG));

    xTaskCreate(TimerIRQ, "PIT", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, xTaskToNotify);
    xTaskCreate(Control, "PIDcontrol", configMINIMAL_STACK_SIZE + 10, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(UART_TX, "TXSerial", configMINIMAL_STACK_SIZE + 10, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(UART_RX, "RXSerial", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, xTaskToNotify);
    xTaskCreate(DAC, "DAC", configMINIMAL_STACK_SIZE + 10, NULL, configMAX_PRIORITIES - 1, NULL);
    vTaskStartScheduler();
    return 0 ;
}
