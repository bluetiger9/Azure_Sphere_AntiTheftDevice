/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "mt3620-baremetal.h"
#include "mt3620-gpio.h"
#include "mt3620-uart.h"
#include "mt3620-intercore.h"
#include "hx711.h"

extern uint32_t StackTop; // &StackTop == end of TCM

_Noreturn static void DefaultExceptionHandler(void);

static void HandleUartIsu0RxIrq(void);
static void HandleUartIsu0RxIrqDeferred(void);

typedef struct CallbackNode {
	bool enqueued;
	struct CallbackNode *next;
	Callback cb;
} CallbackNode;

static void EnqueueCallback(CallbackNode *node);

static _Noreturn void RTCoreMain(void);

// ARM DDI0403E.d SB1.5.2-3
// From SB1.5.3, "The Vector table must be naturally aligned to a power of two whose alignment
// value is greater than or equal to (Number of Exceptions supported x 4), with a minimum alignment
// of 128 bytes.". The array is aligned in linker.ld, using the dedicated section ".vector_table".

// The exception vector table contains a stack pointer, 15 exception handlers, and an entry for
// each interrupt.
#define INTERRUPT_COUNT 100 // from datasheet
#define EXCEPTION_COUNT (16 + INTERRUPT_COUNT)
#define INT_TO_EXC(i_) (16 + (i_))
const uintptr_t ExceptionVectorTable[EXCEPTION_COUNT] __attribute__((section(".vector_table")))
__attribute__((used)) = {
	[0] = (uintptr_t)&StackTop,                // Main Stack Pointer (MSP)
	[1] = (uintptr_t)RTCoreMain,               // Reset
	[2] = (uintptr_t)DefaultExceptionHandler,  // NMI
	[3] = (uintptr_t)DefaultExceptionHandler,  // HardFault
	[4] = (uintptr_t)DefaultExceptionHandler,  // MPU Fault
	[5] = (uintptr_t)DefaultExceptionHandler,  // Bus Fault
	[6] = (uintptr_t)DefaultExceptionHandler,  // Usage Fault
	[11] = (uintptr_t)DefaultExceptionHandler, // SVCall
	[12] = (uintptr_t)DefaultExceptionHandler, // Debug monitor
	[14] = (uintptr_t)DefaultExceptionHandler, // PendSV
	[15] = (uintptr_t)DefaultExceptionHandler, // SysTick

	[INT_TO_EXC(0)] = (uintptr_t)DefaultExceptionHandler,
	[INT_TO_EXC(1)] = (uintptr_t)DefaultExceptionHandler /*Gpt_HandleIrq1*/,
	[INT_TO_EXC(2)... INT_TO_EXC(3)] = (uintptr_t)DefaultExceptionHandler,
	[INT_TO_EXC(4)] = (uintptr_t)Uart_HandleIrq4,
	[INT_TO_EXC(5)... INT_TO_EXC(46)] = (uintptr_t)DefaultExceptionHandler,
	[INT_TO_EXC(47)] = (uintptr_t)Uart_HandleIrq47,
	[INT_TO_EXC(48)... INT_TO_EXC(INTERRUPT_COUNT - 1)] = (uintptr_t)DefaultExceptionHandler };

static _Noreturn void DefaultExceptionHandler(void)
{
	for (;;) {
		// empty.
	}
}

static void HandleUartIsu0RxIrq(void)
{
	static CallbackNode cbn = { .enqueued = false,.cb = HandleUartIsu0RxIrqDeferred };
	EnqueueCallback(&cbn);
}

static void HandleUartIsu0RxIrqDeferred(void)
{
	uint8_t buffer[32];

	for (;;) {
		size_t availBytes = Uart_DequeueData(UartIsu0, buffer, sizeof(buffer));

		if (availBytes == 0) {
			return;
		}

		Uart_EnqueueString(UartCM4Debug, "UART received ");
		Uart_EnqueueIntegerAsString(UartCM4Debug, availBytes);
		Uart_EnqueueString(UartCM4Debug, " bytes: \'");
		Uart_EnqueueData(UartCM4Debug, buffer, availBytes);
		Uart_EnqueueString(UartCM4Debug, "\'.\r\n");
	}
}

static CallbackNode *volatile callbacks = NULL;

static void EnqueueCallback(CallbackNode *node)
{
	uint32_t prevBasePri = BlockIrqs();
	if (!node->enqueued) {
		CallbackNode *prevHead = callbacks;
		node->enqueued = true;
		callbacks = node;
		node->next = prevHead;
	}
	RestoreIrqs(prevBasePri);
}

static void InvokeCallbacks(void)
{
	CallbackNode *node;
	do {
		uint32_t prevBasePri = BlockIrqs();
		node = callbacks;
		if (node) {
			node->enqueued = false;
			callbacks = node->next;
		}
		RestoreIrqs(prevBasePri);

		if (node) {
			(*node->cb)();
		}
	} while (node);
}

static _Noreturn void RTCoreMain(void)
{
	// SCB->VTOR = ExceptionVectorTable
	WriteReg32(SCB_BASE, 0x08, (uint32_t)ExceptionVectorTable);

	Uart_Init(UartCM4Debug, NULL);
	Uart_EnqueueString(UartCM4Debug, "--------------------------------\r\n");
	Uart_EnqueueString(UartCM4Debug, "UART_RTApp_MT3620_BareMetal\r\n");
	Uart_EnqueueString(UartCM4Debug, "App built on: " __DATE__ " " __TIME__ "\r\n");
	Uart_EnqueueString(
		UartCM4Debug,
		"Install a loopback header on ISU0, and press button A to send a message.\r\n");

	//Uart_Init(UartIsu0, HandleUartIsu0RxIrq);

	// Block includes led1RedGpio, GPIO8.
	static const GpioBlock pwm2 = {
		.baseAddr = 0x38030000,.type = GpioBlock_PWM,.firstPin = 8,.pinCount = 4 };

	Mt3620_Gpio_AddBlock(&pwm2);

	// Block includes buttonAGpio, GPIO12
	static const GpioBlock grp3 = {
		.baseAddr = 0x38040000,.type = GpioBlock_GRP,.firstPin = 12,.pinCount = 4 };

	//static const uintptr_t ADC_CTRL_BASE = 0x38000100;

	Mt3620_Gpio_AddBlock(&grp3);

	//Mt3620_Gpio_ConfigurePinForOutput(led1RedGpio);
	//Mt3620_Gpio_ConfigurePinForInput(buttonAGpio);

	BufferHeader *outbound, *inbound;
	uint32_t sharedBufSize = 0;
	if (GetIntercoreBuffers(&outbound, &inbound, &sharedBufSize) == -1) {
		for (;;) {
			// empty.
		}
	}
	

	static const size_t payloadStart = 20;
	static const int tickPeriodUs = 1 * 1000 * 1000;
	while (true) {
		//Uart_WritePoll("Tick\r\n");
		//Gpt3_WaitUs(tickPeriodUs);
		//Uart_WritePoll("Tock\r\n");
		//Gpt3_WaitUs(tickPeriodUs);

		__asm__("wfi");
		InvokeCallbacks();

		uint8_t buf[256];
		uint32_t dataSize = sizeof(buf);
		uint8_t j = 0;
		uint32_t mV;

		// On success, dataSize is set to the actual number of bytes which were read.
		int r = DequeueData(outbound, inbound, sharedBufSize, buf, &dataSize);

		if (r == -1 || dataSize < payloadStart)
		{
			continue;
		}

		uint8_t analog_data[4];// = { 0 , 1, 2, 3 };
		j = 0;
		for (int i = payloadStart; i < payloadStart + 4; i++)
		{
			// Put ADC data to buffer
			buf[i] = analog_data[j++];
		}

		// Send buffer to A7 Core
		EnqueueData(inbound, outbound, sharedBufSize, buf, payloadStart + 4);

	}
}
