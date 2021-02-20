/*
    Audiomux
    Copyright (C) 2021 Malte Marwedel

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.


Version history:
2021-02-07: 0.2 - initial github checkin
2021-02-14: 0.4 - features complete
2021-02-20: 0.5 - analyze stack usage, add WCID extension
2021-02-20: 0.6 - add final USB PID


Maximum HSI clock derivation:
-40°C: -2.25%...+3.75% -> delta: 6%
-20°C: -1.5%...+2.75%  -> delta: 4.25%
-10°C  -1.25%...2.25%  -> delta: 3.5%
+40°C -1%...+1.5%      -> delta: 2.5%
+80°C -1.75%...1.25%   -> delta: 3%

UART charactersistics: Over8 = 0, BRR[3:0] = 0000, M[1:0] = 00, ONEBIT = 0
  -> Allowed tolerance: 3.75% -> The communication between the two PCBs will
  work in the temperature range -10°C to +80°C. This should be enough for
  indoor use :) So no need for HSE crystal or auto baud detection.

UART control codes:
 High byte is send first and must always have MSB 0x80 set ->
   makes re-sync easier if we lost a char
 0x8001 -> Ping
 0x90xy -> Set output x to input y -> so this is limited to 15 outputs and
           15 inputs + mute

USB Control codes:

bmRequestType = USB_REQ_VENDOR & USB_REQ_DEVICE

# bmRequest = 0 -> set/set mux, wIndex = 0...3 + 1 data byte
# bmRequest = 1 -> save as default/get default, getter has 4 databytes
#                                   wIndex = 0 -> USB
#                                   wIndex = 1 -> DC jack
# bmRequest = 2 -> save/get ir slot data, getter has 11 databytes
#                                   wIndex = 1...16
# bmRequest = 3 -> delete ir slot data, wIndex = 1...16
# bmRequest = 4 -> device reset, 0 data bytes
# bmRequest = 5 -> get voltage wIndex = 0 -> USB voltage, 2 data bytes
#                              wIndex = 1 -> DC jack voltage voltage, 2 data bytes
# bmRequest = 6 -> get PCB id. May be 1 or 2, 1 data byte.
# bmRequest = 7 -> Get if other PCB MCU is sending. 1 Got ping within 3s. 1 data byte

Windows 10 try after deleting registry:

type 80 req 6 wVal 100 wInd 0 wLen 18
type 80 req 6 wVal 200 wInd 0 wLen 9
type 80 req 6 wVal 200 wInd 0 wLen 18
type 80 req 6 wVal 100 wInd 0 wLen 64
Cycle 17
type 0 req 5 wVal E wInd 0 wLen 0
type 80 req 6 wVal 100 wInd 0 wLen 18
type 80 req 6 wVal 200 wInd 0 wLen 18
type 0 req 9 wVal 1 wInd 0 wLen 0
type 80 req 6 wVal 100 wInd 0 wLen 64
type 0 req 5 wVal E wInd 0 wLen 0
type 80 req 6 wVal 100 wInd 0 wLen 18
type 80 req 6 wVal 200 wInd 0 wLen 18
type 0 req 9 wVal 1 wInd 0 wLen 0
type 80 req 6 wVal 3EE wInd 0 wLen 18
--This is the 'W'=0x57 request for the driver description:--
type C0 req 57 wVal 0 wInd 4 wLen 16
type C0 req 57 wVal 0 wInd 4 wLen 40
type 80 req 6 wVal 300 wInd 0 wLen 255
type 80 req 6 wVal 302 wInd 409 wLen 255
type 80 req 6 wVal 600 wInd 0 wLen 10
Cycle 18
type C1 req 57 wVal 0 wInd 5 wLen 10

*/


#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

#include "audiomux.h"

#include "usbd_core.h"
#include "usb.h"
#include "main.h"
#include "irmp.h"
#include "femtoVsnprintf.h"


#define COMMANDQUEUELEN 16

#define CMD_MUX_SET 0
#define CMD_SAVE_DEFAULT 1
#define CMD_SAVE_IR 2
#define CMD_DELETE_IR 3

#define MUX_OUTPUTS 4
#define MUX_INPUTS 4
#define IR_PRESET_SLOTS 16

#define FLASH_PAGES ((FLASH_BANK1_END + 1 - FLASH_BASE) / FLASH_PAGE_SIZE)

#define SETTINGS_FLASH_ADDR (FLASH_BASE + (FLASH_PAGES - 1) * FLASH_PAGE_SIZE)

typedef struct {
	/*
	   0 -> set mux, 2 data bytes, 1. data byte ouput, 2. data byte input
	   1 -> save as default, 1 data byte
	   2 -> save as ir slot value, 1 data byte
	   3 -> delete ir slot data, 1 data byte
	*/
	uint8_t type;
	uint8_t data[2];
} commandEntry_t;

typedef struct {
	uint8_t protocol;
	uint16_t address;
	uint32_t command;
	uint8_t outputs[MUX_OUTPUTS];
} irPreset_t;

typedef struct {
	uint8_t initializedMagic1; //42 to be valid
	uint8_t usbDefaults[MUX_OUTPUTS];
	uint8_t jackDefaults[MUX_OUTPUTS];
	irPreset_t irPreset[IR_PRESET_SLOTS];
	uint8_t initializedMagic2; //23 to be valid
} permanentSettings_t;


//-------------------- Global variables ----------------------------------------

usbd_device g_usbDev;

//must be 4byte aligned
uint32_t g_usbBuffer[20];

/* The PID is reserved for this project. Please use another PID,
should the source be used for a project with another purpose/incompatible
USB control commands */
uint8_t g_deviceDescriptor[] = {
	0x12,       //length of this struct
	0x01,       //always 1
	0x00,0x02,  //usb version
	0xFF,       //device class
	0xFF,       //subclass
	0xFF,       //device protocol
	32,         //maximum packet size
	0x09,0x12,  //vid
	0x01,0x77,  //pid
	0x00,0x01,  //revision
	0x1,        //manufacturer index
	0x2,        //product name index
	0x0,        //serial number index
	0x01        //number of configurations
};

uint8_t g_DeviceConfiguration[] = {
	9, //length of this entry
	0x2, //device configuration
	18, 0, //total length of this struct
	0x1, //number of interfaces
	0x1, //this config
	0x0, //descriptor of this config index, not used
	0x80, //bus powered
	85,   //170mA
	//interface descriptor follows
	9, //length of this descriptor
	0x4, //interface descriptor
	0x0, //interface number
	0x0, //alternate settings
	0x0, //number of endpoints without ep 0
	0xFF, //class code -> vendor specific
	0x0, //subclass code
	0x0, //protocol code
	0x0 //string index for interface descriptor
};

static struct usb_string_descriptor g_lang_desc     = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static struct usb_string_descriptor g_manuf_desc_en = USB_STRING_DESC("marwedels.de");
static struct usb_string_descriptor g_prod_desc_en  = USB_STRING_DESC("Audiomux");

//The W will be the code for bRequest later
static struct usb_string_descriptor g_wcid_desc     = USB_ARRAY_DESC('M', 'S', 'F', 'T', '1', '0', '0', 'W');

//good tutorial: https://github.com/pbatard/libwdi/wiki/WCID-Devices
const uint8_t g_WcidPacket[] = {
40, 0, 0, 0,  //descriptor length = 40 bytes = size of this struct
0, 1, //version 1.0
4, 0, //comatibility id
1, //number of sections
0, 0, 0, 0, 0, 0, 0, //reserved...
0, //interface number
1, //reserved... but why should it be 1?
'W', 'I', 'N', 'U', 'S', 'B', 0, 0, //the driver to use, no driver to install, but no python backend
0, 0, 0, 0, 0, 0, 0, 0, //we love zeros...
0, 0, 0, 0, 0, 0 //even more reserved...
};

commandEntry_t g_commandQueue[COMMANDQUEUELEN];
/*logic: if g_commandQueueReadIdx != g_commandQueueWriteIdx -> something to read from
  if (g_commandQueueReadIdx != (g_commandQueueWriteIdx + 1)) -> we have space to write to
  since the read and write index pointer read-access is atomic, no interrupt lock is needed.
  For write no lock is needed, as only the one interrupt modifies the WP, and only the
  main loop modifies the RP.
*/
volatile uint8_t g_commandQueueReadIdx;
volatile uint8_t g_commandQueueWriteIdx;

volatile uint8_t g_ResetRequest;

volatile bool g_irRxValid;
IRMP_DATA g_irRxLast;

#define UARTBUFFERLEN 500

char g_uartBuffer[UARTBUFFERLEN];
volatile uint32_t g_uartBufferReadIdx;
volatile uint32_t g_uartBufferWriteIdx;

bool g_secondPcb; //If true, the PCB has inputs and ouptus 3 and 4 and needs to switch uart RX/TX

//IMC = inter MCU communication
#define IMCBUFFERLEN 8

//This buffer is not used by an interrupt, so no volatile needed
uint16_t g_imcTxQueue[IMCBUFFERLEN];
uint32_t g_imcTxQueueReadIdx;
uint32_t g_imcTxQueueWriteIdx;

//This buffer is filled by the ISR
uint16_t g_imcRxQueue[IMCBUFFERLEN];
volatile uint32_t g_imcRxQueueReadIdx;
volatile uint32_t g_imcRxQueueWriteIdx;



/* Information flow:
   USB -> Own PCB ? Set relays and g_outputState + send to other PCB
   USB -> Not own PCB? -> send to other PCB
   IR -> Own PCB ? Set relays and g_outputState + send to other PCB
   IR -> Not own PCB? -> send to other PCB
   UART -> Own PCB? Set relays and g_outputState + send to other PCB
   UART -> Not own PCB? -> set g_outputState
So by updateing g_outputState for the not own PCB values only over the UART
both devices will always end up in the same state.
*/


uint8_t g_outputState[MUX_OUTPUTS];
uint16_t g_voltageUsb;
uint16_t g_voltageJack;
int16_t g_temperature;

uint8_t g_nextIrCodeSaveSlot;

permanentSettings_t g_settings;

//If reached zero, permanentSettings are written back to flash
uint32_t g_writeCountdown;

//If reached zero, this indicates the oter PCB is not working properly
uint32_t g_OtherPcbCountdown;

//If reached zero, send ping to other PCB
uint32_t g_PingCountdown;

//Statistics about the amount of stack actually used
uintptr_t g_MinStackValue = 0xFFFFFFFF;
extern int _estack;

//--------- Code for debug prints ----------------------------------------------

static void initUart2(void) {
	__HAL_RCC_USART2_CLK_ENABLE();
	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);
	HAL_Delay(50);

	USART2->CR1 = USART_CR1_TXEIE | USART_CR1_RE | USART_CR1_TE; //we want the TX interrupt
	USART2->CR2 = 0;
	USART2->CR3 = 0;
	USART2->BRR = HAL_RCC_GetPCLK1Freq() / 19200;
	USART2->CR1 |= USART_CR1_UE;

	GPIO_InitTypeDef gitd = {0};
	gitd.Pin = GPIO_PIN_2;
	gitd.Mode = GPIO_MODE_AF_PP;
	gitd.Pull = GPIO_NOPULL;
	gitd.Speed = GPIO_SPEED_FREQ_HIGH;
	gitd.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &gitd);

	gitd.Pin = GPIO_PIN_3;
	gitd.Mode = GPIO_MODE_AF_PP;
	gitd.Pull = GPIO_PULLUP;
	gitd.Speed = GPIO_SPEED_FREQ_HIGH;
	gitd.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &gitd);
}


static char printReadChar(void) {
	char out = 0;
	if (g_uartBufferReadIdx != g_uartBufferWriteIdx) {
		uint32_t ri = g_uartBufferReadIdx;
		out = g_uartBuffer[ri];
		__sync_synchronize(); //the pointer increment may only be visible after the copy
		ri = (ri + 1) % UARTBUFFERLEN;
		g_uartBufferReadIdx = ri;
	}
	return out;
}

void USART2_IRQHandler(void) {
	if (USART2->ISR & USART_ISR_TXE) {
		char c = printReadChar();
		if (c) {
			USART2->TDR = c;
		} else {
			USART2->CR1 &= ~USART_CR1_TXEIE;
		}
	}
	//just clear all flags
	USART2->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_IDLECF | USART_ICR_TCCF | USART_ICR_LBDCF | USART_ICR_CTSCF | USART_ICR_RTOCF | USART_ICR_EOBCF | USART_ICR_CMCF | USART_ICR_WUCF;
	NVIC_ClearPendingIRQ(USART2_IRQn);
}

//call only within interrupt locks
static void printWriteChar(char out) {
	uint32_t writeThis = g_uartBufferWriteIdx;
	uint32_t writeNext = (writeThis + 1) % UARTBUFFERLEN;
	if (writeNext != g_uartBufferReadIdx) {
		g_uartBuffer[writeThis] = out;
		g_uartBufferWriteIdx = writeNext;
	}
	if ((USART2->CR1 & USART_CR1_TXEIE) == 0) {
		USART2->CR1 |= USART_CR1_TXEIE;
	}
}

//can be called from both, non interrupt and interrupt code, so lock it
static void writeString(const char * str) {
	__disable_irq();
	while (*str) {
		printWriteChar(*str);
		str++;
	}
	__enable_irq();
}

void dbgPrintf(const char * format, ...)
{
	static char buffer[128];

	va_list args;
	va_start(args, format);
	//Replacing vsnprintf call by femtoVsnprintf saves ~2.5KiB program code.
	//vsnprintf(buffer, sizeof(buffer), format, args);
	femtoVsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	writeString(buffer);
}

//simpler than HAL_UART_Receive and without an annoying timeout
static bool dbgUartGet(uint8_t *pData) {

	if ((USART2->ISR) & USART_ISR_RXNE) {
		*pData = (uint8_t)(USART2->RDR);
		return true;
	}
	return false;
}

void _Error_Handler(const char *file, uint32_t line) {
	dbgPrintf("Error in %s:%u\r\n", file, (unsigned int)line);
}

static void SampleStackUsage(void) {
	uint8_t dummy;
	if ((uintptr_t)&dummy < g_MinStackValue) {
		g_MinStackValue = (uintptr_t)&dummy;
	}
}

//--------- Routing code -------------------------------------------------------

static bool forThisPcb(uint8_t output) {
	if ((g_secondPcb) && (output < 3)) { //2. PCB supports outputs 2 and 3
		return false;
	}
	if ((!g_secondPcb) && (output >= 3)) { //1. PCB supports outputs 1 and 2
		return false;
	}
	return true;
}

//----- Audiomux and usb glue logic --------------------------------------------

static bool usbCmdPut(const commandEntry_t * pCommand) {
	uint8_t writeThis = g_commandQueueWriteIdx;
	uint8_t writeNext = (writeThis + 1) % COMMANDQUEUELEN;
	if (writeNext != g_commandQueueReadIdx) {
		memcpy(&(g_commandQueue[writeThis]), pCommand, sizeof(commandEntry_t));
		/*No need for a barrier, since the main loop cant access the queue before
		  the interrupt is over anyway */
		g_commandQueueWriteIdx = writeNext;
		return true;
	}
	return false;
}

static bool usbCmdPutMux(uint8_t wIndex, const uint8_t * buffer) {
	if ((wIndex > 0) && (wIndex <= MUX_OUTPUTS) && (buffer[0] <= MUX_INPUTS)) {
		commandEntry_t command;
		command.type = CMD_MUX_SET;
		command.data[0] = wIndex;
		command.data[1] = buffer[0];
		return usbCmdPut(&command);
	}
	return false;
}

static bool usbCmdPutSaveDefault(uint8_t wIndex) {
	if (wIndex < 2) {
		commandEntry_t command;
		command.type = CMD_SAVE_DEFAULT;
		command.data[0] = wIndex;
		return usbCmdPut(&command);
	}
	return false;
}

static bool usbCmdPutSaveIr(uint8_t wIndex) {
	if ((wIndex > 0) && (wIndex < IR_PRESET_SLOTS)) {
		commandEntry_t command;
		command.type = CMD_SAVE_IR;
		command.data[0] = wIndex;
		return usbCmdPut(&command);
	}
	return false;
}

static bool usbCmdPutDeleteIr(uint8_t wIndex) {
	if ((wIndex > 0) && (wIndex < IR_PRESET_SLOTS)) {
		commandEntry_t command;
		command.type = CMD_DELETE_IR;
		command.data[0] = wIndex;
		return usbCmdPut(&command);
	}
	return false;
}

static bool usbCmdGet(commandEntry_t * pCommand) {
	if (g_commandQueueReadIdx != g_commandQueueWriteIdx) {
		uint8_t ri = g_commandQueueReadIdx;
		memcpy(pCommand, &(g_commandQueue[ri]), sizeof(commandEntry_t));
		__sync_synchronize(); //the pointer increment may only be visible after the copy
		ri = (ri + 1) % COMMANDQUEUELEN;
		g_commandQueueReadIdx = ri;
		return true;
	}
	return false;
}

static bool usbMuxGet(uint8_t output, uint8_t * dataOut) {
	if ((output > 0) && (output <= MUX_OUTPUTS)) {
		dataOut[0] = g_outputState[output - 1];
		return true;
	}
	return false;
}

static bool usbGetDefault(uint8_t vccType, uint8_t * dataOut) {
	uint8_t * defaults = NULL;
	if (vccType == 0) {
		defaults = g_settings.usbDefaults;
	}
	if (vccType == 1) {
		defaults = g_settings.jackDefaults;
	}
	if (defaults) {
		for (uint8_t i = 0; i < MUX_OUTPUTS; i++) {
			dataOut[i] = defaults[i];
		}
		return true;
	}
	return false;
}

static bool usbGetIrPreset(uint8_t irSlotIndex, uint8_t * dataOut) {
	if ((irSlotIndex) && (irSlotIndex <= IR_PRESET_SLOTS)) {
		irPreset_t * pIrC = &(g_settings.irPreset[irSlotIndex - 1]);
		dataOut[0] = pIrC->protocol;
		dataOut[1] = pIrC->address & 0xFF;
		dataOut[2] = (pIrC->address >> 8) & 0xFF;
		dataOut[3] = pIrC->command & 0xFF;
		dataOut[4] = (pIrC->command >> 8) & 0xFF;
		dataOut[5] = (pIrC->command >> 16) & 0xFF;
		dataOut[6] = (pIrC->command >> 24) & 0xFF;
		for (uint8_t i = 0; i < MUX_OUTPUTS; i++) {
			dataOut[7 + i] = pIrC->outputs[i];
		}
		return true;
	}
	return false;
}

static bool usbGetSensors(uint8_t sensorType, uint8_t * dataOut) {
	if (sensorType == 0) {
		dataOut[0] = g_voltageUsb & 0xFF;
		dataOut[1] = (g_voltageUsb >> 8) & 0xFF;
		return true;
	}
	if (sensorType == 1) {
		dataOut[0] = g_voltageJack & 0xFF;
		dataOut[1] = (g_voltageJack >> 8) & 0xFF;
		return true;
	}
	if (sensorType == 2) {
		dataOut[0] = g_temperature & 0xFF;
		dataOut[1] = (g_temperature >> 8) & 0xFF;
		return true;
	}
	return false;
}

static bool usbGetPcbId(uint8_t * dataOut) {
	if (g_secondPcb) {
		dataOut[0] = 2;
	} else {
		dataOut[0] = 1;
	}
	return true;
}

static bool usbGetOtherPcbState(uint8_t * dataOut) {
	if (g_OtherPcbCountdown) {
		dataOut[0] = 1;
	} else {
		dataOut[0] = 0;
	}
	return true;
}

//--------- IR handling --------------------------------------------------------

void  timer3IntCallback(void) {
	irmp_ISR();
	SampleStackUsage();
}

//---------- USB handling ------------------------------------------------------

void USB_IRQHandler(void)
{
	usbd_poll(&g_usbDev);
}

static usbd_respond usbGetDesc(usbd_ctlreq *req, void **address, uint16_t *length) {
	SampleStackUsage();
	const uint8_t dtype = req->wValue >> 8;
	const uint8_t dnumber = req->wValue & 0xFF;
	void* desc = NULL;
	uint16_t len = 0;
	usbd_respond result = usbd_fail;
	switch (dtype) {
		case USB_DTYPE_DEVICE:
			desc = g_deviceDescriptor;
			len = sizeof(g_deviceDescriptor);
			result = usbd_ack;
			break;
		case USB_DTYPE_CONFIGURATION:
			desc = g_DeviceConfiguration;
			len = sizeof(g_DeviceConfiguration);
			result = usbd_ack;
			break;
		case USB_DTYPE_STRING:
			{
				struct usb_string_descriptor * pStringDescr = NULL;
				if (dnumber == 0) {
					pStringDescr = &g_lang_desc;
				}
				if (dnumber == 1) {
					pStringDescr = &g_manuf_desc_en;
				}
				if (dnumber == 2) {
					pStringDescr = &g_prod_desc_en;
				}
				if (dnumber == 0xEE) {
					pStringDescr = &g_wcid_desc;
				}
				if (pStringDescr) {
					desc = pStringDescr;
					len = pStringDescr->bLength;
					result = usbd_ack;
				}
			}
			break;
	}
	*address = desc;
	*length = len;
	return result;
}

static usbd_respond usbControl(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
	SampleStackUsage();
	//Printing can be done here as long it is buffered. Otherwise it might be too slow
	dbgPrintf("type %x req %x wVal %x wInd %x wLen %u\r\n", req->bmRequestType, req->bRequest, req->wValue, req->wIndex, req->wLength);
	if ((req->bmRequestType & (USB_REQ_TYPE | USB_REQ_RECIPIENT)) == (USB_REQ_VENDOR | USB_REQ_DEVICE)) {
		if (req->bmRequestType & USB_REQ_DIRECTION) { //getter
			switch(req->bRequest) {
				case CMD_MUX_SET: //0
					if (req->wLength > 0) {
						if (usbMuxGet(req->wIndex, req->data)) {
							return usbd_ack;
						}
					}
					break;
				case CMD_SAVE_DEFAULT: //1
					if (req->wLength >= 4) {
						if (usbGetDefault(req->wIndex, req->data)) {
							return usbd_ack;
						}
					}
					break;
				case CMD_SAVE_IR: //2
					if (req->wLength >= (MUX_OUTPUTS + 7)) {
						if (usbGetIrPreset(req->wIndex, req->data)) {
							return usbd_ack;
						}
					}
					break;
				case 5:
					if (req->wLength >= 2) {
						if (usbGetSensors(req->wIndex, req->data)) {
							return usbd_ack;
						}
					}
					break;
				case 6:
					if (req->wLength >= 1) {
						if (usbGetPcbId(req->data)) {
							return usbd_ack;
						}
					}
					break;
				case 7:
					if (req->wLength >= 1) {
						if (usbGetOtherPcbState(req->data)) {
							return usbd_ack;
						}
					}
					break;
				case 'W': //windows driver awareness
					if (req->wIndex == 4) {
						dbgPrintf("Windows special\r\n");
						size_t copyLen = req->wLength;
						//The first request is only 16bytes in length to get the size.
						if (copyLen > sizeof(g_WcidPacket)) {
							copyLen = sizeof(g_WcidPacket);
						}
						memcpy(req->data, g_WcidPacket, copyLen);
						return usbd_ack;
					}
			}
		} else { //setter
			switch(req->bRequest) {
				case CMD_MUX_SET: //0
					if (req->wLength > 0) {
						if (usbCmdPutMux(req->wIndex, req->data)) {
							return usbd_ack;
						}
					}
					break;
				case CMD_SAVE_DEFAULT: //1
					if (usbCmdPutSaveDefault(req->wIndex)) {
						return usbd_ack;
					}
					break;
				case CMD_SAVE_IR: //2
					if (usbCmdPutSaveIr(req->wIndex)) {
						return usbd_ack;
					}
					break;
				case CMD_DELETE_IR: //3
					if (usbCmdPutDeleteIr(req->wIndex)) {
						return usbd_ack;
					}
					break;
				case 4:
					g_ResetRequest = true;
					return usbd_ack;
			}
		}
	}
	return usbd_fail;
}

static usbd_respond usbSetConf(usbd_device *dev, uint8_t cfg) {
	usbd_respond result = usbd_fail;
	switch (cfg) {
		case 0:
			//deconfig
			break;
		case 1:
			//set config
			result = usbd_ack;
			break;
	}
	return result;
}

static void startUsb(void)
{
	__HAL_RCC_USB_CLK_ENABLE();
	usbd_init(&g_usbDev, &usbd_hw, 0x20, g_usbBuffer, sizeof(g_usbBuffer));
	usbd_reg_config(&g_usbDev, usbSetConf);
	usbd_reg_control(&g_usbDev, usbControl);
	usbd_reg_descr(&g_usbDev, usbGetDesc);

	usbd_enable(&g_usbDev, true);
	usbd_connect(&g_usbDev, true);

	NVIC_EnableIRQ(USB_IRQn);
}



//-------- the logic for the mux -----------------------------------------------

static void MuxSetOutput(uint8_t output, uint8_t input, bool enabled) {
	GPIO_PinState gps = GPIO_PIN_RESET;
	if (enabled) {
		gps = GPIO_PIN_SET;
	}
	if (output == 1) {
		if (input == 0) {
			HAL_GPIO_WritePin(SoundAMute_GPIO_Port, SoundAMute_Pin, gps);
		}
		if (input == 1) {
			HAL_GPIO_WritePin(SoundA1_GPIO_Port, SoundA1_Pin, gps);
		}
		if (input == 2) {
			HAL_GPIO_WritePin(SoundA2_GPIO_Port, SoundA2_Pin, gps);
		}
		if (input == 3) {
			HAL_GPIO_WritePin(SoundA3_GPIO_Port, SoundA3_Pin, gps);
		}
		if (input == 4) {
			HAL_GPIO_WritePin(SoundA4_GPIO_Port, SoundA4_Pin, gps);
		}
	}
	if (output == 2) {
		if (input == 0) {
			HAL_GPIO_WritePin(SoundBMute_GPIO_Port, SoundBMute_Pin, gps);
		}
		if (input == 1) {
			HAL_GPIO_WritePin(SoundB1_GPIO_Port, SoundB1_Pin, gps);
		}
		if (input == 2) {
			HAL_GPIO_WritePin(SoundB2_GPIO_Port, SoundB2_Pin, gps);
		}
		if (input == 3) {
			HAL_GPIO_WritePin(SoundB3_GPIO_Port, SoundB3_Pin, gps);
		}
		if (input == 4) {
			HAL_GPIO_WritePin(SoundB4_GPIO_Port, SoundB4_Pin, gps);
		}
	}
}

static void execMuxSwitch(uint8_t output, uint8_t input) {
	if ((output > MUX_OUTPUTS) || (input > MUX_INPUTS)) {
		return;
	}
	if (forThisPcb(output) == false) {
		return;
	}
	uint8_t inputOld = g_outputState[output - 1];
	if (inputOld == input) {
		dbgPrintf("Output %u already at input %u\r\n", output, input);
		return;
	}
	/* The state of the outputs of the other PCB are only updated when getting it
	   as value back over the uart -> always synchronized
	*/
	g_outputState[output - 1] = input;
	dbgPrintf("Set output %u to %u\r\n", output, input);
	if (output >= 3) {
		output -= 2;
	}
	if (inputOld != 0) {
		MuxSetOutput(output, 0, true); //mute on
		HAL_Delay(10);
		for (uint8_t i = 1; i <= MUX_INPUTS; i++) {
			MuxSetOutput(output, i, false); //inputs disconnected
		}
		HAL_Delay(10);
	}
	if (input != 0) {
		MuxSetOutput(output, input, true); //new input connected
		HAL_Delay(10);
		MuxSetOutput(output, 0, false); //mute off
	}
}

static bool imcTxPut(uint16_t command) {
	uint32_t writeThis = g_imcTxQueueWriteIdx;
	uint32_t writeNext = (writeThis + 1) % IMCBUFFERLEN;
	if (writeNext != g_imcTxQueueReadIdx) {
		g_imcTxQueue[writeThis] = command;
		g_imcTxQueueWriteIdx = writeNext;
		return true;
	}
	return false;
}

void execMuxSwitchAndForward(uint8_t output, uint8_t input) {
	uint16_t command = 0x9000 | (output << 4) | input;
	imcTxPut(command);
	execMuxSwitch(output, input);
}

static void applyPermanentSettings(const uint8_t * defaults) {
	for (uint8_t i = 0; i < MUX_OUTPUTS; i++) {
		execMuxSwitchAndForward(i + 1, defaults[i]);
	}
}

static void initiateWriteCountdown(void) {
	g_writeCountdown = 1000*60*2; //batch aquire changes for 2minutes
}

static void storeStatePermanent(uint8_t * outputs) {
	memcpy(outputs, g_outputState, sizeof(g_outputState));
	initiateWriteCountdown();
}

static void execSaveDefault(uint8_t index) {
	if (index == 0) {
		dbgPrintf("Save default for USB\r\n");
		storeStatePermanent(g_settings.usbDefaults);
	}
	if (index == 1) {
		dbgPrintf("Save default for jack\r\n");
		storeStatePermanent(g_settings.jackDefaults);
	}
}

static void execSaveIr(uint8_t index) {
	g_nextIrCodeSaveSlot = index;
	dbgPrintf("Next IR code will save settings\r\n");
}

static void execDeleteIr(uint8_t index) {
	dbgPrintf("Ir preset %u to be deleted\r\n", index);
	irPreset_t * pIp = &(g_settings.irPreset[index - 1]);
	memset(pIp, 0, sizeof(irPreset_t));
	initiateWriteCountdown();
}

static void execIrRec(IRMP_DATA * pIrmp_data) {
	dbgPrintf("Rec IR proto: %u, addr: 0x%x, comm: 0x%x, flags: 0x%x\r\n",
	  pIrmp_data->protocol, pIrmp_data->address, (unsigned int)pIrmp_data->command, pIrmp_data->flags);
	if (g_nextIrCodeSaveSlot == 0) {
		for (uint8_t i = 0; i < IR_PRESET_SLOTS; i++) {
			if ((g_settings.irPreset[i].protocol == pIrmp_data->protocol) &&
			    (g_settings.irPreset[i].address == pIrmp_data->address) &&
			    (g_settings.irPreset[i].command == pIrmp_data->command))
			{
				dbgPrintf("Applying IR preset %i\r\n", i + 1);
				applyPermanentSettings(g_settings.irPreset[i].outputs);
			}
		}
	} else {
		if (g_nextIrCodeSaveSlot <= IR_PRESET_SLOTS) {
			dbgPrintf("Store IR preset %i\r\n", g_nextIrCodeSaveSlot);
			irPreset_t * pIp = &(g_settings.irPreset[g_nextIrCodeSaveSlot - 1]);
			pIp->protocol = pIrmp_data->protocol;
			pIp->address = pIrmp_data->address;
			pIp->command = pIrmp_data->command;
			storeStatePermanent(pIp->outputs);
		}
		g_nextIrCodeSaveSlot = 0;
	}
}

//called every 100ms
static void ledUpdate(void) {
	static bool toggle = false;
	static uint8_t toggleCntDown = 0;

	if (toggleCntDown == 0) {
		if (toggle) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			toggle = false;
		} else {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			toggle = true;
		}
		if (g_OtherPcbCountdown == 0)
		{
			toggleCntDown = 0; //5Hz blink code as error
		} else {
			toggleCntDown = 5; //1Hz blink code as all is ok
		}
	} else {
		toggleCntDown--;
	}
}

static uint16_t getAdc(uint32_t channel) {
	ADC1->ISR |= ADC_ISR_EOC; //clear end of conversion bit
	while (ADC1->CR & ADC_CR_ADSTART); //otherwise the channel can not be changed
	ADC1->CHSELR = (1 << channel);
	ADC1->CR |= ADC_CR_ADSTART;
	while ((ADC1->ISR & ADC_ISR_EOC) == 0);
	uint16_t val = ADC1->DR;
	return val;
}

void adcUpdate() {
	/* 1. PCB has for me:
	   tsCal1 = 1778
	   tsCal2 = 1328
	   2. PCB has for me:
	   tsCal1 = 1776
	   tsCal2 = 1319
	*/
	int32_t tsCal1 = *((uint16_t*)0x1FFFF7B8); //30°C calibration value
	int32_t tsCal2 = *((uint16_t*)0x1FFFF7C2); //110°C calibration value
	uint32_t voltageJack = getAdc(ADC_CHANNEL_8);
	uint32_t voltageUsb = getAdc(ADC_CHANNEL_9);
	int32_t temperature = getAdc(ADC_CHANNEL_TEMPSENSOR);
	//dbgPrintf("%u, %u, %u\n\r", voltageUsb, voltageJack, temperature);
	//dbgPrintf("cal %u, %u\n\r", tsCal1, tsCal2);
	/* divier is 100k:10k, resulution is 12bit -> 4095 max at 3.3V
	   -> 0.000805mV/digit -> /10*110 -> 8.8mV/digit.
	*/
	uint32_t voltageUsbMv = voltageUsb * 88 / 10;
	uint32_t voltageJackMv = voltageJack * 88 / 10;
	int32_t temperatureCelsius = 80000 / (tsCal2 - tsCal1) * (temperature - tsCal1) + 30000;
	temperatureCelsius /= 1000;
	//dbgPrintf("USB:%umV, Jack:%umV, Temp:%i°C\n\r", voltageUsbMv, voltageJackMv, temperatureCelsius);
	__disable_irq();
	g_voltageUsb = voltageUsbMv;
	g_voltageJack = voltageJackMv;
	g_temperature = temperatureCelsius;
	__enable_irq();
}

//in hal_flash_ex.c
void FLASH_PageErase(uint32_t PageAddress);

static void updatePermanentSettings(void) {
	if (memcmp(&g_settings, (void*)SETTINGS_FLASH_ADDR, sizeof(permanentSettings_t))) {
		uint16_t buffer[(sizeof(permanentSettings_t) + sizeof(uint16_t)) / sizeof(uint16_t)] = {0};
		memcpy(buffer, &g_settings, sizeof(permanentSettings_t));
		dbgPrintf("Writing flash...%x, %ubytes\r\n", SETTINGS_FLASH_ADDR, sizeof(buffer));
		HAL_FLASH_Unlock();
		uint32_t error = 0;
		FLASH_EraseInitTypeDef feit = {0};
		feit.TypeErase = FLASH_TYPEERASE_PAGES;
		feit.PageAddress = SETTINGS_FLASH_ADDR;
		feit.NbPages = 1;
		HAL_FLASHEx_Erase(&feit, &error);
		for (uint32_t i = 0; i < sizeof(buffer); i += sizeof(uint16_t)) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, SETTINGS_FLASH_ADDR + i, buffer[i / sizeof(uint16_t)]);
		}
		HAL_FLASH_Lock();
		dbgPrintf("Done\r\n");
	} else {
		dbgPrintf("Skipping flash update - nothing changed\r\n");
	}
}

//-------- inter MCU communication ---------------------------------------------

static uint16_t imcTxGet(void) {
	uint16_t out = 0;
	if (g_imcTxQueueReadIdx != g_imcTxQueueWriteIdx) {
		uint32_t ri = g_imcTxQueueReadIdx;
		out = g_imcTxQueue[ri];
		ri = (ri + 1) % IMCBUFFERLEN;
		g_imcTxQueueReadIdx = ri;
	}
	return out;
}

static bool imcRxPut(uint16_t command) {
	//dbgPrintf("RxPut: %x\r\n", command);
	uint32_t writeThis = g_imcRxQueueWriteIdx;
	uint32_t writeNext = (writeThis + 1) % IMCBUFFERLEN;
	if (writeNext != g_imcRxQueueReadIdx) {
		g_imcRxQueue[writeThis] = command;
		g_imcRxQueueWriteIdx = writeNext;
		return true;
	}
	return false;
}

static uint16_t imcRxGet(void) {
	uint16_t out = 0;
	if (g_imcRxQueueReadIdx != g_imcRxQueueWriteIdx) {
		uint32_t ri = g_imcRxQueueReadIdx;
		out = g_imcRxQueue[ri];
		__sync_synchronize(); //the pointer increment may only be visible after the copy
		ri = (ri + 1) % IMCBUFFERLEN;
		g_imcRxQueueReadIdx = ri;
	}
	return out;
}

void USART1_IRQHandler(void) {
	static uint16_t lastByte = 0;
	static uint8_t cycle = 0;
	if (USART1->ISR & USART_ISR_RXNE) {
		//cycle = 0 -> expected to have 0x80 bit set
		uint8_t thisByte = USART1->RDR;
		//dbgPrintf("Got %x\r\n", thisByte);
		if ((cycle == 0) && (thisByte & 0x80)) {
			lastByte = thisByte;
			cycle = 1;
		} else if (cycle == 1) {
			uint16_t command = (lastByte << 8) | thisByte;
			imcRxPut(command);
			cycle = 0;
		}
	}
	//just clear all flags
	USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_ORECF | USART_ICR_IDLECF | USART_ICR_TCCF | USART_ICR_LBDCF | USART_ICR_CTSCF | USART_ICR_RTOCF | USART_ICR_EOBCF | USART_ICR_CMCF | USART_ICR_WUCF;
	NVIC_ClearPendingIRQ(USART1_IRQn);
}

static void initUart1(void) {
	__HAL_RCC_USART1_CLK_ENABLE();
	NVIC_SetPriority(USART1_IRQn, 0);
	NVIC_EnableIRQ(USART1_IRQn);
	USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE; //we want the RX interrupt
	if (g_secondPcb) {
		/*One PCB must have the swap, otherwise two outputs will drive against
		  each other! */
		USART1->CR2 = USART_CR2_SWAP;
	} else {
		USART1->CR2 = 0;
	}
	USART1->CR3 = 0;
	/*We get 10MHz and sending with ~20000baud should be safe
	  the low 5 bits of BRR must be zero, as this increases the allowed clock
	  derivation. 10M/10000 = 1024 -> 26041baud -> 26kHz is outside the hearing range :)
	*/
	USART1->BRR = 384;
	USART1->CR1 |= USART_CR1_UE;

	GPIO_InitTypeDef gitd = {0};
	gitd.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gitd.Mode = GPIO_MODE_AF_PP;
	gitd.Pull = GPIO_NOPULL;
	gitd.Speed = GPIO_SPEED_FREQ_HIGH;
	gitd.Alternate = GPIO_AF0_USART1;
	HAL_GPIO_Init(GPIOB, &gitd);
}

static void sendUart1Char(uint8_t data) {
	/*
	Very first call (other PCB sending):
	 1. ISR: 0x6000c0
	 2. ISR: 0x600000

 Typical ISR state (other PCB silent)
	 1. Byte: ISR: 0x6000c0
	 2. Byte: ISR: 0x600080
	*/

	//dbgPrintf("ISR: 0x%x\r\n", USART1->ISR);
	uint32_t timeout = HAL_GetTick() + 5;
	while ((USART1->ISR & USART_ISR_TXE) == 0) {
		if (HAL_GetTick() == timeout) {
			break; //never observed, but possible accoring to the datasheet on first sent
		}
	}
	USART1->TDR = data;
}


static void sendUart1Command(const uint8_t * data) {
	sendUart1Char(data[0]);
	sendUart1Char(data[1]);
}

//call for every 1ms. May take longer than 1ms, but not every time
static void imcUpdate(void) {
	uint8_t data[2];
	if (g_OtherPcbCountdown) {
		g_OtherPcbCountdown--;
	}
	if (g_PingCountdown == 0) {
		data[0] = 0x80;
		data[1] = 0x01;
		sendUart1Command(data);
		g_PingCountdown = 999;
	} else {
		g_PingCountdown--;
	}
	if (g_OtherPcbCountdown) {
		/*Only when the other PCB has send a ping, we know its initialized,
		  and the data won't get lost on the other side
		*/
		uint16_t command = imcTxGet();
		if (command) {
			dbgPrintf("Sent command to other PCB\r\n");
			data[0] = command >> 8;
			data[1] = command & 0xFF;
			sendUart1Command(data);
		}
	}
	uint16_t cmdFromOther = imcRxGet();
	if (cmdFromOther) {
		if (cmdFromOther == 0x8001) {
			g_OtherPcbCountdown = 3000;
		} else if ((cmdFromOther & 0xF000) == 0x9000) {
			dbgPrintf("Got command %x\r\n", cmdFromOther);
			uint8_t output = (cmdFromOther >> 4) & 0xF;
			uint8_t input = cmdFromOther & 0xF;
			if ((output <= MUX_OUTPUTS) && (input <= MUX_INPUTS)) {
				if (forThisPcb(output)) {
					execMuxSwitchAndForward(output, input);
				} else {
					if (output <= MUX_OUTPUTS) {
						g_outputState[output - 1] = input;
					}
				}
			}
		}
	}
}

//--------- main loop and init -------------------------------------------------

static void initAdc(void) {
	__HAL_RCC_ADC1_CLK_ENABLE();
	ADC1->CFGR2 = ADC_CFGR2_CKMODE_1; //PCLK div by 4. Allowed range is 0.6MHz to 14MHz.
	ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1 | ADC_SMPR_SMP_2; //slowest possible sampling time, 239.5 clock cycles
	ADC1_COMMON->CCR = ADC_CCR_TSEN; //temperature sensor enabled
	ADC1->CR |= ADC_CR_ADCAL; //start calibration
	while (ADC1->CR & ADC_CR_ADCAL); //wait for calibration to end
	HAL_Delay(1); //errata workaround
	ADC1->CR |= ADC_CR_ADEN; //can not be done within 4 adc clock cycles, due errata
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0); //measured 8 loop cycles @12MHz until bit was set
}

void initPermanentSettings(void) {
	memcpy(&g_settings, (void *)SETTINGS_FLASH_ADDR, sizeof(permanentSettings_t));
	if ((g_settings.initializedMagic1 == 42) && (g_settings.initializedMagic2 == 23))
	{
		if ((g_voltageUsb > 4000) && (g_voltageJack < 1000))
		{
			dbgPrintf("Apply settings for USB supply\r\n");
			applyPermanentSettings(g_settings.usbDefaults);
		}
		if ((g_voltageUsb < 1000) && (g_voltageJack > 5000))
		{
			dbgPrintf("Apply settings for Jack supply\r\n");
			applyPermanentSettings(g_settings.jackDefaults);
		}
	}
	else
	{
		dbgPrintf("No valid permanent settings in flash\r\n");
		memset(&g_settings, 0, sizeof(permanentSettings_t));
		g_settings.initializedMagic1 = 42;
		g_settings.initializedMagic2 = 23;
	}
}

void initAudiomux(void) {
	initUart2();
	dbgPrintf("Audiomux 0.6.0 (c) 2021 by Malte Marwedel\r\nStarting...\r\n");
	startUsb();
	irmp_init();
	initAdc();
	adcUpdate(); //required for permanent settings interpretation
	if (HAL_GPIO_ReadPin(SecondPcb_GPIO_Port, SecondPcb_Pin) == GPIO_PIN_RESET) {
		g_secondPcb = true;
	}
	initUart1(); //*must* be after setting g_secondPcb
	HAL_TIM_Base_Start_IT(&htim3);
	initPermanentSettings();
	dbgPrintf("started\r\n");
}

/*The timestamp may only increase by one for each call.
  The timestamp should be in ms. */
void Audiomux1msPassed(uint32_t timestamp) {
	static uint32_t cycleCnt = 0;

	commandEntry_t command;
	bool newCmd = usbCmdGet(&command);
	if (newCmd)
	{
		if (command.type == CMD_MUX_SET) {
			execMuxSwitchAndForward(command.data[0], command.data[1]);
		} else if (command.type == CMD_SAVE_DEFAULT) {
			execSaveDefault(command.data[0]);
		} else if (command.type == CMD_SAVE_IR) {
			execSaveIr(command.data[0]);
		} else if (command.type == CMD_DELETE_IR) {
			execDeleteIr(command.data[0]);
		}
	}
	IRMP_DATA irmp_data;
	if (irmp_get_data(&irmp_data)) {
		execIrRec(&irmp_data);
	}
	if ((timestamp % 1000) == 0) {
		cycleCnt++;
		dbgPrintf("Cycle %u\r\n", (unsigned int)cycleCnt);
		if ((timestamp % 60000) == 0) {
			uintptr_t stackTop = (uintptr_t)(&_estack);
			uintptr_t delta = stackTop - g_MinStackValue;
			dbgPrintf("Estimated stack usage: %ubyte\r\n", (unsigned int)delta);
		}
	}
	uint8_t debugData = 0;
	if (dbgUartGet(&debugData)) {
		dbgPrintf("Input %u\r\n", debugData);
	}
	if ((debugData == 'R') || (g_ResetRequest == true)) {
		dbgPrintf("Resetting...\r\n");
		HAL_Delay(50); //let the uart print its message and the USB send the act
		NVIC_SystemReset();
	}
	/* We have to call the refresh within 1s, if the clock source has its
	   nominal frequency of 40KHz. However this may vary from 20 to 50kHz!*/
	HAL_IWDG_Refresh(&hiwdg);
	if ((timestamp % 100) == 0) {
		ledUpdate();
		adcUpdate();
	}
	if (g_writeCountdown) {
		g_writeCountdown--;
		if (!g_writeCountdown) {
			updatePermanentSettings();
		}
	}
	imcUpdate();
}

void loopAudiomux(void) {
	static uint32_t counter = 0;
	uint32_t timestamp = HAL_GetTick();
	if (counter != timestamp) {
		counter++;
		Audiomux1msPassed(counter);
	} else {
		__WFI();
	}
}

