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


/* Control codes:
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
*/

#define COMMANDQUEUELEN 8

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
	0x10,0x01,  //usb version
	0xFF,       //device class
	0xFF,       //subclass
	0xFF,       //device protocol
	0x20,       //maximum packet size
	0x09,0x12,  //vid
	0x01,0x00,  //pid
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

#define UARTBUFFERLEN 512

char g_uartBuffer[UARTBUFFERLEN];
volatile uint8_t g_uartBufferReadIdx;
volatile uint8_t g_uartBufferWriteIdx;

bool g_secondPcb; //If true, the PCB has inputs and ouptus 3 and 4 and needs to switch uart RX/TX
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


//--------- Code for debug prints ----------------------------------------------

char printReadChar(void) {
	char out = 0;
	if (g_uartBufferReadIdx != g_uartBufferWriteIdx) {
		uint8_t ri = g_uartBufferReadIdx;
		out = g_uartBuffer[ri];
		__sync_synchronize(); //the pointer increment may only be visible after the copy
		ri = (ri + 1) % UARTBUFFERLEN;
		g_uartBufferReadIdx = ri;
	}
	return out;
}

void USART2_IRQHandler(void) {
	UART_HandleTypeDef * phuart = &huart2;
	if (__HAL_UART_GET_FLAG(phuart, UART_FLAG_TXE) == SET) {
		char c = printReadChar();
		if (c) {
			phuart->Instance->TDR = c;
		} else {
			__HAL_UART_DISABLE_IT(phuart, UART_IT_TXE);
		}
	}
	//just clear all flags
	__HAL_UART_CLEAR_FLAG(phuart, UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_OREF | UART_CLEAR_IDLEF | UART_CLEAR_TCF | UART_CLEAR_LBDF | UART_CLEAR_CTSF | UART_CLEAR_CMF | UART_CLEAR_WUF | UART_CLEAR_RTOF);
}

void printWriteChar(char out) {
	UART_HandleTypeDef * phuart = &huart2;
	uint8_t writeThis = g_uartBufferWriteIdx;
	uint8_t writeNext = (writeThis + 1) % UARTBUFFERLEN;
	if (writeNext != g_uartBufferReadIdx) {
		g_uartBuffer[writeThis] = out;
		g_uartBufferWriteIdx = writeNext;
	}
	__disable_irq();
	if (__HAL_UART_GET_IT_SOURCE(phuart, UART_IT_TXE) == RESET) {
		__HAL_UART_ENABLE_IT(phuart, UART_IT_TXE);
	}
	__enable_irq();
}

void writeString(const char * str) {
	while (*str) {
		printWriteChar(*str);
		str++;
	}
	//HAL_UART_Transmit(&huart1, (unsigned char *)str, strlen(str), 1000);
}

void dbgPrintf(const char * format, ...)
{
	static char buffer[128];

	va_list args;
	va_start(args, format);
	//removing the vsnprintf call would save more than 3KiB program code.
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	writeString(buffer);
}

//simpler than HAL_UART_Receive and without an annoying timeout
bool dbgUartGet(UART_HandleTypeDef *huart, uint8_t *pData) {

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
		*pData = (uint8_t)(huart->Instance->RDR & 0xFF);
		return true;
	}
	return false;
}

void _Error_Handler(const char *file, uint32_t line) {
	dbgPrintf("Error in %s:%u\r\n", file, (unsigned int)line);
}


//----- Audiomux and usb glue logic --------------------------------------------------

static bool cmdPut(const commandEntry_t * pCommand) {
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

static bool cmdPutMux(uint8_t wIndex, const uint8_t * buffer) {
	if ((wIndex > 0) && (wIndex <= MUX_OUTPUTS) && (buffer[0] <= MUX_INPUTS)) {
		commandEntry_t command;
		command.type = CMD_MUX_SET;
		command.data[0] = wIndex;
		command.data[1] = buffer[0];
		return cmdPut(&command);
	}
	return false;
}

static bool cmdPutSaveDefault(uint8_t wIndex) {
	if (wIndex < 2) {
		commandEntry_t command;
		command.type = CMD_SAVE_DEFAULT;
		command.data[0] = wIndex;
		return cmdPut(&command);
	}
	return false;
}

static bool cmdPutSaveIr(uint8_t wIndex) {
	if ((wIndex > 0) && (wIndex < IR_PRESET_SLOTS)) {
		commandEntry_t command;
		command.type = CMD_SAVE_IR;
		command.data[0] = wIndex;
		return cmdPut(&command);
	}
	return false;
}

static bool cmdPutDeleteIr(uint8_t wIndex) {
	if ((wIndex > 0) && (wIndex < IR_PRESET_SLOTS)) {
		commandEntry_t command;
		command.type = CMD_DELETE_IR;
		command.data[0] = wIndex;
		return cmdPut(&command);
	}
	return false;
}

static bool cmdGet(commandEntry_t * pCommand) {
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
		dataOut[1] = pIrC->address && 0xFF;
		dataOut[2] = (pIrC->address >> 8) && 0xFF;
		dataOut[3] = pIrC->command && 0xFF;
		dataOut[4] = (pIrC->command >> 8) && 0xFF;
		dataOut[5] = (pIrC->command >> 16) && 0xFF;
		dataOut[6] = (pIrC->command >> 24) && 0xFF;
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
}

//---------- USB handling ------------------------------------------------------

void USB_IRQHandler(void)
{
	usbd_poll(&g_usbDev);
}

static usbd_respond usbGetDesc(usbd_ctlreq *req, void **address, uint16_t *length) {
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
			if (dnumber < 3) {
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
				desc = pStringDescr;
				len = pStringDescr->bLength;
				result = usbd_ack;
			}
			break;
	}
	*address = desc;
	*length = len;
	return result;
}

static usbd_respond usbControl(usbd_device *dev, usbd_ctlreq *req, usbd_rqc_callback *callback) {
	//Printing can be done here as long it is buffered. Otherwise it might be too slow
	//dbgPrintf("type %x request %x wvalue %x windex %x wLength %u\r\n", req->bmRequestType, req->bRequest, req->wValue, req->wIndex, req->wLength);
	if ((req->bmRequestType & (USB_REQ_TYPE | USB_REQ_RECIPIENT)) == (USB_REQ_VENDOR | USB_REQ_DEVICE)) {
		if (req->bmRequestType & USB_REQ_DIRECTION) { //getter
			switch(req->bRequest)
			{
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
			}
		} else { //setter
			switch(req->bRequest)
			{
				case CMD_MUX_SET: //0
					if (req->wLength > 0) {
						if (cmdPutMux(req->wIndex, req->data)) {
							return usbd_ack;
						}
					}
					break;
				case CMD_SAVE_DEFAULT: //1
					if (cmdPutSaveDefault(req->wIndex)) {
						return usbd_ack;
					}
					break;
				case CMD_SAVE_IR: //2
					if (cmdPutSaveIr(req->wIndex)) {
						return usbd_ack;
					}
					break;
				case CMD_DELETE_IR: //3
					if (cmdPutDeleteIr(req->wIndex)) {
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

void startUsb(void)
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

void execMuxSwitch(uint8_t output, uint8_t input) {
	if ((output > MUX_OUTPUTS) || (input > MUX_INPUTS)) {
		return;
	}
	uint8_t inputOld = g_outputState[output - 1];
	if (inputOld == input) {
		dbgPrintf("Output %u already at input %u\r\n", output, input);
		return;
	}
	g_outputState[output - 1] = input;
	if ((g_secondPcb) && (output < 3)) { //2. PCB supports outputs 2 and 3
		return;
	}
	if ((!g_secondPcb) && (output >= 3)) { //1. PCB supports outputs 1 and 2
		return;
	}
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

void execMuxSwitchAndForward(uint8_t output, uint8_t input) {
	//forward to other PCB

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
	}
}

static void execUartRec(uint8_t uartIn) {


}

static void ledUpdate(void) {
	static bool toggle = false;
	if (toggle) {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		toggle = false;
	} else {
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		toggle = true;
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
	/* 2. PCB has for me:
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

//in ha_flash_ex.c
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
	while ((ADC1->ISR & ADC_FLAG_RDY) == 0); //measured 8 loop cycles @12MHz until bit was set
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
	NVIC_EnableIRQ(USART2_IRQn);
	dbgPrintf("Audiomux 0.2 (c) 2021 by Malte Marwedel\r\nStarting...\r\n");
	startUsb();
	irmp_init();
	initAdc();
	adcUpdate(); //required for permanent settings interpretation
	if (HAL_GPIO_ReadPin(SecondPcb_GPIO_Port, SecondPcb_Pin) == GPIO_PIN_RESET) {
		g_secondPcb = true;
	}
	HAL_TIM_Base_Start_IT(&htim3);
	initPermanentSettings();
	dbgPrintf("started\r\n");
}

/*The timestamp may only increase by one for each call.
  The timestamp should be in ms. */
void Audiomux1msPassed(uint32_t timestamp) {
	static uint32_t cycleCnt = 0;

	commandEntry_t command;
	bool newCmd = cmdGet(&command);
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
	uint8_t uartIn;
	if (dbgUartGet(&huart1, &uartIn)) {
		execUartRec(uartIn);
	}
	if ((timestamp % 1000) == 0)
	{
		cycleCnt++;
		dbgPrintf("Cycle %u\r\n", (unsigned int)cycleCnt);
	}
	uint8_t debugData;
	if (dbgUartGet(&huart2, &debugData)) {
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
	if (g_OtherPcbCountdown) {
		g_OtherPcbCountdown--;
	}
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

