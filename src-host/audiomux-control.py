#!/usr/bin/python3

#audiomux control

#(c) 2021 by Malte Marwedel

#This program is free software; you can redistribute it and/or modify it under
#the terms of the GNU General Public License as published by the Free Software
#Foundation; either version 2 of the License, or (at your option) any later
#version.
#This program is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.

#You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
# Street, Fifth Floor, Boston, MA 02110-1301, USA.

#version 1.0.0, tested with python 3.7


#Control codes:
# bmRequest = 0 -> set/get mux, wIndex = 0...3 + 1 data byte
# bmRequest = 1 -> save as default/get default, getter has 4 databytes
#                                   wIndex = 0 -> USB
#                                   wIndex = 1 -> DC jack
# bmRequest = 2 -> save/get ir slot data, getter has 11 databytes
#                                   wIndex = 1...16
# bmRequest = 3 -> delete ir slot data, wIndex = 1...16
# bmRequest = 4 -> device reset, 0 data bytes
# bmRequest = 5 -> get sensors wIndex = 0 -> USB voltage in mV, 2 data bytes
#                              wIndex = 1 -> DC jack voltage voltage in mV, 2 data bytes
#                              wIndex = 2 -> temperature in °C
# bmRequest = 6 -> get PCB id. May be 1 or 2, 1 data byte.
# bmRequest = 7 -> Get if other PCB MCU is sending. 1 Got ping within 3s. 1 data byte

import sys
from os import path
import time

MuxOutputs = 4
MuxInputs = 4
PresetSlots = 16

def showCurrentMux(dev):
	print('Current state:')
	for i in range(1, MuxOutputs + 1):
		deviceArray = dev.ctrl_transfer(bmRequestRecv, 0, 0, i, 1)
		if (deviceArray[0] == 0):
			print('  Output ' + str(i) + ': off')
		else:
			print('  Output ' + str(i) + ': Input ' + str(deviceArray[0]))

def showCurrentState(dev):
	showCurrentMux(dev)

	deviceArray = dev.ctrl_transfer(bmRequestRecv, 1, 0, 0, 4)
	print('On USB power:')
	for i in range(0, MuxOutputs):
		if (deviceArray[i] == 0):
			print('  Output ' + str(i + 1) + ': off')
		else:
			print('  Output ' + str(i + 1) + ': Input ' + str(deviceArray[i]))

	deviceArray = dev.ctrl_transfer(bmRequestRecv, 1, 0, 1, 4)
	print('On DC jack power:')
	for i in range(0, MuxOutputs):
		if (deviceArray[i] == 0):
			print('  Output ' + str(i + 1) + ': off')
		else:
			print('  Output ' + str(i + 1) + ': Input ' + str(deviceArray[i]))

	print('IR presets:')
	for i in range(1, PresetSlots + 1):
		deviceArray = dev.ctrl_transfer(bmRequestRecv, 2, 0, i, 7 + MuxOutputs)
		protocol = deviceArray[0]
		address = (deviceArray[2] << 8) | deviceArray[1]
		command = (deviceArray[6] << 24) | (deviceArray[5] << 16) | (deviceArray[4] << 8) | deviceArray[3]
		if protocol or address or command:
			print('  Slot' + str(i) + ':')
			print('    Protocol: ' + str(protocol) + ', address: ' + str(address) + ', command: ' + str(command))
			for j in range(1, MuxOutputs + 1):
				out = deviceArray[6 + j]
				if (out == 0):
					print('    Output ' + str(j) + ': off')
				else:
					print('    Output ' + str(i) + ': Input ' + str(out))
		else:
			print('  Slot' + str(i) + ': Unused')

	print('Voltages:')
	deviceArray = dev.ctrl_transfer(bmRequestRecv, 5, 0, 0, 2)
	usbvoltage = (deviceArray[1] << 8) | deviceArray[0]
	print('  USB: ' + str(usbvoltage) + 'mV')
	deviceArray = dev.ctrl_transfer(bmRequestRecv, 5, 0, 1, 2)
	vccvoltage = (deviceArray[1] << 8) | deviceArray[0]
	print('  Jack: ' + str(vccvoltage) + 'mV')
	deviceArray = dev.ctrl_transfer(bmRequestRecv, 5, 0, 2, 2)
	temperature = (deviceArray[1] << 8) | deviceArray[0]
	if (temperature >= 0x8000):
		temperature = temperature - 0x10000
	print('Temperature: ' + str(temperature) + '°C')
	deviceArray = dev.ctrl_transfer(bmRequestRecv, 6, 0, 0, 1)
	print('PCB id: ' + str(deviceArray[0]))
	deviceArray = dev.ctrl_transfer(bmRequestRecv, 7, 0, 0, 1)
	if (deviceArray[0]):
		print('Other MCU: running')
	else:
		print('Other MCU: Error, no response')


def saveAsDefaultUsb(dev):
	dev.ctrl_transfer(bmRequestSend, 1, 0, 0, 0)
	return(True)

def saveAsDefaultJack(dev):
	dev.ctrl_transfer(bmRequestSend, 1, 0, 1, 0)
	return(True)

def storeIr(dev, slotStr):
	slot = int(slotStr, 0)
	if not (0 < slot <= PresetSlots):
		print('Error, slot >' + slotStr + '< unsupported')
		return(False)
	dev.ctrl_transfer(bmRequestSend, 2, 0, slot, 0)
	return(True)

def deleteIr(dev, slotStr):
	slot = int(slotStr, 0)
	if not (0 < slot <= PresetSlots):
		print('Error, slot >' + slotStr + '< unsupported')
		return(False)
	dev.ctrl_transfer(bmRequestSend, 3, 0, slot, 0)
	return(True)

def switchMux(dev, outputStr, inputStr):
	out = int(outputStr, 0)
	if inputStr == 'off':
		inx = 0
	else:
		inx = int(inputStr, 0)
	if not (1 <= out <= MuxOutputs):
		print('Error, output out of range')
		return(False)
	if not (0 <= inx <= MuxInputs):
		print('Error, input out of range')
		return(False)
	usbdata = bytearray(1)
	usbdata[0] = inx
	dev.ctrl_transfer(bmRequestSend, 0, 0, out, usbdata)
	return(True)

def sendReset(dev):
	dev.ctrl_transfer(bmRequestSend, 4, 0, 0, 0)
	return(True)


parameters = len(sys.argv)

if parameters == 2:
	if (sys.argv[1] == '--help') or (sys.argv[1] == '-h'):
		print('audiomux control')
		print('(c) 2021 by Malte Marwedel. Licensed under GPL v2 or later')
		print('')
		print('Usage:')
		print('--help:        Prints this screen')
		print('--version:     Prints the version')
		print('For most setter commands, there is a queue on the other side with up to 8 entries')
		print('<output> <input>: changes the output')
		print('  Numbers can be in decimal or hexadecimal')
		print('  Example: 2 off')
		print('  Example: 2 1')
		print('audiomuxreset: Resets the device. Ignoring the command queue. Both USB ports are being resetted')
		print('savedefaultusb: Save the current state as default if the device is powered over USB')
		print('savedefaultjack: Save the current state as default if the device is powered over an DC jack')
		print('storeir <index>: Save the current state with the next IR remote code to be received. <index> is the slot to use 1...' + str(PresetSlots))
		print('deleteir <index>: Do not use the infrared code anylonger')
		print('state: Print all permanent settings and current state')
		print('Call without parameter: Prints the current state of all mux outputs and internal storage')
		print('')
		print('Return codes:')
		print('0: Command sent')
		print('1: Invalid command')
		print('2: python USB library not installed')
		print('3: USB device not found, or product name wrong')
		sys.exit(0)
	if (sys.argv[1] == '--version'):
		print('Version 0.1.0')
		sys.exit(0)

try:
	import usb.core
	import usb.util

except:
	print('Ouch, can not import usb.core and usb.util')
	print('Try: pip3 install pyusb')
	print('Or as root: apt install python3-usb')
	sys.exit(2)

# find our device
#For testing own devices, use idProduct=0x0001
dev = usb.core.find(idVendor=0x1209, idProduct=0x0001)
#dev = usb.core.find(idVendor=0x1209, idProduct=0x7701)

# was it found?
if dev is None:
	print('Error, device not found')
	sys.exit(3)

# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()


if dev.manufacturer != 'marwedels.de':
	print('Warning, the manufacturer >' + dev.manufacturer + '< might not be supported by this control program')

if dev.product != 'Audiomux':
	print('Error, the product >' + dev.product + '< is not be supported by this control program')
	sys.exit(3)


#For ctrl_transfer: bmRequestType, bmRequest, wValue and wIndex, data array or its length

#audiomux control
bmRequestSend = 0x40
bmRequestRecv = 0xC0

success = False

if parameters == 1:
	success = showCurrentMux(dev)
elif parameters == 2:
	if sys.argv[1] == 'audiomuxreset':
		success = sendReset(dev)
	elif sys.argv[1] == 'savedefaultusb':
		success = saveAsDefaultUsb(dev)
	elif sys.argv[1] == 'savedefaultjack':
		success = saveAsDefaultJack(dev)
	elif sys.argv[1] == 'state':
		success = showCurrentState(dev)
	else:
		print('Error, command unknown')
elif parameters == 3:
	if sys.argv[1] == 'storeir':
		success = storeIr(dev, sys.argv[2])
	elif sys.argv[1] == 'deleteir':
		success = deleteIr(dev, sys.argv[2])
	else:
		success = switchMux(dev, sys.argv[1], sys.argv[2])
else:
	print('Error, invalid number of arguments. Try --help')

if success == False:
	sys.exit(1)
