# vbus-arduino-domoticz
Readout of VBus devices by Arduino and transfer of data to Domoticz home automation software.

## Goal:
Readout the Resol VBus interface of an Oranier Aquacontrol III (= rebranded Resol DeltaTherm FK).

Transfer of data via ethernet to Domoticz home automation software.

## What does it do?
This sketch reads the VBus data and depending on the format of the controller decodes the data and puts it in variables.
You can then send the values via HTTP GET requests to Domoticz (Or do whatever you want with it).

## Controller support
Currently supports the following controllers:
* Resol DeltaTherm FK (0x5611)
* Oranier Aquacontrol III (0x5611)
* Conergy DT5 (0x3271)

If it does not find any of the supported controllers, it will try to decode the first 2 frames which usually contain Temp 1 to 4.

## Hardware:
* VBus RX interface circuit
* Arduino Mega + Wiznet Ethernet Shield
* Raspberry Pi with Domoticz

VBus is NOT RS485. So you need a specific converter circuit to make the VBus data readable for the Arduino UART.
See f.i. [Here](https://piamble.wordpress.com/tag/vbus/).

This sketch uses the Arduino Mega and the Wiznet 5100 ethernet shield.
You can also use another Arduino like the Uno but that one only has one hardware serial port.

Serial1 is used for the VBus module.
Serial is used to debug the output to PC. 
Vbus serial works with 9600 Baudrate and 8N1.

Arduino Mega:
* Serial  on pins  0 (RX)  and 1 (TX),
* Serial1 on pins 19 (RX) and 18 (TX),
* Serial2 on pins 17 (RX) and 16 (TX),
* Serial3 on pins 15 (RX) and 14 (TX). 

Arduino non-Mega:
You can use f.i. AltSoftSerial() instead of Serial1. This sketch should work with AltSoftSerial.
If you do use it, you need to put the VBus input on the correct softserial pin!
If you do not need the debugging option, use 'normal' Serial.

### My controller is not in the list, how can I add it?
Go to http://danielwippermann.github.io/resol-vbus/vbus-packets.html
and find your controller. In the list you can see which information the controller sends.
You need the controller ID and offset, bitsize and names of all the variables.
Now use the examples for the DT5 and FK in VBusRead() to create a new entry for your own controller.
This might be not that easy.
Do not forget to properly declare your new variables too.

If you have tested it and it works, please add a Pull request so I can integrate your controller here.

### Can the author add my controller?
No. First try it yourself. But if you fail, you can always ask.

### Is this sketch the pinnacle of proper programming?
Not by any means, but it works.
If you have any remark or improvement, let the author know.

### Should the author have made a library instead of a complete sketch?
Maybe, but this allows for better modification by myself or others. Also depending on the Arduino you need to set another Serial port. This is easier to do in the sketch.

### Why has been opted for HTTP GET requests instead of MQTT?
The author has only been very shortly aware of MQTT support in Domoticz.
Also, the interface is not documented that well at the moment.

#### Additional credits
Sketch is based on the VBus library from 'Willie' from the Mbed community.

#### Legal Notices
RESOL, VBus, VBus.net and others are trademarks or registered trademarks of RESOL - Elektronische Regelungen GmbH.

All other trademarks are the property of their respective owners.
