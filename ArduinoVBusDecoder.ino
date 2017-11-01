/* VBus decoder to Domoticz sketch for Arduino
 * * 
 * Version 1.1 - August 2017 - Added Joule / Resol Deltasol C (credits: Fatbeard)
 * Version 1.0 - January 2016
 * 
 * Copyright (c) 2016 - 'bbqkees' @ www.domoticz.com/forum
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Legal Notices
 * RESOL, VBus, VBus.net and others are trademarks or registered trademarks of RESOL - Elektronische Regelungen GmbH.
 * All other trademarks are the property of their respective owners.
 * 
 * 
 * * What does it do?
 * This sketch reads the VBus data and depending on the format of the controller decodes
 * the data and puts in in variables.
 * You can then send the values via HTTP GET requests to Domoticz.
 * 
 * In this sketch the VBus data is read out continously and send periodically to Domoticz
 * If the VBus is read out periodically, it sometimes reads nothing and you get all zero's.
 * 
 * * Currently supports the following controllers:
 * -Resol DeltaTherm FK (a.k.a. Oranier Aquacontrol III) 
 * -Conergy DT5
 * -Joule / Resol Deltasol C
 * If it does not find any of the supported controllers,
 * it will try to decode the first 2 frames which usually contain Temp 1 to 4.
 * 
 * * My controller is not in your list, how can I add it?
 * Go to http://danielwippermann.github.io/resol-vbus/vbus-packets.html
 * and find your controller. In the list you can see which information the controller sends.
 * You need the controller ID and offset, bitsize and names of all the variables.
 * Now use the examples for the DT5 and FK in VBusRead()
 * to create a new entry for your own controller.
 * 
 * Sketch is based on VBus library from 'Willie' from the Mbed community.
 * 
 * * Hardware info:
 * VBus is NOT RS485. So you need a specific converter circuit to make the VBus data
 * readable for the Arduino UART.
 * See the Domoticz Wiki or my GitHub for a working example circuit.
 * 
 * This sketch uses the Arduino Mega and the Wiznet 5100 ethernet shield.
 * You can also use another Arduino like the Uno but it only has one hardware serial port.
 *  
 * Serial1 is used for the VBus module.
 * Serial is used to debug the output to PC. 
 * Vbus serial works with 9600 Baudrate and 8N1.
 * 
 * Arduino Mega:
 * Serial  on pins  0 (RX)  and 1 (TX),
 * Serial1 on pins 19 (RX) and 18 (TX),
 * Serial2 on pins 17 (RX) and 16 (TX),
 * Serial3 on pins 15 (RX) and 14 (TX). 
 * 
 * Arduino non-Mega:
 * You can use f.i. AltSoftSerial() instead of Serial1.
 * This sketch should work with AltSoftSerial.
 * If you do use it, you need to put the VBus input on the correct softserial pin!
 * If you do not need the debugging option, use 'normal' Serial.
 * 
 * * Is this sketch the pinnacle of proper programming?
 * Not by any means, but it works.
 * 
 * * Should I have made a library instead of a sketch?
 * Maybe, but this allows for better modification by myself or others.
 * Also depending on the Arduino you need to set another Serial port.
 * This is easier to do in the sketch.
 * 
 */

 /* Other settings:
  * 
  * Domoticz JSON URL's:
  * Temp
  * /json.htm?type=command&param=udevice&idx=IDX&nvalue=0&svalue=TEMP
  * Temp/Hum
  * /json.htm?type=command&param=udevice&idx=IDX&nvalue=0&svalue=TEMP;HUM;HUM_STAT
  * Switch on
  * /json.htm?type=command&param=switchlight&idx=99&switchcmd=On
  * Switch off
  * /json.htm?type=command&param=switchlight&idx=99&switchcmd=Off
  * 
  */

// SPI and Ethernet are used for Ethernet shield
#include <SPI.h>
#include <Ethernet.h>

// Ethernet uses 10,11,12,13
// and 4 for the SD card.

// MAC and IP of the Arduino
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0x30, 0x39, 0x32};
IPAddress ip(0,0,0,0); //The IP addres of this Arduino

// IP address and port of Domoticz
IPAddress domoticz(192,168,0,0); //The IP addres of the Domoticz server
int port = 8080;
EthernetClient client;

long lastTime = 0;        // will store last time 
long lastTimedht = 0;
long lastTimetemp = 0;
long lastTimevbus = 0;

long intervalvbus = 30000;           // interval at which to send data (milliseconds)


// Change the IDX values below to your own!

int IDXtempBuffer1 =  111; //(Resol)
int IDXtempBuffer4 = 111; //(Resol)
int IDXtempHaardUit = 111; //(Resol)
int IDXtempCvRetour = 111; //(Resol)
int IDXrelaisHaardPomp = 111; //(Resol)
int IDXrelaisHaard3Weg = 111; //(Resol)
int IDXsytemNotification = 111; //(Resol)

String relayPump = "off";
String relay3WayValve = "off";
String SystemAlert = "off";

// Settings for the VBus decoding
#define Sync  0xAA  // Synchronisation bytes
#define FLength 6   // Framelength
#define FOffset 10  // Offset start of Frames
#define FSeptet 4   // Septet byte in Frame
#define ResolAddress 0x3271  //   ConergyDT5 (0x3271) Original default Address of the controller 
#define SENSORNOTCONNECTED 8888 // Sometimes this might be 888 instead.

uint16_t networkaddress;
float Sensor1_temp;
float Sensor2_temp;
float Sensor3_temp;
float Sensor4_temp;

float Sensor1_temp_max;
float Sensor2_temp_max;
float Sensor3_temp_max;
float Sensor4_temp_max;

  // Conergy DT5 specific
  char PumpSpeed1;  // in  %
  char PumpSpeed2;  //  in %
  char RelaisMask;
  char ErrorMask;
  char Scheme;
  char OptionPostPulse;
  char OptionThermostat;
  char OptionHQM;
  uint16_t OperatingHoursRelais1;
  uint16_t OperatingHoursRelais2;
  uint32_t HeatQuantity;
  uint16_t Version;
  uint16_t OperatingHoursRelais1Today;
  uint16_t SystemTime;
  //


  //Resol DeltaTherm FK specific
  char Relay1;  // in  %
  char Relay2;  //  in %
  char MixerOpen; // in  %
  char MixerClosed; // in  %
  char SystemNotification;
  //

  // These are set neither to 'on' or 'off' at initialization so at startup the value
  // from the first valid datagram will be sent to Domoticz.
  String lastRelay1 = "1";
  String lastRelay2 = "1" ;
  String lastSystemAlert = "1" ;

unsigned char Buffer[80];
volatile unsigned char Bufferlength;

unsigned int Destination_address;
unsigned int Source_address;
unsigned char ProtocolVersion;
unsigned int Command;
unsigned char Framecnt;
unsigned char Septet;
unsigned char Checksum;

long lastTimeTimer;
long timerInterval;
bool all;

// Set to "1" when debugging 
// If enabled, the Arduino sends the decoded values over the Serial port.
// If enabled it also prints the source address in case you do not
// know what controller(s) you have.
#define DEBUG 0

// Clear all maximum values
void VBusClearMax() {
    Sensor1_temp_max=0.0;
    Sensor2_temp_max=0.0;
    Sensor3_temp_max=0.0;
    Sensor4_temp_max=0.0;
}

// Initialize some parameters
// Currently setting the controller does nothing special.
void VBusInit(int addr=0) {
    timerInterval=2000;
    if (addr!=0)
        networkaddress=addr;
    else
        networkaddress=ResolAddress;

}

void setup() {
Serial1.begin(9600);
#if DEBUG
Serial.begin(9600);
Serial.println("Arduino debugging started");
#endif

Ethernet.begin(mac, ip);

all=false;
VBusClearMax();
VBusInit(0x5611); //0 for Conergy DT5, 0x5611 for DeltaTherm FK
} // end void setup()

void loop() {

if (VBusRead()){
  #if DEBUG
        Serial.println("------Decoded VBus data------");
        Serial.print("Destination: ");
        Serial.println(Destination_address, HEX);
        Serial.print("Source: ");
        Serial.println(Source_address, HEX);
        Serial.print("Protocol Version: ");
        Serial.println(ProtocolVersion);
        Serial.print("Command: ");
        Serial.println(Command, HEX);
        Serial.print("Framecount: ");
        Serial.println(Framecnt);
        Serial.print("Checksum: ");
        Serial.println(Checksum);
        Serial.println("------Values------");
        Serial.print("Sensor 1: ");
        Serial.println(Sensor1_temp);
        Serial.print("Sensor 2: ");
        Serial.println(Sensor2_temp);
        Serial.print("Sensor 3: ");
        Serial.println(Sensor3_temp);
        Serial.print("Sensor 4: ");
        Serial.println(Sensor4_temp);
        Serial.print("Relay 1: ");
        Serial.println(Relay1, DEC);
        Serial.print("Relay 2: ");
        Serial.println(Relay2, DEC);
        Serial.println(SystemNotification, DEC);
        Serial.println("------END------");
#endif
} //end VBusRead

/*
 * S1 = Sensor 1 (sensor SFB/stove)
   S2 = Sensor 2 (sensor store base)
   S3 = Sensor 3 (sensor store top)
   S4 = Sensor 4 (system-dependent)
 * R1 = Pump
 * R2 = 3-way valve
 */

  // loop for VBus readout and http GET requests
  // This loop is executed every intervalvbus milliseconds.
  if(millis() - lastTimevbus > intervalvbus) {
    lastTimevbus = millis(); 

// Convert relay value to On or Off.
if (Relay1 == 0x64){relayPump = "On";}
else if (Relay1 == 0x00){relayPump = "Off";}
else {relayPump = "Off";}

if (Relay2 == 0x64){relay3WayValve = "On";}
else if (Relay2 == 0x00){relay3WayValve = "Off";}
else {relay3WayValve = "Off";}

if (SystemNotification != 0x00){SystemAlert = "On";}
else if (SystemNotification == 0x00){SystemAlert = "Off";}
else {SystemAlert = "Off";}

// In the first (few) VBus readout all values are zero.
// In order to prevent we are sending incorrect data to Domoticz we check it is not zero.

// Only send relay status to Domoticz if it's state has changed
if (Sensor1_temp != 0 && relayPump != lastRelay1){
httpRequestSwitch(IDXrelaisHaardPomp, relayPump);
lastRelay1 = relayPump;
}

if (Sensor1_temp != 0 && relay3WayValve != lastRelay2){
httpRequestSwitch(IDXrelaisHaard3Weg, relay3WayValve);
lastRelay2 = relay3WayValve;
}

if (Sensor3_temp != 0){httpRequestTemp(IDXtempBuffer1, Sensor3_temp);}
if (Sensor2_temp != 0){httpRequestTemp(IDXtempBuffer4, Sensor2_temp);}
if (Sensor1_temp != 0){httpRequestTemp(IDXtempHaardUit, Sensor1_temp);}
if (Sensor4_temp != 0){httpRequestTemp(IDXtempCvRetour, Sensor4_temp);}

// Only send a alert if the value has changed.
if (Sensor1_temp != 0 && SystemAlert != lastSystemAlert){
httpRequestSwitch(IDXrelaisHaardPomp, SystemAlert);
lastSystemAlert = SystemAlert;
}
} //end loop VBus readout

  
}// end void loop()


#if DEBUG
#endif

// The following is needed for decoding the data
void  InjectSeptet(unsigned char *Buffer, int Offset, int Length) {
    for (unsigned int i = 0; i < Length; i++) {
        if (Septet & (1 << i)) {
            Buffer [Offset + i] |= 0x80;
        }
    }
}


// The following function reads the data from the bus and converts it all
// depending on the used VBus controller.
bool VBusRead() {
    int F;
    char c;
    bool start,stop,quit;

    start = true;
    stop = false;
    quit = false;
    Bufferlength=0;
    lastTimeTimer = 0;
    lastTimeTimer = millis();

    while ((!stop) and (!quit))  {
          if (Serial1.available()) {
              c=Serial1.read();
            
char sync1 = 0xAA;
            if (c == sync1) {

#if DEBUG
           // Serial.println("Sync found");
#endif
              
                if (start) {
                    start=false;
                    Bufferlength=0;
//#if DEBUG
//#endif
                } else {
                    if (Bufferlength<20) {
                       lastTimeTimer = 0;
                       lastTimeTimer = millis();
                        Bufferlength=0;
                    } else
                        stop=true;
                }
            }
#if DEBUG
           // Serial.println(c, HEX);
#endif
            if ((!start) and (!stop)) {
                Buffer[Bufferlength]=c;
                Bufferlength++;
            }
        }
        if ((timerInterval > 0) &&  (millis() - lastTimeTimer > timerInterval )  ) {
            quit=true;
#if DEBUG
         //   Serial.print("Timeout: ");
         //   Serial.println(lastTimeTimer);
#endif
        }
    }

   lastTimeTimer = 0;

    if (!quit) {
        Destination_address = Buffer[2] << 8;
        Destination_address |= Buffer[1];
        Source_address = Buffer[4] << 8;
        Source_address |= Buffer[3];
        ProtocolVersion = (Buffer[5]>>4) + (Buffer[5] &(1<<15));

        Command = Buffer[7] << 8;
        Command |= Buffer[6];
        Framecnt = Buffer[8];
        Checksum = Buffer[9];  //TODO check if Checksum is OK
#if DEBUG
        Serial.println("---------------");
        Serial.print("Destination: ");
        Serial.println(Destination_address, HEX);
        Serial.print("Source: ");
        Serial.println(Source_address, HEX);
        Serial.print("Protocol Version: ");
        Serial.println(ProtocolVersion);
        Serial.print("Command: ");
        Serial.println(Command, HEX);
        Serial.print("Framecount: ");
        Serial.println(Framecnt);
        Serial.print("Checksum: ");
        Serial.println(Checksum);
        Serial.println("---------------");
       
#endif
        // Only analyse Commands 0x100 = Packet Contains data for slave
        // with correct length = 10 bytes for HEADER and 6 Bytes  for each frame

        if ((Command==0x0100) and (Bufferlength==10+Framecnt*6)) {

          //Only decode the data from the correct source address
          //(There might be other VBus devices on the same bus).
          
          if (Source_address ==0x3271){
#if DEBUG
        Serial.println("---------------");
        Serial.println("Now decoding for 0x3271");
        Serial.println("---------------");
       
#endif

            // Frame info for the Resol ConergyDT5
            // check VBusprotocol specification for other products

            // This library is made for the ConergyDT5 (0x3271)

            //Offset  Size    Mask    Name                    Factor  Unit
            //0       2               Temperature sensor 1    0.1     &#65533;C
            //2       2               Temperature sensor 2    0.1     &#65533;C
            //4       2               Temperature sensor 3    0.1     &#65533;C
            //6       2               Temperature sensor 4    0.1     &#65533;C
            //8       1               Pump speed pump         1       1
            //9       1               Pump speed pump 2       1
            //10      1               Relay mask              1
            //11      1               Error mask              1
            //12      2               System time             1
            //14      1               Scheme                  1
            //15      1       1       Option PostPulse        1
            //15      1       2       Option thermostat       1
            //15      1       4       Option HQM              1
            //16      2               Operating hours relay 1 1
            //18      2               Operating hours relay 2 1
            //20      2               Heat quantity           1       Wh
            //22      2               Heat quantity           1000    Wh
            //24      2               Heat quantity           1000000 Wh
            //26      2               Version 0.01
            //
            // Each frame has 6 bytes
            // byte 1 to 4 are data bytes -> MSB of each bytes
            // byte 5 is a septet and contains MSB of bytes 1 to 4
            // byte 6 is a checksum
            //
            //*******************  Frame 1  *******************

            F=FOffset;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
            Sensor1_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
            Sensor2_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);

            //*******************  Frame 2  *******************
            F=FOffset+FLength;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            Sensor3_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            Sensor4_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);

            //*******************  Frame 3  *******************
            F=FOffset+FLength*2;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            PumpSpeed1 = (Buffer[F] & 0X7F);
            PumpSpeed2 = (Buffer[F+1] & 0X7F);
            RelaisMask = Buffer[F+2];
            ErrorMask  = Buffer[F+3];

            //*******************  Frame 4  *******************
            F=FOffset+FLength*3;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            SystemTime = Buffer[F+1] << 8 | Buffer[F];
            Scheme    =  Buffer[F+2];

            OptionPostPulse  = (Buffer[F+3] & 0x01);
            OptionThermostat = ((Buffer[F+3] & 0x02) >> 1);
            OptionHQM  = ((Buffer[F+3] & 0x04) >> 2);

            //*******************  Frame 5  *******************
            F=FOffset+FLength*4;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            OperatingHoursRelais1=Buffer[F+1] << 8 | Buffer[F];
            OperatingHoursRelais2=Buffer[F+3] << 8| Buffer[F+2];

            //*******************  Frame 6  *******************
            F=FOffset+FLength*5;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            HeatQuantity=(Buffer[F+1] << 8 | Buffer[F])+(Buffer[F+3] << 8| Buffer[F+2])*1000;

            //*******************  Frame 7  *******************
            F=FOffset+FLength*6;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            HeatQuantity=HeatQuantity+(Buffer[F+1] << 8 | Buffer[F])*1000000;
            Version=Buffer[F+3] << 8| Buffer[F+2];

            ///******************* End of frames ****************

           }// end 0x3271 Conenergy DT5
           
           else if (Source_address ==0x5611){
#if DEBUG
        Serial.println("---------------");
        Serial.println("Now decoding for 0x5611");
        Serial.println("---------------");
       
#endif
            // Frame info for the Resol Deltatherm FK and Oranier Aquacontrol III
            // check VBusprotocol specification for other products

            // 

            //Offset  Size    Mask    Name                    Factor  Unit
            // Frame 1
            //0       2               Temperature sensor 1    0.1     &#65533;C
            //2       2               Temperature sensor 2    0.1     &#65533;C
            // Frame 2
            //4       2               Temperature sensor 3    0.1     &#65533;C
            //6       2               Temperature sensor 4    0.1     &#65533;C
            // Frame 3
            //8       1               Relay 1                 1       %
            //9       1               Relay 2                 1       %  
            //10      1               Mixer open              1       %
            //11      1               Mixer closed            1       %
            // Frame 4
            //12      4               System date             1
            // Frame 5
            //16      2               System time             1       
            //18      1               System notification     1
            //
            // Each frame has 6 bytes
            // byte 1 to 4 are data bytes -> MSB of each bytes
            // byte 5 is a septet and contains MSB of bytes 1 to 4
            // byte 6 is a checksum
            //
            //*******************  Frame 1  *******************

            F=FOffset;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            Sensor1_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            Sensor2_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);
            Serial.println( Buffer[F], HEX);
            Serial.println( Buffer[F+1], HEX);
            Serial.println( Buffer[F+2], HEX);
            Serial.println( Buffer[F+3], HEX);

            //*******************  Frame 2  *******************
            F=FOffset+FLength;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            Sensor3_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            Sensor4_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);
            Serial.println( Buffer[F], HEX);
            Serial.println( Buffer[F+1], HEX);
            Serial.println( Buffer[F+2], HEX);
            Serial.println( Buffer[F+3], HEX);
            //*******************  Frame 3  *******************
            F=FOffset+FLength*2;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            // Some of the values are 7 bit instead of 8.
            // Adding '& 0x7F' means you are only interested in the first 7 bits.
            // 0x7F = 0b1111111.
            // See: http://stackoverflow.com/questions/9552063/c-language-bitwise-trick
            Relay1 = (Buffer[F] & 0X7F);
            Relay2 = (Buffer[F+1] & 0X7F);
            MixerOpen = (Buffer[F+2] & 0X7F);
            MixerClosed  = (Buffer[F+3] & 0X7F);
            //*******************  Frame 4  *******************
            F=FOffset+FLength*3;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            // System date is not needed for Domoticz
            
            //*******************  Frame 5  *******************
            F=FOffset+FLength*4;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            // System time is not needed for Domoticz

            // Status codes System Notification according to Resol:
            //0: no error / warning
            //1: S1 defect
            //2: S2 defect
            //3: S3 defect
            //4: VFD defect
            //5: Flow rate?
            //6: ΔT too high
            //7: Low water level
            
            SystemNotification = Buffer[F+2];
            
            ///******************* End of frames ****************

           } //End 0x5611 Resol DeltaTherm FK 

           
            else if (Source_address == 0x4212) {
      #if DEBUG
              Serial.println("---------------");
              Serial.println("Now decoding for DeltaSol C 0x4212");
              Serial.println("---------------");
      
      #endif
      
              // Frame info for the Resol DeltaSol C (Joule)
              // check VBusprotocol specification for other products
      
              // This library is made for the Resol DeltaSol C (0x4212)
      
         
              //Offset  Mask        Name                Factor      Unit
              //0                   Temperature S1      1.0         °C
              //1                   Temperature S1      256.0       °C
              //2                   Temperature S2      1.0         °C
              //3                   Temperature S2      256.0       °C
              //4                   Temperature S3      1.0         °C
              //5                   Temperature S3      256.0       °C
              //6                   Temperature S4      1.0         °C
              //7                   Temperature S4      256.0       °C
              //8                   Pump Speed R1       1           %
              //9                   Pump Speed R2       1           %
              //10                  Error Mask          1  
              //11                  Scheme              1  
              //12                  Operating Hours R1  1           h
              //13                  Operating Hours R1  256         h
              //14                  Operating Hours R2  1           h
              //15                  Operating Hours R2  256         h
              //16                  Heat Quantity       1           Wh
              //17                  Heat Quantity       256         Wh
              //18                  Heat Quantity       1000        Wh
              //19                  Heat Quantity       256000      Wh
              //20                  Heat Quantity       1000000     Wh
              //21                  Heat Quantity       256000000   Wh
              //22                  Minute of Day       1  
              //23                  Minute of Day       256 
              
              //
              // Each frame has 6 bytes
              // byte 1 to 4 are data bytes -> MSB of each bytes
              // byte 5 is a septet and contains MSB of bytes 1 to 4
              // byte 6 is a checksum
              //
              //*******************  Frame 1  *******************
      
              F = FOffset;
      
              Septet = Buffer[F + FSeptet];
              InjectSeptet(Buffer, F, 4);
      
              // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
              Sensor1_temp = CalcTemp(Buffer[F + 1], Buffer[F]);
              // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
              Sensor2_temp = CalcTemp(Buffer[F + 3], Buffer[F + 2]);
      
              //*******************  Frame 2  *******************
              F = FOffset + FLength;
      
              Septet = Buffer[F + FSeptet];
              InjectSeptet(Buffer, F, 4);
      
              Sensor3_temp = CalcTemp(Buffer[F + 1], Buffer[F]);
              Sensor4_temp = CalcTemp(Buffer[F + 3], Buffer[F + 2]);
      
              //*******************  Frame 3  *******************
              F = FOffset + FLength * 2;
      
              Septet = Buffer[F + FSeptet];
              InjectSeptet(Buffer, F, 4);
      
	  	      Relay1 = (Buffer[F] );
	          Relay2 = (Buffer[F + 1]);
	          ErrorMask  = Buffer[F + 2]; //This is the notification
	          Scheme    =  Buffer[F + 3];
      
              //*******************  Frame 4  *******************
              F = FOffset + FLength * 3;
      
              Septet = Buffer[F + FSeptet];
              InjectSeptet(Buffer, F, 4);
      
              OperatingHoursRelais1 = Buffer[F + 1] << 8 | Buffer[F];
              OperatingHoursRelais2    =  Buffer[F + 3] << 8 | Buffer[F + 2];;
      
              //*******************  Frame 5  *******************
              F = FOffset + FLength * 4;
      
              Septet = Buffer[F + FSeptet];
              InjectSeptet(Buffer, F, 4);
      
              HeatQuantity = (Buffer[F + 1] << 8 | Buffer[F]) + (Buffer[F + 3] << 8 | Buffer[F + 2]) * 1000;
      
              //*******************  Frame 6  *******************
              F = FOffset + FLength * 5;
      
              Septet = Buffer[F + FSeptet];
              InjectSeptet(Buffer, F, 4);
      
              HeatQuantity = HeatQuantity + (Buffer[F + 1] << 8 | Buffer[F]) * 1000000;
       
      
              SystemTime = Buffer[F + 3] << 8 | Buffer[F + 2];
        
              ///******************* End of frames ****************
      
            }// end 0x4212 DeltaSol C 

           /* Add your own controller ID and code in the if statement below and uncomment
           else if (Source_address ==0x????){
           }
           */
           
           else {

            // Default temp 1-4 extraction
            // For most Resol controllers temp 1-4 are always available, so
            // even if you do not know the datagram format you can still see
            // these temps 1 to 4.

            // 

            //Offset  Size    Mask    Name                    Factor  Unit
            // Frame 1
            //0       2               Temperature sensor 1    0.1     &#65533;C
            //2       2               Temperature sensor 2    0.1     &#65533;C
            // Frame 2
            //4       2               Temperature sensor 3    0.1     &#65533;C
            //6       2               Temperature sensor 4    0.1     &#65533;C
            //
            // Each frame has 6 bytes
            // byte 1 to 4 are data bytes -> MSB of each bytes
            // byte 5 is a septet and contains MSB of bytes 1 to 4
            // byte 6 is a checksum
            //
            //*******************  Frame 1  *******************

            F=FOffset;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
            Sensor1_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
            Sensor2_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);

            //*******************  Frame 2  *******************
            F=FOffset+FLength;

            Septet=Buffer[F+FSeptet];
            InjectSeptet(Buffer,F,4);

            Sensor3_temp =CalcTemp(Buffer[F+1], Buffer[F]);
            Sensor4_temp =CalcTemp(Buffer[F+3], Buffer[F+2]);

            ///******************* End of frames ****************

           } //End of Default temp 1-4 extraction

            if (Sensor1_temp>Sensor1_temp_max)
                Sensor1_temp_max=Sensor1_temp;
            if (Sensor2_temp>Sensor2_temp_max)
                Sensor2_temp_max=Sensor2_temp;
            if (Sensor3_temp>Sensor3_temp_max)
                Sensor3_temp_max=Sensor3_temp;
            if (Sensor4_temp>Sensor4_temp_max)
                Sensor4_temp_max=Sensor4_temp;
          
        } // end if command 0x0100
    } // end !quit

    return !quit;
} // end VBusRead()


// This function converts 2 data bytes to a temperature value.
float CalcTemp(int Byte1, int Byte2) {
   int v;
    v = Byte1 << 8 | Byte2; //bit shift 8 to left, bitwise OR

    if (Byte1 == 0x00){
    v= v & 0xFF;  
    }

    if (Byte1 == 0xFF)
        v = v - 0x10000;

    if (v==SENSORNOTCONNECTED)
        v=0;

    return (float)((float) v * 0.1);
    }    

//The part below contain all the httprequest functions.

// Send a temperature value to Domoticz
// /json.htm?type=command&param=udevice&idx=IDX&nvalue=0&svalue=TEMP
void httpRequestTemp(int IDX, float temp) {
  // if there's a successful connection:
  if (client.connect(domoticz, port)) {
    client.print( "GET /json.htm?type=command&param=udevice&idx=");
    client.print(IDX);
    client.print("&nvalue=0&svalue=");
    client.print(temp);
    client.println( " HTTP/1.1");
    client.println( "Host: 192.168.0.0"); //change to your IP
    client.println( "Connection: close");
    client.println();

    client.println();
    client.stop();
    delay(150);
  } 
  else {
    client.stop();
  }
}

// Send a switch command to Domoticz
// /json.htm?type=command&param=switchlight&idx=XX&switchcmd=On
void httpRequestSwitch(int IDX, String status) {
  // if there's a successful connection:
  if (client.connect(domoticz, port)) {
    client.print( "GET /json.htm?type=command&param=switchlight&idx=");
    client.print(IDX);
    client.print("&switchcmd=");
    client.print(status);
    client.println( " HTTP/1.1");
    client.println( "Host: 192.168.0.0"); //change to your IP
    client.println( "Connection: close");
    client.println();

    client.println();
    client.stop();
    delay(150);
  } 
  else {
    client.stop();
  }
}

// Total heat value to Domoticz in Wh
// Only send the total from start, not the daily or weekly!
// Domoticz will calculate daily production by itself.
// json.htm?type=command&param=udevice&idx=IDX&svalue=COUNTER
void httpRequestPower(int IDX, float counter) {
  // if there's a successful connection:
  if (client.connect(domoticz, port)) {
    client.print( "GET json.htm?type=command&param=udevice&idx=");
    client.print(IDX);
    client.print("&svalue=");
    client.print(counter);
    client.println( " HTTP/1.1");
    client.println( "Host: 192.168.0.0"); //change to your IP
    client.println( "Connection: close");
    client.println();

    client.println();
    client.stop();
    delay(150);
  } 
  else {
    client.stop();
  }
}
