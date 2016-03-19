/**************************************************************************
    LocoIno - Configurable Arduino Loconet Module
    Copyright (C) 2014 Daniel Guisado Serra

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ------------------------------------------------------------------------
 AUTHOR : Dani Guisado - http://www.clubncaldes.com - dguisado@gmail.com
 ------------------------------------------------------------------------
 DESCRIPTION:
    This software emulates the functionality of a GCA50 board from Peter
    Giling (Giling Computer Applications) attached to two GCA136 to manage
    servos. 
    Configuration is done through SV Loconet protocol and can be configured
    from Rocrail (Programming->GCA->GCA50).
 ------------------------------------------------------------------------
 PIN ASSIGNMENT:
   0,1 -> Serial, used to debug and Loconet Monitor (uncomment DEBUG)
   2,3,4,5,6,9,10,11 -> Attach one servo motor to each pin
   7 -> Loconet TX (connected to GCA185 shield)
   8 -> Loconet RX (connected to GCA185 shield)
   Configure in Rocrail first 8 ports as output, and last 8 as input
   to be used as retro signaling for the switch position
 ------------------------------------------------------------------------
 CREDITS: 
 * Based on MRRwA Loconet libraries for Arduino - http://mrrwa.org/ and 
   the Loconet Monitor example.
 * Inspired in GCA50 board from Peter Giling - http://www.phgiling.net/
 * Idea also inspired in LocoShield from SPCoast - http://www.scuba.net/
 * Thanks also to Rocrail group - http://www.rocrail.org
*************************************************************************/
#include <LocoNet.h>
#include <EEPROM.h>
#include <ServoTimer2.h>

//Uncomment this line to debug through the serial monitor
#define DEBUG
#define VERSION 102

#define SVTABLE_MAX_RECORD 125
#define SERVO_LAPSE 10  //millis between servo movements

ServoTimer2 servo[8];

//3 bytes defining a pin behavior ( http://wiki.rocrail.net/doku.php?id=loconet-io-en )
typedef struct 
{
  uint8_t cnfg;
  uint8_t value1;
  uint8_t value2;
} PIN_CFG;

//Memory map exchanged with SV read and write commands ( http://wiki.rocrail.net/doku.php?id=lnsv-en )
typedef struct
{
  uint8_t vrsion;
  uint8_t addr_low;
  uint8_t addr_high;
  PIN_CFG pincfg[SVTABLE_MAX_RECORD-3];
} SV_TABLE;

//Union to access the data with the struct or by index
typedef union {
  SV_TABLE svt;
  uint8_t data[SVTABLE_MAX_RECORD];
} SV_DATA;

SV_DATA svtable;
lnMsg *LnPacket;

//This table contains the addresses already transformed in a decimal value
uint16_t directions[16];

int servoCurrentPos[8]; //current position of each servo
  
void setup()
{
  int n;
  
  // First initialize the LocoNet interface
  LocoNet.init(7);

  // Configure the serial port for 57600 baud
  #ifdef DEBUG
  Serial.begin(57600);
  Serial.print("LocoNet Monitor v.");Serial.println(VERSION);
  #endif 

  //Load config from EEPROM
  for (n=0;n<SVTABLE_MAX_RECORD;n++)
    svtable.data[n]=EEPROM.read(n);

  //Load right addresses moving the right bits
  for (n=0;n<16;n++)
  {
    //TODO set right addresses for inputs
    directions[n]=svtable.svt.pincfg[n].value1;
    bitWrite(directions[n],7,bitRead(svtable.svt.pincfg[n].value2,0));
    bitWrite(directions[n],8,bitRead(svtable.svt.pincfg[n].value2,1));
    bitWrite(directions[n],9,bitRead(svtable.svt.pincfg[n].value2,2));
  }
  
  //Attacch Servos
  servo[0].attach(2);
  servo[1].attach(3);
  servo[2].attach(4);
  servo[3].attach(5);
  servo[4].attach(6);
  servo[5].attach(9);
  servo[6].attach(10);
  servo[7].attach(11);

  pinMode(13, OUTPUT);
  
  //Check for a valid config
  if (svtable.svt.vrsion!=VERSION)
  {
    svtable.svt.vrsion=VERSION;
    svtable.svt.addr_low=81;
    svtable.svt.addr_high=1;
    EEPROM.write(0,VERSION);
    EEPROM.write(1, svtable.svt.addr_low);
    EEPROM.write(2, svtable.svt.addr_high);

    //Center servos if no previous configuration
    for (n=0;n<8;n++)
    {
      servoCurrentPos[n]=63;
      positionServo(n,63);
    }
  }
  else
  {
    //Position servos and set retro signals
    for (n=0;n<8;n++)
    {
      servoCurrentPos[n]=svtable.data[101+n*3];
      positionServo(n,servoCurrentPos[n]);
      bitWrite(svtable.svt.pincfg[n+8].value2,4,0);
      LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n+8].value1, svtable.svt.pincfg[n+8].value2);
    }
  }
}

void loop()
{  
  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if( LnPacket )
  {
    #ifdef DEBUG 
    // First print out the packet in HEX
    Serial.print("RX: ");
    uint8_t msgLen = getLnMsgSize(LnPacket); 
    for (uint8_t x = 0; x < msgLen; x++)
    {
      uint8_t val = LnPacket->data[x];
      // Print a leading 0 if less than 16 to make 2 HEX digits
      if(val < 16)
        Serial.print('0');        
      Serial.print(val, HEX);
      Serial.print(' ');
    }
    Serial.println();
    #endif  

    // If this packet was not a Switch or Sensor Message checks por PEER packet
    if (!LocoNet.processSwitchSensorMessage(LnPacket))
      processPeerPacket();
  }
}

/*************************************************************************/
/*            SERVO FUNCTIONS                                            */
/*************************************************************************/
// moves a servo transforming the angle 0-127 to pulses value range needed by the library
void positionServo(int pServoNum, int pPosition)
{
  int val=0;
  int lncvnum;
  
  val=map(pPosition,1, 127, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  servo[pServoNum].write(val);  
}

// moves a servo according to the servoDestPos[#servo]
// and the configured speed
void moveServo(int pNumServo, int pDestPos)
{
  int grades;
  int steps;

  //if servo already in desired position exit
  if (pDestPos==servoCurrentPos[pNumServo]) return;

  digitalWrite(13, HIGH);
  
  //read configuration servo speed 0 - 5
  steps=5-svtable.data[103+pNumServo*3];

  if (servoCurrentPos[pNumServo]<pDestPos)
  {
    // increment grades
    for (grades=servoCurrentPos[pNumServo];grades<=pDestPos;grades++)
    {
      positionServo(pNumServo,grades);
      delay(SERVO_LAPSE*steps);
    }
    bitWrite(svtable.svt.pincfg[pNumServo+8].value2,4,1);
    LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[pNumServo+8].value1, svtable.svt.pincfg[pNumServo+8].value2);          
  }
  else
  {
    // decrement grades
    for (grades=servoCurrentPos[pNumServo];grades>=pDestPos;grades--)
    {
      positionServo(pNumServo,grades);
      delay(SERVO_LAPSE*steps);
    }
    bitWrite(svtable.svt.pincfg[pNumServo+8].value2,4,0);
    LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[pNumServo+8].value1, svtable.svt.pincfg[pNumServo+8].value2);          
  }
  servoCurrentPos[pNumServo]=pDestPos;

  digitalWrite(13, LOW);
}

/*************************************************************************/
/*          LOCONET FUNCTIONS                                            */
/*************************************************************************/
void notifyPower( uint8_t State )
{
  int n;
  
  #ifdef DEBUG
  Serial.print("POWER: ");  
  Serial.println( State ? "ON" : "OFF" );
  #endif
  if (State)
  {
    for (n=0;n<8;n++)   
          LocoNet.send(OPC_INPUT_REP, svtable.svt.pincfg[n+8].value1, svtable.svt.pincfg[n+8].value2);
  }
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages
void notifySensor( uint16_t Address, uint8_t State )
{
  #ifdef DEBUG
  Serial.print("Sensor: ");
  Serial.print(Address, DEC);
  Serial.print(" - ");
  Serial.println( State ? "Active" : "Inactive" );
  #endif
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch Request messages
void notifySwitchRequest( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  int n;
  
  //Direction must be changed to 0 or 1, not 0 or 32
  Direction ? Direction=1 : Direction=0;
  
  #ifdef DEBUG
  Serial.print("Switch Request: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
  
  //Check if the Address is assigned, configured as output and same Direction
  for (n=0; n<8; n++)
  {    
    //if ((svtable.svt.pincfg[n].value1 == Address-1) &&  //Address
    if ((directions[n] == Address-1) &&
        (bitRead(svtable.svt.pincfg[n].cnfg,7) == 1))   //Setup as an Output
    {
      //If continue and software reset, one Direction ON turns on and other Direction ON turns off
      //OFF messages are not listened
      if (bitRead(svtable.svt.pincfg[n].cnfg,3)==0 && bitRead(svtable.svt.pincfg[n].cnfg,2)==0 && Output)
      {
        
        if (!Direction)
        {
          //Servo to one side
          moveServo(n,svtable.data[101+3*n]);
        }
        else
        {
          //Servo to the other side
          moveServo(n,svtable.data[102+3*n]);
        }
        break;
      }
    }
  }
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch Report messages
void notifySwitchReport( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print("Switch Report: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
}

  // This call-back function is called from LocoNet.processSwitchSensorMessage
  // for all Switch State messages
void notifySwitchState( uint16_t Address, uint8_t Output, uint8_t Direction )
{
  #ifdef DEBUG
  Serial.print("Switch State: ");
  Serial.print(Address, DEC);
  Serial.print(':');
  Serial.print(Direction ? "Closed" : "Thrown");
  Serial.print(" - ");
  Serial.println(Output ? "On" : "Off");
  #endif
}

/*************************************************************************/
/*          SV TABLE FUNCTIONS                                           */
/*************************************************************************/
boolean processPeerPacket()
{
  
  //Check is a OPC_PEER_XFER message
  if (LnPacket->px.command != OPC_PEER_XFER) return(false);

  #ifdef DEBUG
  Serial.println("<< OPC_PEER_XFER received...");
  #endif
  
  //Check is my destination
  if ((LnPacket->px.dst_l!=0 || LnPacket->px.d5!=0) &&
      (LnPacket->px.dst_l!=0x7f || LnPacket->px.d5!=svtable.svt.addr_high) &&
      (LnPacket->px.dst_l!=svtable.svt.addr_low || LnPacket->px.d5!=svtable.svt.addr_high))
  {
    #ifdef DEBUG
    Serial.println("OPC_PEER_XFER not for me!");
    Serial.print("LnPacket->px.dst_l: ");Serial.print(LnPacket->px.dst_l);Serial.print(" Addr low: ");Serial.println(svtable.svt.addr_low);
    Serial.print("LnPacket->px.d5: ");Serial.print(LnPacket->px.d5);Serial.print(" Addr high: ");Serial.println(svtable.svt.addr_high);
    Serial.print("LnPacket->px.dst_h: ");Serial.print(LnPacket->px.dst_h);Serial.print(" Addr high: ");Serial.println(svtable.svt.addr_high);
    Serial.print("LnPacket->px.d1: ");Serial.println(LnPacket->px.d1);
    Serial.print("LnPacket->px.d2: ");Serial.println(LnPacket->px.d2);
    #endif
    return(false);
  }  

  //Set high bits in right position
  bitWrite(LnPacket->px.d1,7,bitRead(LnPacket->px.pxct1,0));
  bitWrite(LnPacket->px.d2,7,bitRead(LnPacket->px.pxct1,1));
  bitWrite(LnPacket->px.d3,7,bitRead(LnPacket->px.pxct1,2));
  bitWrite(LnPacket->px.d4,7,bitRead(LnPacket->px.pxct1,3));
  
  bitWrite(LnPacket->px.d5,7,bitRead(LnPacket->px.pxct2,0));
  bitWrite(LnPacket->px.d6,7,bitRead(LnPacket->px.pxct2,1));
  bitWrite(LnPacket->px.d7,7,bitRead(LnPacket->px.pxct2,2));
  bitWrite(LnPacket->px.d8,7,bitRead(LnPacket->px.pxct2,3));

  //OPC_PEER_XFER D1 -> Command (1 SV write, 2 SV read)
  //OPC_PEER_XFER D2 -> Register to read or write
  if (LnPacket->px.d1==2)
  {
    #ifdef DEBUG
    Serial.print("READ ");Serial.print(LnPacket->px.d2);Serial.print(" ");Serial.print(LnPacket->px.d2+1);Serial.print(" ");Serial.println(LnPacket->px.d2+2);
    #endif
    delay(50);
    sendPeerPacket(svtable.data[LnPacket->px.d2], svtable.data[LnPacket->px.d2+1], svtable.data[LnPacket->px.d2+2]);
    #ifdef DEBUG
    Serial.println(">> OPC_PEER_XFER answer sent");
    Serial.println("=============================================");
    #endif
    return (true);
  }
  
  //Write command
  if (LnPacket->px.d1==1)
  {
    //SV 0 contains the program version (write SV0 == RESET? )
    if (LnPacket->px.d2>0)
    {
      //Store data
      svtable.data[LnPacket->px.d2]=LnPacket->px.d4;
      EEPROM.write(LnPacket->px.d2,LnPacket->px.d4);

      //set servo position if servo config command
      if (LnPacket->px.d2==103)
      {
        moveServo(0,svtable.data[101]);
        delay(1000);
        moveServo(0,svtable.data[102]);
      }
      if (LnPacket->px.d2==106)
      {
        moveServo(1,svtable.data[104]);
        delay(1000);
        moveServo(1,svtable.data[105]);
      }
      if (LnPacket->px.d2==109)
       {
        moveServo(2,svtable.data[107]);
        delay(1000);
        moveServo(2,svtable.data[108]);
      }
      if (LnPacket->px.d2==112)
        {
        moveServo(3,svtable.data[110]);
        delay(1000);
        moveServo(3,svtable.data[111]);
      }
      if (LnPacket->px.d2==115)
      {
        moveServo(4,svtable.data[113]);
        delay(1000);
        moveServo(4,svtable.data[114]);
      }
      if (LnPacket->px.d2==118)
      {
        moveServo(5,svtable.data[116]);
        delay(1000);
        moveServo(5,svtable.data[117]);
      }
      if (LnPacket->px.d2==121)
      {
        moveServo(6,svtable.data[119]);
        delay(1000);
        moveServo(6,svtable.data[120]);
      }
      if (LnPacket->px.d2==124)
      {
        moveServo(7,svtable.data[122]);
        delay(1000);
        moveServo(7,svtable.data[123]);
      }
        
      #ifdef DEBUG
      Serial.print("ESCRITURA "); Serial.print(LnPacket->px.d2); Serial.print(" <== ");
      Serial.print(LnPacket->px.d4); Serial.print(" | ");
      Serial.print(LnPacket->px.d4, HEX); Serial.print(" | ");
      Serial.println(LnPacket->px.d4, BIN);
      #endif
    }

    //Answer packet        
    delay(50);
    sendPeerPacket(0x00, 0x00, LnPacket->px.d4);
    #ifdef DEBUG
    Serial.println(">> OPC_PEER_XFER answer sent");
    Serial.println("=============================================");
    #endif
    return (true);
  }
  
  return (false);
  
}

void sendPeerPacket(uint8_t p0, uint8_t p1, uint8_t p2)
{
  lnMsg txPacket;

  txPacket.px.command=OPC_PEER_XFER;
  txPacket.px.mesg_size=0x10;
  txPacket.px.src=svtable.svt.addr_low;
  txPacket.px.dst_l=LnPacket->px.src;
  txPacket.px.dst_h=LnPacket->px.dst_h; 
  txPacket.px.pxct1=0x00;
  txPacket.px.d1=LnPacket->px.d1;  //Original command
  txPacket.px.d2=LnPacket->px.d2;  //SV requested
  txPacket.px.d3=svtable.svt.vrsion;
  txPacket.px.d4=0x00;
  txPacket.px.pxct2=0x00;
  txPacket.px.d5=svtable.svt.addr_high; //SOURCE high address
  txPacket.px.d6=p0;
  txPacket.px.d7=p1;
  txPacket.px.d8=p2;

  //Set high bits in right position  
  bitWrite(txPacket.px.pxct1,0,bitRead(txPacket.px.d1,7));
  bitClear(txPacket.px.d1,7);
  bitWrite(txPacket.px.pxct1,1,bitRead(txPacket.px.d2,7));
  bitClear(txPacket.px.d2,7);
  bitWrite(txPacket.px.pxct1,2,bitRead(txPacket.px.d3,7));
  bitClear(txPacket.px.d3,7);
  bitWrite(txPacket.px.pxct1,3,bitRead(txPacket.px.d4,7));
  bitClear(txPacket.px.d4,7);
  bitWrite(txPacket.px.pxct2,0,bitRead(txPacket.px.d5,7));
  bitClear(txPacket.px.d5,7);
  bitWrite(txPacket.px.pxct2,1,bitRead(txPacket.px.d6,7));
  bitClear(txPacket.px.d6,7);
  bitWrite(txPacket.px.pxct2,2,bitRead(txPacket.px.d7,7));
  bitClear(txPacket.px.d7,7);
  bitWrite(txPacket.px.pxct2,3,bitRead(txPacket.px.d8,7));
  bitClear(txPacket.px.d8,7);
   
  LocoNet.send(&txPacket);
  
  #ifdef DEBUG
  Serial.println("OPC_PEER_XFER Packet sent!");
  #endif
}
