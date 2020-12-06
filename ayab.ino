// ayab.ino
/*
This file is part of AYAB.

    AYAB is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AYAB is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with AYAB.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2013 Christian Obersteiner, Andreas Müller
    http://ayab-knitting.com
*/


/*
 * INCLUDES
 */
#include "Arduino.h"
#include "SerialCommand.h"
#include <SoftI2CMaster.h>

#include "./libraries/PacketSerial/src/PacketSerial.h"
#include "./debug.h"
#include "./settings.h"

#include "./knitter.h"

/*
 * DEFINES
 */


/*
 *  DECLARATIONS
 */ 
Knitter     *knitter;
byte        lineBuffer[25];

SLIPPacketSerial packetSerial;

/*! Mapping of Pin EncA to its ISR
 *
 */
void isr_encA() {
  knitter->isr();
}

/*
 * Serial Command handling
 */
void h_reqStart(const uint8_t* buffer, size_t size) {
  byte _startNeedle = (byte)buffer[1];
  byte _stopNeedle  = (byte)buffer[2];
  bool _continuousReportingEnabled = (bool)buffer[3];

  // TODO verify operation
  // memset(lineBuffer,0,sizeof(lineBuffer));
  // temporary solution:
  for (int i = 0; i < 25; i++) {
    lineBuffer[i] = 0xFF;
  }

  bool _success = knitter->startOperation(_startNeedle,
                                          _stopNeedle,
                                          _continuousReportingEnabled,
                                          &(lineBuffer[0]));

  uint8_t payload[2];
  payload[0] = cnfStart_msgid;
  payload[1] = _success;
  packetSerial.send(payload, 2);
}


void h_cnfLine(const uint8_t* buffer, size_t size) {
  byte _lineNumber = 0;
  byte _flags = 0;
  byte _crc8  = 0;
  bool _flagLastLine = false;

  _lineNumber = (byte)buffer[1];

  for (int i = 0; i < 25; i++) {
    // Values have to be inverted because of needle states
    lineBuffer[i] = ~(byte)buffer[i+2];
  }
  _flags = (byte)buffer[27];
  _crc8  = (byte)buffer[28];

  // TODO insert CRC8 check

  if (knitter->setNextLine(_lineNumber)) {
    // Line was accepted
    _flagLastLine = bitRead(_flags, 0);
    if ( _flagLastLine ) {
      knitter->setLastLine();
    }
  }
 }

void h_reqInfo() {
  uint8_t payload[4];
  payload[0] = cnfInfo_msgid;
  payload[1] = API_VERSION;
  payload[2] = FW_VERSION_MAJ;
  payload[3] = FW_VERSION_MIN;
  packetSerial.send(payload, 4);
}

void h_reqTest() {
    bool _success = knitter->startTest();

    uint8_t payload[2];
    payload[0] = cnfTest_msgid;
    payload[1] = _success;
    packetSerial.send(payload, 2);
}

void h_reqI2cWrite(byte address, byte command, byte value) {
  knitter->i2c_write(address, command, value);
}

void h_reqI2cRead(byte address, byte command) {
  uint8_t payload[2];

  payload[0] = repI2cRead_msgid;
  payload[1] = knitter->i2c_read(address, command);
  packetSerial.send(payload, 2);
}

void h_unrecognized() {
  return;
}

/*! Callback for PacketSerial
 *
 */
unsigned long rxPacketLedTime=0;
void onPacketReceived(const uint8_t* buffer, size_t size)
{
  digitalWrite(LED_PIN_B, 0);
  rxPacketLedTime = millis() + 100;

  switch (buffer[0]) {
    case reqStart_msgid:
      h_reqStart(buffer, size);
      break;

    case cnfLine_msgid:
      h_cnfLine(buffer, size);
      break;

    case reqInfo_msgid:
      h_reqInfo();
      break;

    case reqTest_msgid:
      h_reqTest();
      break;

    case reqI2cWrite_msgid:
      h_reqI2cWrite(buffer[1], buffer[2], buffer[3]);
      break;

    case reqI2cRead_msgid:
      h_reqI2cRead(buffer[1], buffer[2]);
      break;

    default:
      h_unrecognized();
      break;
  }
}

/*
 * SETUP
 */
unsigned long aliveLedTime;
bool aliveLedState = true;
void setup() {
  packetSerial.begin(SERIAL_BAUDRATE);
  packetSerial.setPacketHandler(&onPacketReceived);

  knitter = new Knitter(&packetSerial);

  pinMode(ENC_PIN_A, INPUT);
  pinMode(ENC_PIN_B, INPUT);
  pinMode(ENC_PIN_C, INPUT);

  pinMode(LED_PIN_A, OUTPUT);
  pinMode(LED_PIN_B, OUTPUT);
  digitalWrite(LED_PIN_A, aliveLedState);
  digitalWrite(LED_PIN_B, 1);

  pinMode(DBG_BTN_PIN, INPUT);

  // Attaching ENC_PIN_A(=2), Interrupt No. 0
  attachInterrupt(0, isr_encA, CHANGE);

  aliveLedTime = millis() + 1000;
}


void loop() {
  unsigned long currentTime;
    
  knitter->fsm();
  packetSerial.update();

  currentTime = millis();
  if(currentTime > aliveLedTime) {
    aliveLedTime = currentTime+1000;
    aliveLedState = !aliveLedState;
    digitalWrite(LED_PIN_A, aliveLedState);
  }
  if(currentTime > rxPacketLedTime) {
    digitalWrite(LED_PIN_B, 1);
  }
}
