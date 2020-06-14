#ifndef CPP_MEMORYCONTROL
  #define CPP_MEMORYCONTROL

#include <Arduino.h>

//EEPROM Libraries
#include <Wire.h> //for saving to external chip eeprom
#include <EEPROM.h> //for saving to internal eeprom memory

#include "globals.h"
#include "memoryControl.h"
#include "midiControl.cpp"


//-------------------------------------//
//-- INTERNAL/EXTERNAL EEPROM SAVING --//
//-------------------------------------//



void saveTrack(byte diskAddress, String saveType) {

  int slotOffset =  (saveLoadSlot * 128); //128 bytes allocated for each track save
  
  //unneeded? - depreciated becaus of I2C use rather than internal EEPROM
  //int eepromAddressHelper = SAVE_BYTE_ADDRESS + slotOffset;
  
  
  if (saveType == "load") {
    byte saveByteCheck = pullEepromByte(diskAddress, SAVE_BYTE_ADDRESS + slotOffset, 1);
    if (saveByteCheck == VALID_SAVE_BYTE) {
      notesOff();
      trackLength = pullEepromByte(diskAddress, TRACK_LENGTH_ADDRESS + slotOffset, 1);
      tempo = pullEepromByte(diskAddress, TEMPO_ADDRESS + slotOffset, 2);
      int eepromAddressHelper = TRACK_CONTAINER_ADDRESS + slotOffset; 
      for(int trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){
        for(int valueCycler = 0; valueCycler < TRACK_VARIABLE_QTY; valueCycler++) {
          trackContainer[trackCycler][valueCycler] = pullEepromByte(diskAddress, eepromAddressHelper, 1);
          eepromAddressHelper += 1;
        }
      }
    } else {
      notesOff();
      byte emptyTrackContainer[TRACK_QTY][TRACK_VARIABLE_QTY] =  { {1,63,0,1,0,1,63,8,0,0}, {2,63,0,0,0,1,63,8,0,0}, {3,63,0,0,0,1,63,8,0,0}, {4,63,0,0,0,1,63,8,0,0}, {5,63,0,0,0,1,63,8,0,0}, {6,63,0,0,0,1,63,8,0,0}, {7,63,0,0,0,1,63,8,0,0}, {8,63,0,0,0,1,63,8,0,0}, {9,63,0,0,0,1,63,8,0,0}, {11,63,0,0,0,1,63,8,0,0}, {12,63,0,0,0,1,63,8,0,0}, {13,63,0,0,0,1,63,8,0,0} };
      trackLength = 16;
      tempo = 1200;
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        for(byte variableCycler = 0; variableCycler < TRACK_VARIABLE_QTY; variableCycler++){
          trackContainer[trackCycler][variableCycler] = emptyTrackContainer[trackCycler][variableCycler];
        }
      }
    }
  }
  if (saveType == "save") {
    putEepromByte(diskAddress, SAVE_BYTE_ADDRESS + slotOffset, 1, VALID_SAVE_BYTE);
    putEepromByte(diskAddress, TRACK_LENGTH_ADDRESS + slotOffset, 1, trackLength);
    putEepromByte(diskAddress, TEMPO_ADDRESS + slotOffset, 2, tempo);
    putEepromTrackContainer(diskAddress);
  }
  if (saveType == "erase") {
    putEepromByte(diskAddress, SAVE_BYTE_ADDRESS + slotOffset, 1, 0); //override save byte with 0 to stop detection of a save at current saveSlot
  }
}


//pull bytes from an exernal eeprom chip over I2C, provide I2C address, address of byte(s) needed, qty of bytes to pull (either 1 or 2)
//does using int return solve the problem of being able to pull multiple bytes?
int pullEepromByte(byte diskAddress, int byteAddress, byte qtyBytes) {
  int rdata = 0;
  if (qtyBytes ==1 || qtyBytes == 2) { //only transmit if the request is for 1 or 2 bytes
    Wire.beginTransmission(diskAddress);
    Wire.write((int)(byteAddress >> 8));   // MSB
    Wire.write((int)(byteAddress & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(diskAddress,qtyBytes);
    if (Wire.available() == 1) {
      rdata = Wire.read();
    } else if (Wire.available() > 1) {
      byte high = Wire.read();
      byte low = Wire.read();
      rdata = word(high,low);
    }
  }
  return rdata;
}


//put 1 or 2 bytes to external eeprom
void putEepromByte(byte diskAddress, int byteAddress, byte qtyBytes, int data) { 
  Wire.beginTransmission(diskAddress);
  Wire.write((int)(byteAddress >> 8));   // MSB
  Wire.write((int)(byteAddress & 0xFF)); // LSB
  if (data < 256) { //one byte
    Wire.write(data);
  } else if (data > 255 && data < 32768) { //two bytes
    Wire.write((int)(data >> 8)); //write MSB
    Wire.write((int)(data & 0xFF)); //write LSB
  }
  Wire.endTransmission();
  delay(5);
}



void putEepromTrackContainer(byte diskAddress) {
  int byteAddress = (saveLoadSlot * 128) + TRACK_CONTAINER_ADDRESS;
  
  Wire.beginTransmission(diskAddress);
  Wire.write((int)(byteAddress >> 8));   // MSB
  Wire.write((int)(byteAddress & 0xFF)); // LSB
  
  byte trackSendLimit = 3; //limit the amount of tracks written to keep from overflowing the arduino's softwareWire 32 byte buffer limit (2 bytes used for eeprom byte save location)
  for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
    if (trackCycler == trackSendLimit) { //triggers sending off of buffer of 30 bytes if more bytes are going to need to be sent afterward
      Wire.endTransmission();
      delay(5);
      byteAddress += 30; //breaks up maximum potential trackContainer size (120) into 4 sends to prevent softwarewire buffer overflow
      Wire.beginTransmission(diskAddress);
      Wire.write((int)(byteAddress >> 8));   // addresses are ints (2 bytes), so first send the MSB
      Wire.write((int)(byteAddress & 0xFF)); // ...then send the LSB
      trackSendLimit += 3; //increase send limit to capture next set of 3 tracks using trackCycler
    }
    for(byte variableCycler = 0; variableCycler < TRACK_VARIABLE_QTY; variableCycler++){
      Wire.write(trackContainer[trackCycler][variableCycler]);
    }
  }
 Wire.endTransmission();
 delay(5);
  
}


#endif
