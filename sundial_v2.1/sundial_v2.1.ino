
/******************************************************************************

Orbital Midi Sequencer

ToDo:

    change millisForPages[] to a single variable instead of array for each page
    
    audit timer system to check if it's streamlined
    
    audit variables to see if they can be changed to #defines instead
    
    check speed of program, can midi read be moved outside of timer limiters?
    
    cleanup notesOff stuff to include actual noteoff midi commands
 
  Math Cleanup:
  
    fix tempo display, currently shows +1 value
  
  midi still implementing:
    usb midi and crosstalk with DIN
    re-implement start/stop/continue command handling?
    receive song position over midi (any other?)
    
  Need to make an EEPROM filler program or method for programming new devices with a standard baseline
  

*******************************************************************************/


#include <Arduino.h>

#include "globals.h"
#include "memoryControl.cpp"
#include "inputsOutputs.cpp"

//-- SETUP --
void setup() {

  //Begin I2C 
  Wire.begin();

  //enable input pins and turn pullup resistors on
  pinMode(ENCODER_PIN1, INPUT);
  pinMode(ENCODER_PIN2, INPUT);
  pinMode(ENCODER_BUTTON_PIN, INPUT);
  digitalWrite(ENCODER_PIN1, HIGH);
  digitalWrite(ENCODER_PIN2, HIGH);
  digitalWrite(ENCODER_BUTTON_PIN, HIGH);
  for (byte i = 0; i < 3; i++) {
    pinMode(muxPins1[i], OUTPUT);
    digitalWrite(muxPins1[i], HIGH);
  }
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);
  
  //Enable LED shift register pins
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT); 
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  analogWrite(BRIGHTNESS_PIN, 100);
  
  //MIDI Implementation
  //MIDI.setHandleClock(handleClock);
  //MIDI.setHandleStart(handleStart);
  //MIDI.setHandleStop(handleStop);
  //MIDI.setHandleContinue(handleContinue);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDIUSB.begin(MIDI_CHANNEL_OMNI);

  //to suppress midi signals to the VS1053B
  pinMode(MIDI_SUPPRESS_PIN, OUTPUT);
  digitalWrite(MIDI_SUPPRESS_PIN, HIGH);

  //clear LEDs and prepare them for next write
  //writes full clear bits out for each of the 6 595 chips
  digitalWrite(LATCH_PIN, LOW);
  for(int i = 0; i < 6; i++) {
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b00000000);
  }
  digitalWrite(LATCH_PIN, HIGH); //cycle the shift registers to display
  digitalWrite(LATCH_PIN, LOW); //cycle the shift registers to be ready to receive bytes for next display
  
  //Load Internal EEPROM settings...
  //check internal EEPROM brightness and beatFlash settings and load them if valid
  byte saveByteCheck = 0;  
  int eepromAddressHelper = BRIGHTNESS_ADDRESS;
  EEPROM.get(eepromAddressHelper, saveByteCheck);
  if(saveByteCheck > 0 && saveByteCheck < 17) {
    ledBrightness = saveByteCheck;
    previousLEDBrightness = saveByteCheck;
  }
  saveByteCheck = 0;
  eepromAddressHelper = BEATFLASH_ADDRESS;
  EEPROM.get(eepromAddressHelper, saveByteCheck);
  if(saveByteCheck < 17) { //can have a zero value
    beatFlash = saveByteCheck;
    previousBeatFlash = saveByteCheck;
  }
  //Plans to add option-based auto-loading track from saveslot zero. Lets user change default track settings without programming - Also need to consider using track zero when loading from empty slot 
  //saveTrack("load"); //load first saveSlot (0) into sequencer
  
}
//-- END OF SETUP --


//-- LOOP --
void loop() {
  
  timerHandler();
  
  readMIDI();
  
  updateEncoder();
  
//  inputDetection();

//  if(canRun1ms) { 

//    updateButtons();
    
//  }
  
  
 if(canRun20ms) {

    updateButtons();
  
    updateTempoValues();
    
    updateNoteTriggers();
    
    beatCheck();
    
    handleNotes();
    
    pageControl();
    
  }

}
//-- END OF LOOP --
