#ifndef CPP_INPUTSOUTPUTS
  #define CPP_INPUTSOUTPUTS

  
#include <Arduino.h>
#include <MIDI.h>
#include <USB-MIDI.h>

#include "globals.h"
#include "inputsOutputs.h"
#include "midiControl.cpp"
#include "memoryControl.cpp"

//extern class MIDI;

//---------------//
//-- UTILITIES --//
//---------------//

//consolodates program timing to one function, rather than a timer for each program funciton
void timerHandler() {
  
  currentMillis = millis();
  currentMicros = micros();
  canRun1ms = 0;
  //commenting out 10ms timer, currently nothing uses it
  //canRun10ms = 0; 
  canRun20ms = 0;    

  if(currentMicros >= timer1ms) {  //timer resolves using microseconds, not milliseconds
    timer1ms = currentMicros + 1000;
    canRun1ms = 1;
  }
  //if(currentMillis >= timer10ms) {
  //  timer10ms = currentMillis + 10;
  //  canRun10ms = 1;
  //}
  if(currentMillis >= timer20ms) {
    timer20ms = currentMillis + 20;
    canRun20ms = 1;
  }
}


byte cleanupByteValue(byte value, byte lowestVal, byte highestVal, bool rollover) {
  if (value > highestVal + 15 || value < lowestVal) {
    if (rollover) {
      return highestVal;
    } else {
      return lowestVal;
    }
  } else if(value > highestVal) {
    if (rollover) {
      return lowestVal;
    } else {
      return highestVal;
    }
  } else {
    return value;
  }
}


//---------------------------//
//--     CONTROL INPUTS    --//
//---------------------------//


//Update the state of the encoder (if it was spun)
//needs to happen frequently or it misses inputs (can't be in every 20ms loops)
void updateEncoder() {
  
  //Update Encoder State  
  byte encoderPin1State = digitalRead(ENCODER_PIN1);
  
  if(encoderPin1State != lastEncoderPin1State) {
    byte encoderPin2State = digitalRead(ENCODER_PIN2);
    if(encoderPin1State != encoderPin2State) {
      encoderState = 1;
    } else {
      encoderState = -1;
    }
    lastEncoderPin1State = encoderPin1State;
  }
  
}


//run this every 20ms for built-in debounce instead of per-button debounce checks
void updateButtons () {

  //Update Button States
  for (byte i = 0; i < 8; i++) {     //for each button....
    for (byte j = 0; j < 3; j++) {   //set the multiplexer for the correct pot
      digitalWrite(muxPins1[j], muxCommands1[i][j]);
    }
    //delayMicroseconds(5); //delay 5 microseconds (0.005 millis) to let mux stabilize (I can't believe I'm using delay =X )
    buttonState[i] = !digitalRead(BUTTON_PIN);
  }
  encoderButtonState = !digitalRead(ENCODER_BUTTON_PIN);
  
  
  byte pageCounter = 0;
  //buttons 1-7 and encoder button (button 8 is play/stop, not used for page changing)
  const byte buttonPageValues[8] = {1,2,4,8,16,32,64,128};
  
  //add button values to determine page number
  for(byte buttonCycler = 0; buttonCycler < 7; buttonCycler++) {
    if (buttonState[buttonCycler]) {    // && currentMillis > lastButtonMillis[buttonCycler] + BUTTON_DELAY) {
      pageCounter += buttonPageValues[buttonCycler];
    }
  }
  if (encoderButtonState) { // && currentMillis > lastButtonMillis[8] + BUTTON_DELAY) {
    lastButtonMillis[8] = currentMillis; //encoder value
    millisForPages = currentMillis;
    pageCounter += buttonPageValues[7]; //encoder value
  }
  
  //check to make sure a valid page is landed on
  for (byte pageChecker = 0; pageChecker < PAGE_QTY; pageChecker++) {
    if (pageCounter == validPages[pageChecker]) {
      currentPage = pageCounter;
      millisForPages = currentMillis;
    }
  }

/*
  //milli equalizer - evens out button delays to prevent milli based offset problems (unwanted page switching)
  int equalizer = pageCounter;
  for (int buttonCycler = 6; buttonCycler >= 0; buttonCycler--) {
    if(equalizer - buttonPageValues[buttonCycler] >= 0) {
      lastButtonMillis[buttonCycler] = currentMillis;
      equalizer -= buttonPageValues[buttonCycler];
    }
  }
*/

}



//---------------------------//
//--  PAGE DISPLAY/INTERP  --//
//---------------------------//


//used in ledControl() to display the ones digit and tens digit of a single number in 2 different colors
void countByTensLED(byte tensColor[3], byte selectionColor[3], byte LEDvariable) {

  LEDvariable += 1;
  
  int tensDigit = 0;
  if(LEDvariable > 9) {
    tensDigit = floor(LEDvariable / 10);
  }
  int selection = LEDvariable;
  if(tensDigit > 0) {
    selection = LEDvariable % 10;
  }
  
  for(byte countTens = 0; countTens < tensDigit; countTens++) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(tensColor[colorCheck]) {
        bitSet(ledContainer[colorCheck], 15 - countTens); 
      }
    }
  }
  for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
    if(selectionColor[colorCheck]) {
      bitSet(ledContainer[colorCheck], 16 - selection);
    }
  }
}

//used in ledControl() to display pages of 16 selections for each page, up to 16 pages total. Should be limited to 0-256 (one more than a byte) for display purposes
void countBySixteensLED(byte pageCountColor[3], byte selectionColor[3], int LEDvariable, bool overridePageColor) {
  
  byte selection = 0;
  if (LEDvariable > 0) {
    selection = LEDvariable % 16;
  }
  
  byte page = 1; //pages start at 1 for display purposes, page 1-1 to page 16-16 for all 256 slots instead of 0-1 to 15-16 which looks strange
  if (LEDvariable > 16) {
    page = (LEDvariable / 16) + 1;
  }
  
  for(byte countPage = 0; countPage < page; countPage++) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(pageCountColor[colorCheck]) {
        bitSet(ledContainer[colorCheck], 15 - countPage); 
      }
    }
  }
  for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
    if(selectionColor[colorCheck]) {
      bitSet(ledContainer[colorCheck], 15 - selection);
    } else if (overridePageColor) { //if overridePageColor is TRUE then completely override the color on selection's location
      bitClear(ledContainer[colorCheck], 15 - selection);
    }
  }
  
}


//used in ledControl() to display the ones, tens, hundreds digits of a single number in 3 different colors
void countByHundredsLED(byte hundredsColor[3], byte tensColor[3], byte selectionColor[3], int LEDvariable) {

  LEDvariable += 1;  

  //capture the last digit
  int selection = LEDvariable;
  if(LEDvariable > 0) {
    selection = LEDvariable % 10;
  } 
  //shrink number to 1/10th size to...
  //LEDvariable = floor(LEDvariable * 0.1);
  //capture 10's number
  int tensDigit = 0;
  if(LEDvariable > 9) {
    tensDigit = (LEDvariable / 10U) % 10;
  } 
  //shrink number to 1/10th size to...
  //LEDvariable = floor(LEDvariable * 0.1);
  //capture 100's number
  int hundredsDigit = 0;
  if(LEDvariable > 99) {
    hundredsDigit = (LEDvariable / 100U) % 10;
  }

/*  Uses no division, might work as a replacement process if needed for speed increase
  //capture the last digit
  int selection = LEDvariable;
  if(LEDvariable > 0) {
    selection = LEDvariable % 10;
  } 
  //shrink number to 1/10th size to...
  LEDvariable = floor(LEDvariable * 0.1);
  //capture 10's number
  int tensDigit = 0;
  if(LEDvariable > 0) {
    tensDigit = LEDvariable % 10;
  } 
  //shrink number to 1/10th size to...
  LEDvariable = floor(LEDvariable * 0.1);
  //capture 100's number
  int hundredsDigit = 0;
  if(LEDvariable > 0) {
    hundredsDigit = LEDvariable % 10;
  }
*/
  //Set ledContainer to the three colors provided, based off the LEDvariable provided
  for(byte countHundreds = 0; countHundreds < hundredsDigit; countHundreds++) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(hundredsColor[colorCheck]) {
        bitSet(ledContainer[colorCheck], 15 - countHundreds); 
      }
    }
  }
  for(byte countTens = 0; countTens < tensDigit; countTens++) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(tensColor[colorCheck]) {
        bitSet(ledContainer[colorCheck], 15 - countTens); 
      }
    }
  }
  for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
    if(selectionColor[colorCheck]) {
      bitSet(ledContainer[colorCheck], 16 - selection);
    }
  }
  
}





//handles button input and displaying LEDs
void pageControl() {
  
  

  if (encoderState != 0) {
    encoderWasChanged = true;
    saveLoadConfirmation = false;
    millisForPages = currentMillis;
    
    
    //need to update fastspin to require more clicks before fastspin turns on
    //FastSpin Encoder Detection
    if(encoderState == lastEncoderMovementValue) { //if encoder kept moving in the same direction...
      for(byte moveDigits = 1; moveDigits < 5; moveDigits++) { //bump all the values down the container 1 slot
        encoderMScontainer[moveDigits - 1] = encoderMScontainer[moveDigits];
      }
    } else { //if encoder changed movement direction...
      for(byte resetDigits = 0; resetDigits < 4; resetDigits++) //clear the container with high numbers to prevent fastSpin triggering
      encoderMScontainer[resetDigits] = 999;
    }
    if (currentMillis - lastButtonMillis[ENCODER_SPIN] < 1000) {
      encoderMScontainer[4] = currentMillis - lastButtonMillis[ENCODER_SPIN]; //put in the MS between last encoder spin and this one in last slot
    } else {
      encoderMScontainer[4] = 999;
    }
    int encoderMSaddition = 0;
    for(byte countMSdigits = 0; countMSdigits < 5; countMSdigits++) {
      encoderMSaddition += encoderMScontainer[countMSdigits];
    }
    if(encoderMSaddition < ENCODER_DELAY * 5) { //if the encoder has spun 5 click in the same direction faster than 3 standard encoder delays, turn on fastSpin
      fastSpin = true;
    } else {
      fastSpin = false;
    }
    lastEncoderMovementValue = encoderState; //update last encoder change value
  }


  //BOOK KEEPING
  
  //remove confirmation status for saving and loading if the page has changed
  if(previousPage != currentPage) {
    saveLoadConfirmation = false;
  }
  
  
  byte ledOff = 0b00000000;
  
  int brightnessFactor = 6;//less brightness for non-prototype running 595 chips on the 5v rail instead of prototype's 3v 595 design
  //if (prototype) {brightnessFactor = 16;} //previously used to raise brightness for 3v multiplexer setup (5v in use now is birghter)
  
  //empty the LED display container, the rest of the ledControl() function refills it;
  for(int ledTrack = 0; ledTrack < 3; ledTrack++) {
    for(byte led = 0; led < 16; led++) {
      bitClear(ledContainer[ledTrack],led);
    }
  }
 //if ledBrightness setting changed, then output the new brightness over PWM
 if(changeBrightness) {
    
    byte newBrightness = 254 - ((ledBrightness -1) * brightnessFactor);
    analogWrite(BRIGHTNESS_PIN, newBrightness);
    changeBrightness = false;
  } 
  //flashes brightness brighter on new beats, intensity controlled by beatFlash
  if(beatFlash > 0 && allowBeatFlash) {
    if(lastBeatMillis - currentMillis < 200) {
      byte newBrightness = 0;  //initially, set brightness to max
      if(ledBrightness > 9) {  //if brightness is over 9 (of 16) then blank out lights instead of flashing brighter
        newBrightness = 255;
      }
      if(ledBrightness + beatFlash < 17) { //if brightness plus beatflash is higher than 16 (max), leave it at max instead of trying to go higher
        newBrightness = 254 - ((ledBrightness + beatFlash - 1) * brightnessFactor); //if brightness + beatflash was lower than 16, then set exact brightness level
      }
      analogWrite(BRIGHTNESS_PIN, newBrightness);
    } else {   //if it's been longer than 200ms since last beat, set brightness to normal setting
      byte newBrightness = 254 - ((ledBrightness - 1) * brightnessFactor); 
      analogWrite(BRIGHTNESS_PIN, newBrightness);
    }
  }
  allowBeatFlash = true; //bookkeeping to reset beatFlash to be turned on, forces diasabling processes to actively (continually) disable it

  
  
  //PAGE INPUTS/DISPLAYS
  
  if (currentPage == SEQUENCER_PAGE) {
    
    //input controls
     
    //if the encoder was spun and the encoder's button delay has passed....
    if(!buttonState[SEQUENCER_BUTTON]){ //if the track button (0) is not being held down...
      if(encoderWasChanged && !encoderButtonState && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) { //if encoder isn't being held down
        trackContainer[currentTrack][NOTE_QTY] += encoderState; //change the NOTE_QTY for the currentTrack
        //cleanup for NOTE_QTY
        trackContainer[currentTrack][NOTE_QTY] = cleanupByteValue(trackContainer[currentTrack][NOTE_QTY],0,trackLength,true);
        lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
      } else if (encoderWasChanged && encoderButtonState && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
        trackContainer[currentTrack][START_NOTE] += encoderState; //... then add or subtrack encoderState value from the currentTrack's START_NOTE
        //cleanup for START_NOTE values
        trackContainer[currentTrack][START_NOTE] = cleanupByteValue(trackContainer[currentTrack][START_NOTE],0,trackLength - 1,true); 
        lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
      }
    } else {
      //encoderButtonState == TRUE for this is page 10
      if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
        currentTrack += encoderState; //... then change the currentTrack by the encoderState value
        //cleanup for currentTrack values
        currentTrack = cleanupByteValue(currentTrack, 0, TRACK_QTY - 1 , true);
        lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
      } 
    }
    
    //if brightness or beatflash has changed, save it to EEPROM now 
    //Putting on page 0 prevents saving every single time the value is changed, 
    //and instead focuses on saving when changes are assumed complete
    //to save EEPROM life
    if (previousLEDBrightness != ledBrightness) {
      int eepromAddressHelper = BRIGHTNESS_ADDRESS;
      EEPROM.put(eepromAddressHelper, ledBrightness);
      previousLEDBrightness = ledBrightness;
    }
    if (previousBeatFlash != beatFlash) {
      int eepromAddressHelper = BEATFLASH_ADDRESS;
      EEPROM.put(eepromAddressHelper, beatFlash);
      previousBeatFlash = beatFlash;
    }
    
    //led display
    
    const bool trackColors[TRACK_QTY][3] = { //RGB colors for each track, repeats due to limited colors
      {1,0,1}, //purple
      {0,0,1}, //blue
      {1,1,0}, //yellow
      {0,1,1}, //aqua
      {0,1,0}, //grn
      {1,0,0}, //red
      {1,0,1}, //purple
      {0,0,1}, //blue
      {1,1,0}, //yellow
      {0,1,1}, //aqua
      {0,1,0}, //grn
      {1,0,0}  //red
    };
    
    //Turns on LEDs for notes on the current track
    for(byte beatCheck = 0; beatCheck < trackLength; beatCheck++){ //checks currentTrack for illuminating LEDs
      if(noteTriggers[currentTrack][beatCheck]){ // if this beat contains a noteTrigger TRUE...
        for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //cycle thru Red, Green, Blue and check the proper LED color of the track
          if(trackColors[currentTrack][colorCheck]) { //if this trackColor (RGB) is TRUE...
            bitSet(ledContainer[colorCheck], 15 - beatCheck); // set the LED to ON
          }
        }
      }
    }
    
    //make current beat's location LED white
    if(currentBeat > 0) {
      for(byte beatColors = 0; beatColors < 3; beatColors++) {
        bitSet(ledContainer[beatColors], 16 - currentBeat);
      }
    }
    
    //overrides and turns on all LEDs with the trackColors if track changed
    //effectively flashes them since this loop runs with canRun20ms
    if(ledPreviousTrack != currentTrack) {
      if (currentTrack < 6) {
        flashTrackLED = 10;
      } else {
        flashTrackLED = 20;
      }
    }
    if (flashTrackLED > 0) {
      allowBeatFlash = false; //disable beatFlash while flashTrackLED'ing
      if(flashTrackLED < 11 || flashTrackLED > 13) {
        for(byte beatCounter = 0; beatCounter < 16; beatCounter++) {
          for(byte colorCheck = 0; colorCheck < 3; colorCheck++) {
            if(trackColors[currentTrack][colorCheck]) { //if this trackColor (RGB) is TRUE...
              bitSet(ledContainer[colorCheck], 15 - beatCounter); // set the LED to ON
            } else {
              bitClear(ledContainer[colorCheck], 15 - beatCounter); // or set the LED to OFF if not
            }
          }
        }
      } else {
        //don't fill LED container with anything (simulates a blink if flashTrackLED is higher than 10 but lower than 14)
      }
      flashTrackLED -= 1;
    }
    ledPreviousTrack = currentTrack;
    
  } else if ( currentPage == PITCH_PAGE) {
  
    //input controls
  
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      if(lastNoteMillis[currentTrack] != 0){ //if the track has started playing a note already, stop it
        MIDI.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        MIDIUSB.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        lastNoteMillis[currentTrack] = 0;
      }
      if (fastSpin) {
        trackContainer[currentTrack][PITCH] += encoderState * 5;
      } else {
        trackContainer[currentTrack][PITCH] += encoderState;
      }
      //cleanup for PITCH values
      trackContainer[currentTrack][PITCH] = cleanupByteValue(trackContainer[currentTrack][PITCH],0,127,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte tensColor[3] = {0,0,1};
    byte selectionColor[3] = {0,1,0};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][PITCH]);
  
  } else if ( currentPage == INSTRUMENT_PAGE) {

    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      if (fastSpin) { 
        trackContainer[currentTrack][INSTRUMENT] += (encoderState * 5);
      } else {
        trackContainer[currentTrack][INSTRUMENT] += encoderState;
      }
      //cleanup INSTRUMENT value
      trackContainer[currentTrack][INSTRUMENT] = cleanupByteValue(trackContainer[currentTrack][INSTRUMENT],0,127,true);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte tensColor[3] = {0,1,0};
    byte selectionColor[3] = {1,0,1};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][INSTRUMENT]);  
    
  } else if ( currentPage == TEMPO_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(tempoInternal) { // only change tempo is using tempoInternal
      if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
        
        if(fastSpin) {
        tempo += (encoderState * 100);
        } else {
          tempo += (encoderState * 10);
        }
        //manually clean up tempo variable (cleanupByteValue is for bytes, tempo is int)
        if (tempo > MAXIMUM_TEMPO) {
          tempo = MAXIMUM_TEMPO;
        } else if(tempo < MINIMUM_TEMPO) {
          tempo = MINIMUM_TEMPO;
        }
        lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
      }
    }
    
    //led display
    
    byte hundredsColor[3] = {1,0,0};
    byte tensColor[3] = {0,0,1};
    byte selectionColor[3] = {0,1,0};
    
    countByHundredsLED(hundredsColor, tensColor, selectionColor, (tempo * 0.1) - 1); //tempo 8 0.1 to turn it into BPM, -1 to do LED function math propery (zero based counting system)
    
  } else if ( currentPage == SAVE_PAGE || currentPage == LOAD_PAGE || currentPage == ERASE_PAGE ) {
    
    //input controls
    
    allowBeatFlash = false;
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      if(fastSpin) {
        saveLoadSlot += encoderState * 16;
      } else {
        saveLoadSlot += encoderState;
      }
      saveLoadSlot = cleanupByteValue(saveLoadSlot,0,255,true);
      saveLoadConfirmation = false;
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    if(encoderButtonState && currentMillis > lastButtonMillis[ENCODER_BUTTON] + BUTTON_DELAY) {
      if(!saveLoadConfirmation) {
        saveLoadConfirmation = true;
      } else {
        if(currentPage == SAVE_PAGE) {
          saveTrack(DISK1_ADDR, "save");
          currentPage = 0;
          flashTrackLED = 10;
        } else if (currentPage == LOAD_PAGE) {
          saveTrack(DISK1_ADDR, "load");
          currentPage = 0;
          currentTrack = 0;
          flashTrackLED = 10;
        } else if (currentPage == ERASE_PAGE) {
          saveTrack(DISK1_ADDR, "erase");
        }
        saveLoadConfirmation = false;
      }
      lastButtonMillis[ENCODER_BUTTON] = currentMillis;
    }
    
    //led display
    
    //RGB for page 4, 5, and 6 display
    byte pageCountColor[3] = {0,0,1};       //pages will count upward as blue
    byte saveLoadColors[3] = {1,1,1};  //empty save slots will display as white
    int eepromAddressHelper = saveLoadSlot * 128; //preparing to check is a save slot is occupiued
    byte saveByteCheck = pullEepromByte(DISK1_ADDR, eepromAddressHelper, 1);
    if (saveByteCheck == VALID_SAVE_BYTE) { //if saveslot is occupied, change display color (saveLoadColors) to currentPage's (save, load, erase) color
      if (currentPage == SAVE_PAGE || currentPage == ERASE_PAGE) {
        saveLoadColors[0] = 1;
        saveLoadColors[1] = 0;
        saveLoadColors[2] = 0;
      } else if (currentPage == LOAD_PAGE) {
        saveLoadColors[0] = 0;
        saveLoadColors[1] = 1;
        saveLoadColors[2] = 0;
      }
    }
    //display final color selections, override the page color with the selection's color. Most of the program's existing code
    //harmonizes counting display colors and allows overlap for clarity, but with the amount of options with save, load, erase, 
    //plus the pageCount color, override works better for clarity in the end. Will require the user to pay slightly more 
    //attention to the page they're on.
    countBySixteensLED(pageCountColor, saveLoadColors, saveLoadSlot, true);
    
  } else if ( currentPage == VELOCITY_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      if(fastSpin) { 
        trackContainer[currentTrack][VELOCITY] += encoderState * 5;
      } else {
        trackContainer[currentTrack][VELOCITY] += encoderState;
      }
      
      //cleanup for VELOCITY values
      trackContainer[currentTrack][VELOCITY] = cleanupByteValue(trackContainer[currentTrack][VELOCITY],0,127,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }  
    
    //led controls
    
    byte tensColor[3] = {1,0,1};
    byte selectionColor[3] = {0,1,0};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][VELOCITY]);
    
  } else if ( currentPage == CHANNEL_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      if(lastNoteMillis[currentTrack] != 0){ //if the track has started playing a note already, stop it
        MIDI.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        lastNoteMillis[currentTrack] = 0;
      }
      trackContainer[currentTrack][CHANNEL] += encoderState; //... then change the Channel of the current track with encoderState
      //cleanup for CHANNEL values 
      trackContainer[currentTrack][CHANNEL] = cleanupByteValue(trackContainer[currentTrack][CHANNEL],1,16,true);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte trackChannelColors[3] = {0,0,1};
    
    for(byte channelCheck = 0; channelCheck < trackContainer[currentTrack][CHANNEL]; channelCheck++) { //fill the ledContianer based off the channel number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(trackChannelColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - channelCheck);
        }
      }
    }
    
    
  } else if ( currentPage == TRACK_LENGTH_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      trackLength += encoderState; //add or subtrack the encoderState value from the amount of beats in the trackLength
      //cleanup for trackLength values
      trackLength = cleanupByteValue(trackLength,1,16,false);
      //recalculate all track START_NOTEs and NOTE_QTYs if length changes
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        trackContainer[trackCycler][NOTE_QTY] = cleanupByteValue(trackContainer[trackCycler][NOTE_QTY],0,trackLength,false);
        trackContainer[trackCycler][START_NOTE] = cleanupByteValue(trackContainer[trackCycler][START_NOTE],0,trackLength - 1,false); 
      }
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte trackLengthColors[3] = {1,0,0};
    
    for(byte lengthCheck = 0; lengthCheck < trackLength; lengthCheck++) { //fill the ledContianer based off the trackLength...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(trackLengthColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - lengthCheck);
        }
      }
    }
    
  } else if ( currentPage == NOTESPREAD_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      trackContainer[currentTrack][NOTE_SPREAD] += encoderState; //... then change the Note Spread of the current track with encoderState
      //cleanup for Note Spread values 
      trackContainer[currentTrack][NOTE_SPREAD] = cleanupByteValue(trackContainer[currentTrack][NOTE_SPREAD],1,8,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte noteSpreadColors[3] = {0,1,0};
    
    for(byte noteSpreadCheck = 0; noteSpreadCheck < trackContainer[currentTrack][NOTE_SPREAD]; noteSpreadCheck++) { //fill the ledContianer based off the channel number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(noteSpreadColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - noteSpreadCheck);
        }
      }
    }
    
  } else if ( currentPage == BRIGHTNESS_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      ledBrightness += encoderState; //... then change the display's brightness with ledBrightness
      //cleanup for ledBrightness values 
      ledBrightness = cleanupByteValue(ledBrightness,1,16,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
      changeBrightness = true; //trigger for ledControl() to update the brightness;
    }
    
    //led display
    
    byte brightnessColors[3] = {1,1,1};
    
    for(byte brightnessCheck = 0; brightnessCheck < ledBrightness; brightnessCheck++) { //fill the ledContianer based off the ledBrightness number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(brightnessColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - brightnessCheck);
        }
      }
    }
    
    
  } else if ( currentPage == NOTE_LENGTH_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      trackContainer[currentTrack][NOTE_LENGTH] += encoderState;
      trackContainer[currentTrack][NOTE_LENGTH] = cleanupByteValue(trackContainer[currentTrack][NOTE_LENGTH],1,16,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte noteLengthColors[3] = {1,1,0};
    for(byte lengthCheck = 0; lengthCheck < trackContainer[currentTrack][NOTE_LENGTH]; lengthCheck++) { //fill the ledContianer based off the trackLength...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(noteLengthColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - lengthCheck);
        }
      }
    }  
    
  } else if ( currentPage == BEATFLASH_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      beatFlash += encoderState; //... then change the intensity of beatFlash with encoderState
      //cleanup for beatFlash values 
      beatFlash = cleanupByteValue(beatFlash,0,16,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
    //led display
    
    byte beatFlashColors[3] = {0,1,1};
    
    for(byte beatFlashCheck = 0; beatFlashCheck < beatFlash; beatFlashCheck++) { //fill the ledContianer based off the ledBrightness number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(beatFlashColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - beatFlashCheck);
        }
      }
    }
    

  }  //END OF PAGES 
  
  
  //PLAY/PAUSE BUTTON INPUT
  if(buttonState[PLAY_BUTTON] && currentMillis > lastButtonMillis[PLAY_BUTTON] + BUTTON_DELAY) {
    playing = !playing;
    lastBeatMillis = currentMillis;
    lastButtonMillis[PLAY_BUTTON] = currentMillis;
  }
  
  //STOP BUTTON INPUT
  if(buttonState[STOP_BUTTON] && currentMillis > lastButtonMillis[STOP_BUTTON] + BUTTON_DELAY) {
    playing = false;
    currentBeat = 0;
    notesOff(); 
    lastButtonMillis[STOP_BUTTON] = currentMillis;
  }
  
  
  //check if the currentPage needs reset to page 0 due to inactivity
  if(currentPage != SEQUENCER_PAGE){
    if(currentMillis > millisForPages + PAGE_0_TIMEOUT) {
      currentPage = 1;
    }
  }
  
  
  encoderWasChanged = false;
  encoderState = 0; //reset back to neutral state to prevent misdetection

  previousPage = currentPage; // update page detection
  
  
  //Push containers out to LEDs
  for(int j = 2; j >= 0; j--) {
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, lowByte(ledContainer[j])); 
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, highByte(ledContainer[j]));
  }
  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(LATCH_PIN, LOW);
  
}


#endif
