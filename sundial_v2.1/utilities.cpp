#ifndef CPP_UTILITIES
  #define CPP_UTILITIES


#include <Arduino.h>
#include "globals.h"
#include "utilities.h"


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

#endif
