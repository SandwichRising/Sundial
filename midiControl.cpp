#ifndef CPP_MIDICONTROL
  #define CPP_MIDICONTROL


#include <Arduino.h>

//MIDI Libraries
#include <MIDI.h>
#include <USB-MIDI.h> //companion to MIDI.h for usb midi usage

#include "globals.h"
#include "midiControl.h"
#include "inputsOutputs.cpp"


// -- MIDI USB + Serial Initialize -- //

USING_NAMESPACE_MIDI;
typedef USBMIDI_NAMESPACE::usbMidiTransport __umt;
typedef MIDI_NAMESPACE::MidiInterface<__umt> __ss;
__umt usbMIDI(0); // cableNr
__ss MIDIUSB((__umt&)usbMIDI); //names the object MIDIUSB
typedef Message<MIDI_NAMESPACE::DefaultSettings::SysExMaxSize> MidiMessage;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI)


  
//-------------------//
//--  MIDI CONTROL --//
//-------------------//

//check midi inputs for messages and forward them between serial midi/usb port
void readMIDI() {
  
  if(MIDI.read()) {
    MIDIUSB.send(MIDI.getType(),MIDI.getChannel(),MIDI.getData1(),MIDI.getData2());
  }
  if(MIDIUSB.read()) {
    MIDI.send(MIDIUSB.getType(),MIDIUSB.getData1(),MIDIUSB.getData2(),MIDIUSB.getChannel());
  }
}


//plays and stops playback of MIDI notes
void handleNotes() {
  //check all tracks for notes that need turned off with lastNoteMillis
  for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){ //cycle through each track
    if(lastNoteMillis[trackCycler] != 0) {  //if a note was played for a track, check the timing to see if it's time send the note off command
      if(currentMillis >= lastNoteMillis[trackCycler] + (trackContainer[trackCycler][NOTE_LENGTH] * msBetweenBeats) || !playing) {  //check NOTE_LENGTH math for current track and also check if playback should be full-stopped
        MIDI.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]); //send out noteOff commands
        MIDIUSB.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
        lastNoteMillis[trackCycler] = 0; //empty out this track's lastNoteMillis[] slot
      }
    }
  }

  if(beatJustChanged){ //if beatCheck() just advanced to the next beat
    for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){ //cycle through each track
      if(noteTriggers[trackCycler][currentBeat - 1]) { //if a track is set to play a note this beat
        MIDI.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]); //
        MIDI.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
        
        MIDIUSB.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]);
        MIDIUSB.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
        lastNoteMillis[trackCycler] = currentMillis; //fill this track's lastNoteMillis[] slot with this note's timing
      }
    }
    beatJustChanged = false; //turns off beatJustChanged toggle after all notes are calculated for the current beat
  }
}

//sends noteOff for each track and empties out lastNoteMillis[]
void notesOff() {
  for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){
    MIDI.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
    MIDIUSB.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
    lastNoteMillis[trackCycler] = 0;
  }
}


//-------------------//
//-- MUSIC CONTROL --//
//-------------------//



//check program timing and compare with tempo variables to see if it's time to advance to the next beat in the track
void beatCheck() {
  if(playing && currentBeat == 0) {
    currentBeat += 1;
    beatJustChanged = true; //this triggers notes to play with handleNotes()
    lastBeatMillis = currentMillis;
  } else if(playing && currentMillis >= lastBeatMillis + msBetweenBeats) {
    currentBeat += 1;
    currentBeat = cleanupByteValue(currentBeat,1,trackLength,true);
    beatJustChanged = true; //this triggers notes to play with handleNotes()
    lastBeatMillis = currentMillis;
  }
}

//check to see if the tempo has changed since last loop, and if so adjust tempo variables accordingly
void updateTempoValues() {
  if(lastTempo != tempo) {
    msBetweenBeats = (600000 / tempo); //600000 instead of 60000 because tempo is BPM * 10
    lastTempo = tempo;
  }
}


//loads the trackContainer array with note triggers based on trackContainer array settings
//the math used here controls how notes spread out when adding more and more
void updateNoteTriggers() {
  //empty all notes from the noteTriggers before recalculating them
  for(byte trackCycle = 0; trackCycle < TRACK_QTY; trackCycle++){ 
    for(byte noteCycle = 0; noteCycle < 16; noteCycle++) {
      noteTriggers[trackCycle][noteCycle] = 0;
    }
  }
  //refill the noteTriggers array with notes
  for (byte trackNumber = 0; trackNumber < TRACK_QTY; trackNumber++){ //cycle through each track
    if(trackContainer[trackNumber][NOTE_QTY] > 0) { //only fill tracks that have 1 or more notes added (and prevent dividing by 0)
    
      int nextNoteLocation = 0 + trackContainer[trackNumber][START_NOTE]; //get the first note's location
      
      noteTriggers[trackNumber][nextNoteLocation] = 1; //toggle the first note to 'on'
      byte remainingNotes = trackLength; //set the available amount of room to spread notes out to match the song's trackLength
      for(byte noteCounter = 1; noteCounter < trackContainer[trackNumber][NOTE_QTY]; noteCounter++) { //cycle once for each note remaining in the track
        byte nextGap = ceil(remainingNotes / (trackContainer[trackNumber][NOTE_QTY] - noteCounter + 1)); //calculate the distance (in beats) to place the next note by dividing the available room to work with (remainingNotes) by how many notes are left to calculate and rounding up
        if(nextGap > trackContainer[trackNumber][NOTE_SPREAD]) { //if this note's distance would be farther away than the track's NOTE_SPREAD variable...
          nextGap = trackContainer[trackNumber][NOTE_SPREAD];    //then put this note at NOTE_SPREAD's distance instead of the calculated nextGap location
        }
        remainingNotes -= nextGap; //subtract the distance used to place this note from the available space remaining to place more notes
        nextNoteLocation += nextGap; //add the nextGap's distance to the last note's placement to get this note's placement location on the track
        if(nextNoteLocation > (trackLength - 1)) { //if the placement is outside of the track's availalbe beats...
          nextNoteLocation -= trackLength;         //then subract the trackLength to rollover the location to the start of the track
        }
        noteTriggers[trackNumber][nextNoteLocation] = 1; //toggle the calculated note to 'on' 
      }
    }
  }
}


#endif
