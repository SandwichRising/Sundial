
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
//EEPROM
#include <SoftwareWire.h> //for saving to external chip eeprom, software wire is used to save interrupt pins for rotary encoder
#include <EEPROM.h> //for saving to internal eeprom memory
//MIDI
#include <MIDI.h>
//#include <midi_UsbTransport.h>
#include "MIDIUSB.h"  //currently not used, wanting to use MIDI.h to handle all midi functions


//selecting which board is being programmed
#define V0_PROTOTYPE_BOARD
#define V1_PCB_BOARD
#define V2_PCB_BOARD

// -- PIN/CODE DEFINES -- //
#define ENCODER_PIN1 2 //MSB pin of encoder
#define ENCODER_PIN2 3 //LSB pin of encoder
#define ENCODER_BUTTON_PIN 4 //encoder button pin
#define BUTTON_PIN 5 //button multiplexer output pin
//Multiplexer Pins
#define BUTTON_MUX_PIN_1 6
#define BUTTON_MUX_PIN_2 7
#define BUTTON_MUX_PIN_3 8
//LED Shift Register Pins
#define BRIGHTNESS_PIN 9
#define CLOCK_PIN 14
#define LATCH_PIN 15
#define DATA_PIN 16
//I2C Bit-Banged Pins (EEPROM)
#define SCL_PIN 20
#define SDA_PIN 21

//for changing to the original prototype (handwired) version program format - rewiriting to board defines, slated for removal
bool prototype = false;


// -- HELPER DEFINES  -- (for code clarity) //
//These are to use throughout the code to provide information on the variables referenced rather than relying on obscure numbers for things like array indexes

//trackContainer[][] helpers
//using 24LC256, potential cap is 12 tracks with 10 (1 byte) variables each track, plus tempo, trackLength and saveState
//this allows 16 pages of 16 tracks for saving
#define TRACK_QTY 12  //how many tracks to cycle between
#define TRACK_VARIABLE_QTY 10 //how many parameters each track stores
#define CHANNEL 0
#define PITCH 1
#define START_NOTE 2
#define NOTE_QTY 3
#define INSTRUMENT 4
#define NOTE_LENGTH 5
#define VELOCITY 6
#define NOTE_SPREAD 7
//two more track variable bytes allowed and currently unused for each instument track (12 instrument tracks per save)
//#define unused_variable 8
//#define unused_variable 9

//EEPROM defines
//internal EEPROM save addresses
#define BRIGHTNESS_ADDRESS 0
#define BEATFLASH_ADDRESS 1
//1022 bytes still available for internal EEPROM saving

//external EEPROM I2C address
#define DISK1_ADDR 0x50  //24LC256 I2C address with all address pins wired LOW to GND
//external EEPROM save addresses
#define SAVE_BYTE_ADDRESS 0   //VALID_SAVE_BYTE saved here indicates a valid track save, any other value indicates empty track slot 
#define TRACK_LENGTH_ADDRESS 1 //one byte to indicate how many beats in the song
#define TEMPO_ADDRESS 2 //two bytes to indicate the song's tempo (BPM * 10)
#define TRACK_CONTAINER_ADDRESS 4 //120 bytes, stores all 12 tracks (trackContainer[])
//addresses 124,125,126,127 still available for saving/loading bytes for each song.
//These are at the END of the 128 byte (2 page) save slot to make half the trackContainer (64 bytes) fit evenly before the end of the page, 
//and half after the page end. Moving these slots will break that and page writes will wrap around to beginning of 24LC256 pages
//because of how putEepromTrackContainer writes pages instead of individual bytes.
//The tutorial at http://www.hobbytronics.co.uk/eeprom-page-write has info on this, kudos to their great write-up

//Buttons
#define SEQUENCER_BUTTON 0
#define PITCH_BUTTON 1
#define TEMPO_BUTTON 2
#define CHANNEL_BUTTON 3
#define SAVE_BUTTON 4
#define LOAD_BUTTON 5
#define PLAY_BUTTON 6
#define STOP_BUTTON 7
#define ENCODER_BUTTON 8 //only used for certain operations, like lastButtonMillis. Has its own dedicated pin.
#define ENCODER_SPIN 9   //only used for certain operations, like lastButtonMillis.

//Pages
#define SEQUENCER_PAGE 0
#define PITCH_PAGE 1
#define TEMPO_PAGE 2
#define INSTRUMENT_PAGE 13
#define SAVE_PAGE 4
#define LOAD_PAGE 5
#define ERASE_PAGE 6
#define NOTESPREAD_PAGE 7
#define BRIGHTNESS_PAGE 9
#define NOTE_LENGTH_PAGE 10
#define VELOCITY_PAGE 11
#define TRACK_LENGTH_PAGE 12
#define CHANNEL_PAGE 3
#define BEATFLASH_PAGE 19



// -- WIRE I2C Initialize -- //
SoftwareWire softWire(SDA_PIN, SCL_PIN); //SDA, SCL

// -- MIDI Initialize -- //
static const unsigned sUsbTransportBufferSize = 16;
//typedef midi::UsbTransport<sUsbTransportBufferSize> UsbTransport;
//UsbTransport sUsbTransport;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI)
//MIDI_CREATE_INSTANCE(UsbTransport, sUsbTransport, MIDIUSB)

//May bring back to handle connection between ATMEGA and VS1053B
//#include <SoftwareSerial.h>
//#define SOFTPIN_RX 18 //A0
//#define SOFTPIN_TX 10
//SoftwareSerial softSerial(SOFTPIN_RX,SOFTPIN_TX); //rx, tx
//MIDI_CREATE_INSTANCE(SoftwareSerial, softSerial, MIDI)





//-- GLOBAL VARIABLES --//
byte currentPage = 0; //controls what is displayed on the LED ring and what button inputs do
byte currentTrack = 0; //0-11 for 12 separate tracks
bool playing = true; //is the device playing or paused/stopped?
byte currentBeat = 0; //what beat the track is currently at (stopped = 0, music starts on 1, and rolls over from 16 (or trackLength) to 1)
byte saveLoadSlot = 0; //remembers selected save/load slot
bool saveLoadConfirmation = false; //bool to require second click of encoder to save, load, or erase a track

//Button and Detection Variables
bool buttonState[8] = {0,0,0,0,0,0,0,0}; //container to know if buttons are currently pushed or not
bool encoderButtonState = 0; //container to know if encoder button is being pushed
char encoderState = 0; //-1 is encoder moved down, 1 is encoder moved up, 0 is no change
unsigned long lastButtonMillis[10] = {0,0,0,0,0,0,0,0,0,0}; //timers to prevent unintentional button represses (8 buttons, encoder button, encoder knobs)
unsigned long millisForPages = 0; //timers to decide when to change back to page 0 automatically
#define PAGE_0_TIMEOUT 4000 //(pageZEROtimeout) ms without any input before display goes back to page 0 automatically
#define BUTTON_DELAY 180 //ms delay between detecting button presses
#define ENCODER_DELAY 100 //ms delay between detecting encoder knob turns

//Encoder Math Variables
byte lastEncoderPin1State = 0; //for updateEncoder() math, for checking to see if the encoder moved
bool encoderWasChanged = false; //reference if the encoder registered a changed in UP/DOWN state
int lastEncoderMovementValue = 0; //for fast-spinning detection, will only contain +1 or -1 after first encoder use
int encoderMScontainer[5] = {0,0,0,0,0}; //for fast-spinning detection/math, used by loading with timing info to see how fast the encoder is changing states
bool fastSpin = false;

//MIDI and Note Playing Variables
unsigned long lastNoteMillis[TRACK_QTY] = {0,0,0,0,0,0,0,0,0,0,0,0}; //container for each track to hold when last note was played
bool noteTriggers[TRACK_QTY][16] = { //to hold note triggers for each of the tracks
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

//external tempo control - disabled for now, was not timing correctly
/*
long unsigned lastClockMessageMicros = 0; //storage for the last clock signal received to do math with
long unsigned currentClockMessageMicros = 0; //storage for the current clock message getting received for math
bool midiStartCommand = false; //trigger for recevied midi start handler to listen for next clock message and play
*/

//Multi-Function Timer Variables
//rather than having each function checking milli math and using their own timer variables,
//this section is setup to handle timers multiple functions will use. timerHandler() 
//watches the clock and updates the canRun variables which other individual functions watch
//in order to know when it's time to run. Without canRun statements, timers alone may 
//reach thresholds in the middle of loop() and only some functions may end up running.
//canRun's increment up, triggering different functions as they go up to prevent bogging down
unsigned long currentMillis = 0;  //for global timing features (milliseconds)
unsigned long currentMicros = 0;  //for global timing features (microseconds)
unsigned long timer1ms = 0; //micro() container for functions that will run every 1 millisecond (microseconds needed for accuracy at the 1 milliseond level)
unsigned long timer10ms = 0; //milli() container for functions that will run every 10 ms
unsigned long timer20ms = 0; //milli() container for functions that will run every 20 ms
bool canRun1ms = true; //functions watch these ints to know if they are allowed to run
bool canRun10ms = true;
bool canRun20ms = true;


//array of pins to run multiplexer commands
int muxPins1[] = {
  BUTTON_MUX_PIN_1,BUTTON_MUX_PIN_2,BUTTON_MUX_PIN_3
};
//multiplexer commands that match corresponding muxPins
bool muxCommands1[8][3] = {
  {0,0,0}, //button0   track change / note length change
  {1,0,0}, //button1   pitch change / velocity change
  {0,1,0}, //button2   tempo change / track length
  {1,1,0}, //button3   channel change / instrument change
  {0,0,1}, //button4   save / confirm save
  {1,0,1}, //button5   load / confirm load
  {0,1,1}, //button6   play or pause
  {1,1,1}, //button7   stop
};

//-- EEPROM VARIABLES --// variables that are saved to EEPROM when saving tracks
byte trackLength = 16; // 1-16 beats per song
int tempo = 1200; //BPM * 10 to be able to use int instead of float, and still increment by .5 BPM. updateTempoValues resolves the math by * 0.1
//tracks 1-12, values are {channel, pitch, start note (1st beat is 0), qty of notes, instrument #, note length (currently beat qty), velocity, note spread, unused, unused}
byte trackContainer[TRACK_QTY][TRACK_VARIABLE_QTY] = { {1,63,0,1,0,1,63,8,0,0}, {2,63,0,0,0,1,63,8,0,0}, {3,63,0,0,0,1,63,8,0,0}, {4,63,0,0,0,1,63,8,0,0}, {5,63,0,0,0,1,63,8,0,0}, {6,63,0,0,0,1,63,8,0,0}, {7,63,0,0,0,1,63,8,0,0}, {8,63,0,0,0,1,63,8,0,0}, {9,63,0,0,0,1,63,8,0,0}, {11,63,0,0,0,1,63,8,0,0}, {12,63,0,0,0,1,63,8,0,0}, {13,63,0,0,0,1,63,8,0,0} };
#define VALID_SAVE_BYTE 99 //validation number to check for in save slot checks and to save to SAVE_BYTE_ADDRESS to mark that a valid save exists in a given slot

//Tempo Variables
int lastTempo = 0; //to detect changes in tempo to prevent unneccesary math
#define MINIMUM_TEMPO 300 //lowest tempo can be manually set
#define MAXIMUM_TEMPO 6000 //highest tempo can be manually set
bool tempoInternal = true; //1:internal source, 0:external source for tempo control 
int msBetweenBeats = 500; //used to fill with tempo-based math for beat timers
long unsigned lastBeatMillis = 0; //when last beat happened to detect if it's time for a new beat to happen
bool beatJustChanged = false;  //triggers once-per-beat actions like playing notes

//Helper Variables
byte previousPage = 0; //to detect changes in page for button action decision

//LED variables
byte ledPreviousTrack = 0; //for flashing leds on page 0 when a track changes
unsigned int ledContainer[3] = {
  0b0000000000000000,
  0b0000000000000000,
  0b0000000000000000
};
byte flashTrackLED = 0; //flashes the currentTrack's led colors on the sequencer page 0 (set to 10 for a single 200ms flash, set to 20 for a double flash)
//Brightness Variables
byte ledBrightness = 2;  //controls the PWM to the 595 chips to control overall LED brightness
byte previousLEDBrightness = 2; //used to redo math when ledBrightness changes, and to calculate when to save to EEPROM (and prevent saving too often)
byte beatFlash = 3; //controls the intensity of the beat flashing, 0 to turn off
byte previousBeatFlash = 3; //to calculate when to save to EEPROM
bool changeBrightness = true; //for inputDetections() and ledControl() to communicate with each other about when to change brightness (inputDetection watches previousBrightness and handles saving to EEPROM, ledControl handles actually displaying brightness )
bool allowBeatFlash = true; //allows processes to disable beatFlash'ing temporarily (only stays disabled while actively being set to false, re-enables each ledControl() cycle) 


//-- SETUP --
void setup() {

  //Begin I2C 
  softWire.begin();

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
//  MIDIUSB.begin(MIDI_CHANNEL_OMNI);

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
  
  inputDetection();

  if(canRun1ms) { 

    updateButtons();
    
  }
  
  
 if(canRun20ms) {
  
    updateTempoValues();
    
    updateNoteTriggers();
    
    beatCheck();
    
    handleNotes();
    
    ledControl();
    
  }

}
//-- END OF LOOP --


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


//-------------------//
//-- MUSIC CONTROL --//
//-------------------//


//check midi inputs for messages and forward them between serial midi/usb port
void readMIDI() {
  
  if(MIDI.read()) {
//    MIDIUSB.send(MIDI.getType(),MIDI.getChannel(),MIDI.getData1(),MIDI.getData2());
  }
//  if(MIDIUSB.read()) {
 //   MIDI.send(MIDIUSB.getType(),MIDIUSB.getData1(),MIDIUSB.getData2(),MIDIUSB.getChannel());
//  }
}


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


//plays and stops playback of MIDI notes
void handleNotes() {
  //check all tracks for notes that need turned off with lastNoteMillis
  for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){ //cycle through each track
    if(lastNoteMillis[trackCycler] != 0) {  //if a note was played for a track, check the timing to see if it's time send the note off command
      if(currentMillis >= lastNoteMillis[trackCycler] + (trackContainer[trackCycler][NOTE_LENGTH] * msBetweenBeats) || !playing) {  //check NOTE_LENGTH math for current track and also check if playback should be full-stopped
        MIDI.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]); //send out noteOff commands
//        MIDIUSB.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
        lastNoteMillis[trackCycler] = 0; //empty out this track's lastNoteMillis[] slot
      }
    }
  }

  if(beatJustChanged){ //if beatCheck() just advanced to the next beat
    for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){ //cycle through each track
      if(noteTriggers[trackCycler][currentBeat - 1]) { //if a track is set to play a note this beat
        MIDI.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]); //
        MIDI.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
        
  //      MIDIUSB.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]);
 //       MIDIUSB.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
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
//    MIDIUSB.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
    lastNoteMillis[trackCycler] = 0;
  }
}


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
    softWire.beginTransmission(diskAddress);
    softWire.write((int)(byteAddress >> 8));   // MSB
    softWire.write((int)(byteAddress & 0xFF)); // LSB
    softWire.endTransmission();
    softWire.requestFrom(diskAddress,qtyBytes);
    if (softWire.available() == 1) {
      rdata = softWire.read();
    } else if (softWire.available() > 1) {
      byte high = softWire.read();
      byte low = softWire.read();
      rdata = word(high,low);
    }
  }
  return rdata;
}


//put 1 or 2 bytes to external eeprom
void putEepromByte(byte diskAddress, int byteAddress, byte qtyBytes, int data) { 
  softWire.beginTransmission(diskAddress);
  softWire.write((int)(byteAddress >> 8));   // MSB
  softWire.write((int)(byteAddress & 0xFF)); // LSB
  if (data < 256) { //one byte
    softWire.write(data);
  } else if (data > 255 && data < 32768) { //two bytes
    softWire.write((int)(data >> 8)); //write MSB
    softWire.write((int)(data & 0xFF)); //write LSB
  }
  softWire.endTransmission();
  delay(5);
}

void putEepromTrackContainer(byte diskAddress) {
  int byteAddress = (saveLoadSlot * 128) + TRACK_CONTAINER_ADDRESS;
  
  softWire.beginTransmission(diskAddress);
  softWire.write((int)(byteAddress >> 8));   // MSB
  softWire.write((int)(byteAddress & 0xFF)); // LSB
  
  byte trackSendLimit = 3; //limit the amount of tracks written to keep from overflowing the arduino's softwareWire 32 byte buffer limit (2 bytes used for eeprom byte save location)
  for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
    if (trackCycler == trackSendLimit) { //triggers sending off of buffer of 30 bytes if more bytes are going to need to be sent afterward
      softWire.endTransmission();
      delay(5);
      byteAddress += 30; //breaks up maximum potential trackContainer size (120) into 4 sends to prevent softwarewire buffer overflow
      softWire.beginTransmission(diskAddress);
      softWire.write((int)(byteAddress >> 8));   // addresses are ints (2 bytes), so first send the MSB
      softWire.write((int)(byteAddress & 0xFF)); // ...then send the LSB
      trackSendLimit += 3; //increase send limit to capture next set of 3 tracks using trackCycler
    }
    for(byte variableCycler = 0; variableCycler < TRACK_VARIABLE_QTY; variableCycler++){
      softWire.write(trackContainer[trackCycler][variableCycler]);
    }
  }
 softWire.endTransmission();
 delay(5);
  
}


//---------------------//
//-- INPUT DETECTION --//
//---------------------//

//update encoder state variables
//Checks if the first pin of the encoder has changed. If it has, it compares it to the second pin to see if the encoder increased or decreased.
void updateEncoder(){
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



//updates buttonState array to reflect current buttons being pressed
void updateButtons() {
  
  for (byte i = 0; i < 8; i++) {     //for each button....
    for (byte j = 0; j < 3; j++) {   //set the multiplexer for the correct pot
      digitalWrite(muxPins1[j], muxCommands1[i][j]);
    }
    //delayMicroseconds(5); //delay 5 microseconds (0.005 millis) to let mux stabilize (I can't believe I'm using delay =X )
    buttonState[i] = !digitalRead(BUTTON_PIN);
  }
  encoderButtonState = !digitalRead(ENCODER_BUTTON_PIN);
}


void inputDetection() {
  
  //SET PAGE TO DISPLAY
  
  //BRIGHTNESS PAGE
  //if both buttons are held down... 
  if(buttonState[TEMPO_BUTTON] && buttonState[CHANNEL_BUTTON]) {
    //and the button delay has passed... (button delay on multibutton menus have 50 ms subtracted to give multibutton menus a mild priority over single button menus, ends up making for a cleaner interface feel)
    if (currentMillis > lastButtonMillis[TEMPO_BUTTON] + BUTTON_DELAY - 50 && currentMillis > lastButtonMillis[CHANNEL_BUTTON] + BUTTON_DELAY - 50) {
      lastButtonMillis[TEMPO_BUTTON] = currentMillis; //update button milli timer checking (helps prevent misdetection and changing to single button menus when multibutton ones are attempted)
      lastButtonMillis[CHANNEL_BUTTON] = currentMillis;
      millisForPages = currentMillis; //update page 0 timer reset checking
      currentPage = BRIGHTNESS_PAGE; //and change the page
    }
  //ERASE PAGE
  } else if(buttonState[SAVE_BUTTON] && buttonState[LOAD_BUTTON]) {
    if(currentMillis > lastButtonMillis[SAVE_BUTTON] + BUTTON_DELAY - 50 && currentMillis > lastButtonMillis[LOAD_BUTTON] + BUTTON_DELAY - 50) {
      lastButtonMillis[SAVE_BUTTON] = currentMillis;
      lastButtonMillis[LOAD_BUTTON] = currentMillis;
      millisForPages = currentMillis;
      currentPage = ERASE_PAGE;
    }
  
  //NOTESPREAD PAGE  
  } else if (buttonState[SEQUENCER_BUTTON] && buttonState[PITCH_BUTTON]) {
    if(currentMillis > lastButtonMillis[SEQUENCER_BUTTON] + BUTTON_DELAY - 50 && currentMillis > lastButtonMillis[PITCH_BUTTON] + BUTTON_DELAY - 50) {
      lastButtonMillis[SEQUENCER_BUTTON] = currentMillis;
      lastButtonMillis[PITCH_BUTTON] = currentMillis;
      millisForPages = currentMillis;
      currentPage = NOTESPREAD_PAGE;
    }
  
  //SINGLE-BUTTON PAGES  
  } else {
    
    for(byte checkButton = 0; checkButton < 6; checkButton++){
      //this code causes simultanious pushes to resolve to the farthest down the list if double-button pages didn't tigger earlier
      if(buttonState[checkButton] && currentMillis > lastButtonMillis[checkButton] + BUTTON_DELAY){  //if a button is pushed
        lastButtonMillis[checkButton] = currentMillis;
        currentPage = checkButton;   //set the page to that button
        millisForPages = currentMillis; //update the timing of button push for page 0 reset checks
      }
    }
  }
  
  //SECOND LEVEL PAGES
  if ((currentPage >= SEQUENCER_PAGE && currentPage <= CHANNEL_PAGE) || currentPage == BRIGHTNESS_PAGE) { //if on a single-button page that has a second page assigned to the button (encoder button page)...
    if(currentPage == SEQUENCER_PAGE) {
      if(buttonState[SEQUENCER_BUTTON] && encoderButtonState) {
      currentPage += 10; //update the page number (second level page numbers are always 10 higher than their root page)
      lastButtonMillis[SEQUENCER_BUTTON] = currentMillis;
      lastButtonMillis[ENCODER_BUTTON] = currentMillis; //update encoder button timer millis
      millisForPages = currentMillis; //update page 0 reset timer millis
      }
      
    } else if(encoderButtonState) { //and the encoder button is held down...
      currentPage += 10; //update the page number
      lastButtonMillis[ENCODER_BUTTON] = currentMillis; //update encoder button timer millis
      millisForPages = currentMillis; //update page 0 reset timer millis
    }
  }
  
  if (encoderState != 0) {
    encoderWasChanged = true;
    saveLoadConfirmation = false;
    millisForPages = currentMillis;
    
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
  
  //remove confirmation status for saving and loading if the page has changed
  if(previousPage != currentPage) {
    saveLoadConfirmation = false;
  }
  
  //DETECT USER INPUT
    
  //PAGE INPUTS - SEQUENCER
  if(currentPage == SEQUENCER_PAGE) {
    
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
    
  //PAGE INPUTS - PITCH
  } else if (currentPage == PITCH_PAGE){
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      if(lastNoteMillis[currentTrack] != 0){ //if the track has started playing a note already, stop it
        MIDI.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
//          MIDIUSB.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
//          MidiUSB.flush();
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
    
  //PAGE INPUTS - TEMPO
  } else if (currentPage == TEMPO_PAGE){
    
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
        
  //PAGE INPUTS - INSTRUMENT
  } else if (currentPage == INSTRUMENT_PAGE){

    
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
    
  //PAGE INPUTS - SAVE, LOAD, AND ERASE
  } else if (currentPage == SAVE_PAGE || currentPage == LOAD_PAGE || currentPage == ERASE_PAGE){
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
    
  //PAGE INPUTS - NOTE SPREAD
  } else if (currentPage == NOTESPREAD_PAGE) {
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      trackContainer[currentTrack][NOTE_SPREAD] += encoderState; //... then change the Note Spread of the current track with encoderState
      //cleanup for Note Spread values 
      trackContainer[currentTrack][NOTE_SPREAD] = cleanupByteValue(trackContainer[currentTrack][NOTE_SPREAD],1,8,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
  //PAGE INPUTS - BRIGHTNESS
  } else if (currentPage == BRIGHTNESS_PAGE) {
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      ledBrightness += encoderState; //... then change the display's brightness with ledBrightness
      //cleanup for ledBrightness values 
      ledBrightness = cleanupByteValue(ledBrightness,1,16,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
      changeBrightness = true; //trigger for ledControl() to update the brightness;
    }
    
  //Encoder button used to access all 10+ pages
  //PAGE INPUTS - NOTE LENGTH 
  } else if (currentPage == NOTE_LENGTH_PAGE){
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      trackContainer[currentTrack][NOTE_LENGTH] += encoderState;
      trackContainer[currentTrack][NOTE_LENGTH] = cleanupByteValue(trackContainer[currentTrack][NOTE_LENGTH],1,16,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }
    
  //PAGE INPUTS - VELOCITY
  } else if (currentPage == VELOCITY_PAGE){
    
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
    
  //PAGE INPUTS - TRACK LENGTH
  } else if (currentPage == TRACK_LENGTH_PAGE){
    
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
    
  //PAGE INPUTS - CHANNEL
  } else if (currentPage == CHANNEL_PAGE){
    
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

  //PAGE INPUTS - BEATFLASH
  } else if (currentPage == BEATFLASH_PAGE) {

    //if the encoder was spun and the encoder's button delay has passed....
    if(encoderWasChanged && currentMillis > lastButtonMillis[ENCODER_SPIN] + ENCODER_DELAY) {
      beatFlash += encoderState; //... then change the intensity of beatFlash with encoderState
      //cleanup for beatFlash values 
      beatFlash = cleanupByteValue(beatFlash,0,16,false);
      lastButtonMillis[ENCODER_SPIN] = currentMillis; //update rotory encoder delay detection after accepting input
    }

    
  }
  
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
      currentPage = 0;
    }
  }
  
  //clear out buttonWasPressed && encoderWasChanged
//    for(byte buttonSet = 0; buttonSet < 9; buttonSet++) {
//      buttonWasPressed[buttonSet] = 0;
//    }

  encoderWasChanged = false;
  encoderState = 0; //reset back to neutral state to prevent misdetection

  previousPage = currentPage; // update page detection

}


//-----------------//
//-- LED DISPLAY --//
//-----------------//


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


//LED DISPLAY CONTROL LOOP
void ledControl() {
  
  byte ledOff = 0b00000000;
  
  int brightnessFactor = 6;//less brightness for non-prototype running 595 chips on the 5v rail instead of prototype's 3v 595 design
  if (prototype) {brightnessFactor = 16;}
  
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

  // -- PAGE 0 LED DISPLAY (sequencer)-- //
  if(currentPage == 0) {
    
    bool trackColors[TRACK_QTY][3] = { //RGB colors for each track, repeats due to limited colors
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
    
    
  // -- LED DISPLAY - PITCH -- //      
  } else if (currentPage == PITCH_PAGE) {
    
    byte tensColor[3] = {0,0,1};
    byte selectionColor[3] = {0,1,0};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][PITCH]);
    
    
  // -- LED DISPLAY - TEMPO -- //      
  } else if (currentPage == TEMPO_PAGE) {
   
  byte hundredsColor[3] = {1,0,0};
  byte tensColor[3] = {0,0,1};
  byte selectionColor[3] = {0,1,0};
  
  countByHundredsLED(hundredsColor, tensColor, selectionColor, (tempo * 0.1) - 1); //tempo 8 0.1 to turn it into BPM, -1 to do LED function math propery (zero based counting system)
 
  
    
  // -- LED DISPLAY - CHANNEL -- //      
  } else if (currentPage == CHANNEL_PAGE) {
    
    //RGB for channel display
    byte trackChannelColors[3] = {0,0,1};
    
    for(byte channelCheck = 0; channelCheck < trackContainer[currentTrack][CHANNEL]; channelCheck++) { //fill the ledContianer based off the channel number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(trackChannelColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - channelCheck);
        }
      }
    }
    
  // -- LED DISPLAY - SAVE, LOAD, AND ERASE -- //      
  } else if (currentPage == SAVE_PAGE || currentPage == LOAD_PAGE || currentPage == ERASE_PAGE) {
  
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
  
  
  // -- LED DISPLAY - NOTESPREAD -- //
  } else if (currentPage == NOTESPREAD_PAGE) {
    //RGB for channel display
    byte noteSpreadColors[3] = {0,1,0};
    
    for(byte noteSpreadCheck = 0; noteSpreadCheck < trackContainer[currentTrack][NOTE_SPREAD]; noteSpreadCheck++) { //fill the ledContianer based off the channel number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(noteSpreadColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - noteSpreadCheck);
        }
      }
    }
  
  // -- LED DISPLAY - BRIGHTNESS -- //
  }else if (currentPage == BRIGHTNESS_PAGE) {
    
    //RGB for brightness display
    byte brightnessColors[3] = {1,1,1};
    
    for(byte brightnessCheck = 0; brightnessCheck < ledBrightness; brightnessCheck++) { //fill the ledContianer based off the ledBrightness number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(brightnessColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - brightnessCheck);
        }
      }
    }
    
  // -- LED DISPLAY - NOTE LENGTH -- //      
  } else if (currentPage == NOTE_LENGTH_PAGE) {
    

    
    byte noteLengthColors[3] = {1,1,0};
    for(byte lengthCheck = 0; lengthCheck < trackContainer[currentTrack][NOTE_LENGTH]; lengthCheck++) { //fill the ledContianer based off the trackLength...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(noteLengthColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - lengthCheck);
        }
      }
    }  
  
  
  // -- LED DISPLAY - VELOCITY -- //      
  } else if (currentPage == VELOCITY_PAGE) {
    
    byte tensColor[3] = {1,0,1};
    byte selectionColor[3] = {0,1,0};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][VELOCITY]);
    

  
  // -- LED DISPLAY - TRACK LENGTH -- //      
  } else if (currentPage == TRACK_LENGTH_PAGE) {
    
    //RGB for page 10 display
    byte trackLengthColors[3] = {1,0,0};
    
    for(byte lengthCheck = 0; lengthCheck < trackLength; lengthCheck++) { //fill the ledContianer based off the trackLength...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(trackLengthColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - lengthCheck);
        }
      }
    }
 
  
  
  // -- LED DISPLAY - INSTRUMENT -- //      
  } else if (currentPage == INSTRUMENT_PAGE) {

    byte tensColor[3] = {0,1,0};
    byte selectionColor[3] = {1,0,1};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][INSTRUMENT]);
    
    
  // -- LED DISPLAY - BEATFLASH -- // 
  } else if (currentPage == BEATFLASH_PAGE) {

    //RGB for beatflash display
    byte beatFlashColors[3] = {0,1,1};
    
    for(byte beatFlashCheck = 0; beatFlashCheck < beatFlash; beatFlashCheck++) { //fill the ledContianer based off the ledBrightness number...
      for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
        if(beatFlashColors[colorCheck]) {
          bitSet(ledContainer[colorCheck], 15 - beatFlashCheck);
        }
      }
    }

    
  }
  
  
  // -- PUSH CONTAINER TO LEDS -- //
  //ledContainer is only 3 ints, but is treated as a 3 long bits. So only 3 numbers need sent to the shift registers
  //prototypes do single color 595 chips, new boards will be wired to full LEDs and each 595 will send multiple colors
  if(prototype == false) {
    unsigned int reformatLedContainer[3] = {
      0b0000000000000000,
      0b0000000000000000,
      0b0000000000000000
    };
    
    byte reformatContainerChipPlaceholder = 0;
    byte reformatContainerLEDPlaceholder = 0;
    //cycle through the ledContainer bits to rearrange the colors to the 595 multicolor chip format
    for(byte ledCountNumber = 0; ledCountNumber < 16; ledCountNumber++){
      //cycle through RGB inside the ledContainer
      for(int colorSelect = 0; colorSelect < 3; colorSelect++) {
        if(bitRead(ledContainer[colorSelect], 15 - ledCountNumber)) {
          bitSet(reformatLedContainer[reformatContainerChipPlaceholder], 15 - reformatContainerLEDPlaceholder);
        }
        reformatContainerLEDPlaceholder += 1;
        if(reformatContainerLEDPlaceholder >= 16) {
          reformatContainerLEDPlaceholder = 0;
          reformatContainerChipPlaceholder +=1;
        }      
      }
    }
    
    for(int i = 0; i < 3; i++) {
      ledContainer[i] = reformatLedContainer[i];
    }
  }

  for(int j = 2; j >= 0; j--) {
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, lowByte(ledContainer[j])); 
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, highByte(ledContainer[j]));
  }
  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(LATCH_PIN, LOW);
}













// ---------------------------------------------------
// Code below this is not actively used by the program, and has been replaced or removed completely
// Kept as reference for now
// ---------------------------------------------------

/* Removed for now, handles receiving external clock, start, stop, continue messages
//handles receiving external clock signals for tempo 
void handleClock(){
  if(!tempoInternal) {
    lastClockMessageMicros = currentClockMessageMicros;
    currentClockMessageMicros = currentMicros;
    if (lastClockMessageMicros != 0 && currentClockMessageMicros != 0) {
      //clock message are at a rate of 24 per quarter note (beat)
      //find the difference in millis between the messages 
      //and math them to find tempo and ms between beats
      msBetweenBeats = round((currentClockMessageMicros - lastClockMessageMicros) * 0.024); //equivalent to (((currentClockMessageMicros - lastClockMessageMicros)/1000)*24) - find difference, go from micros to millis, miltiply by 24 because 24 clock signals per quarter note (beat)
      tempo = 600000 / msBetweenBeats; //tempo is BPM * 10 for math purposes, so it's 600000 instead of 60000
    }
    if (midiStartCommand) {  //if handleStart or handleContinue received a start command, go ahead and start playing
      playing = true;
      midiStartCommand = false;
    }
  }
}
//midi message handle
void handleStart(){
  if(!tempoInternal && !playing) {
    currentBeat = 0; //reset to beginning of the tracks
    midiStartCommand = true; //and prepare to play on next midi clock signal
  }
}
//midi message handle
void handleStop(){
  if(!tempoInternal) {
    playing = false;
  }
}
//midi message handle
void handleContinue(){
  if(!tempoInternal && !playing) { //do I need to subtract 1 from the current beat to make this work?
    midiStartCommand = true;
  }
}
*/


//OLD BUTTON INPUT/PAGE SELECTION
    /*
    //if Save and Load buttons are both held down, go to Erase page
    if(buttonState[SAVE_BUTTON] && buttonState[LOAD_BUTTON]) {
      currentPage = 6;   //set the page to erase page
      buttonWasPressed[SAVE_BUTTON] = true; //and mark it pushed
      buttonWasPressed[LOAD_BUTTON] = true;
      lastButtonMillisForPages[SAVE_BUTTON] = currentMillis; //update the timing of button push for page 0 reset check
      lastButtonMillisForPages[LOAD_BUTTON] = currentMillis;
    }    
    //if Sequencer and Pitch buttons are both held down, go to Note Spread page
    if(buttonState[0] && buttonState[1]) {
      currentPage = 7;   //set the page to the notespread page
      //lastButtonMillisForPages[0] = currentMillis - page0Timeout - 100;
      //lastButtonMillisForPages[1] = currentMillis - page0Timeout - 100;
    }
    //if Tempo and Channel buttons are both held down, go to Brightness page
    if(buttonState[2] && buttonState[3] && !encoderButtonState) {
      currentPage = 9;   //set the page to the brightness page
      //lastButtonMillisForPages[2] = currentMillis - page0Timeout - 100;
      //lastButtonMillisForPages[3] = currentMillis - page0Timeout - 100;
        
    //if Tempo, Channel, and also Encoder buttons are are held down, go to beat flash page
    } else if (buttonState[2] && buttonState[3] && encoderButtonState) {
      currentPage = 19;
      lastButtonMillisForPages[2] = currentMillis - page0Timeout - 100;
      lastButtonMillisForPages[3] = currentMillis - page0Timeout - 100;
    }
    if(encoderButtonState) {
      if(currentPage == 0 && buttonState[0]) { //if on sequencer page and track/length button is held while encoder is pushed
        currentPage += 10; //add 10 to go to track length change page
      } else if (currentPage > 0 && currentPage < 4){  //if on a page that can have a second level
        currentPage += 10; //add 10 to go to second level of pages
      }
      buttonWasPressed[8] = true;
      lastButtonMillisForPages[8] = currentMillis;  //to help decide when to go back to page 0
    }
    */





//SAVING TO INTERNAL EEPROM
/* Old Method (went to expernal eeprom chip for track saving) - cannibalize for new method of internal saving (for general settings like brightness)
void saveTrack(String saveType) {
  
  //each saveslot is afforded 64 bytes of data
  
  #define SAVE_BYTE_ADDRESS 0   //0 value means save is empty, 99 value shows occupied
  #define TRACK_LENGTH_ADDRESS 1
  #define TEMPO_ADDRESS 2
  #define TRACK_CONTAINER_ADDRESS 6
  
  int slotOffset =  (saveLoadSlot * 64);
  int eepromAddressHelper = SAVE_BYTE_ADDRESS + slotOffset;
  
  if (saveType == "load") {
    byte saveByteCheck = 0;
    EEPROM.get(eepromAddressHelper, saveByteCheck);
    if(saveByteCheck == 99) {
      notesOff();
      eepromAddressHelper = TRACK_LENGTH_ADDRESS + slotOffset;
      EEPROM.get(eepromAddressHelper, trackLength);
      eepromAddressHelper = TEMPO_ADDRESS + slotOffset;
      EEPROM.get(eepromAddressHelper, tempo);
      eepromAddressHelper = TRACK_CONTAINER_ADDRESS + slotOffset;
      for(int trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){
        for(int valueCycler = 0; valueCycler < TRACK_VARIABLE_QTY; valueCycler++) {
          eepromAddressHelper += (trackCycler*TRACK_VARIABLE_QTY) + valueCycler;
          EEPROM.get(eepromAddressHelper,trackContainer[trackCycler][valueCycler]);
        }
      }
    } else {
      notesOff();
      byte emptyTrackContainer[TRACK_QTY][TRACK_VARIABLE_QTY] = { {1,63,0,1,1,1,63,8}, {1,63,0,0,1,1,63,8}, {1,63,0,0,1,1,63,8}, {1,63,0,0,1,1,63,8}, {1,63,0,0,1,1,63,8}, {1,63,0,0,1,1,63,8} };
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        for(byte variableCycler = 0; variableCycler < TRACK_VARIABLE_QTY; variableCycler++){
          trackContainer[trackCycler][variableCycler] = emptyTrackContainer[trackCycler][variableCycler];
        }
      }
    }
  }
  
  if (saveType == "save") {
    EEPROM.put(eepromAddressHelper, 99);
    eepromAddressHelper = TRACK_LENGTH_ADDRESS + slotOffset;
    EEPROM.put(eepromAddressHelper, trackLength);
    eepromAddressHelper = TEMPO_ADDRESS + slotOffset;
    EEPROM.put(eepromAddressHelper, tempo);
    eepromAddressHelper = TRACK_CONTAINER_ADDRESS + slotOffset;
    for(int trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){
      for(int valueCycler = 0; valueCycler < TRACK_VARIABLE_QTY; valueCycler++) {
        eepromAddressHelper += (trackCycler*TRACK_VARIABLE_QTY) + valueCycler;
        EEPROM.put(eepromAddressHelper,trackContainer[trackCycler][valueCycler]);
      }
    }
  }
  
  if (saveType == "erase") {
    //change the save state byte to 0 to cause the program to treat it as empty
    EEPROM.put(eepromAddressHelper, 0);
  }
}
*/



//GOES INTO BUTTON INPUTS TO TRIGGER TEMPO INTERNAL/EXTERNAL
      
    //INTERNAL/EXTERNAL TEMPO CODE
      /*
      //if the encoder was spun and the encoder's button delay has passed....
      if(encoderWasChanged && currentMillis > lastButtonMillis[9] + buttonDelay) {
        tempoInternal = !tempoInternal; //commented out until external tempo source code is implemented
        lastButtonMillis[9] = currentMillis; //update rotory encoder delay detection after accepting input
        if (tempoInternal) {  //reset the external clock message tracking if tempo is set to internal
          lastClockMessageMicros = 0;
          currentClockMessageMicros = 0;
          midiStartCommand = false; //cleanup these triggers just in case
        }
      }
      */
      
      
//SERIAL DEBUG
/*
void DEBUGcycleCount() {
  cycleCount += 1;
 
  if(canRun1ms == 1){
    cycleCount1ms += 1;
  }
  
  if(canRun10ms == 1){
    cycleCount10ms += 1;
  }
  if(canRun20ms == 1){
    cycleCount20ms += 1;
  }
  
  if(currentMillis >= cycleCountMillis + 1000) {
    Serial.print("Cycles/Second: ");
    Serial.println(cycleCount);
    
    Serial.print("MS Delayed CPS: 1ms: ");
    Serial.println(cycleCount1ms);
    
    Serial.print(", 10ms: ");
    Serial.print(cycleCount10ms);
    Serial.print(", 20ms: ");
    Serial.println(cycleCount20ms);
    
    Serial.print("Current Beat: ");
    Serial.println(currentBeat);
    
    Serial.print("Tempo: ");
    Serial.println(tempo);
    
    Serial.print("Button state 6: ");
    Serial.println(buttonState[6]);    
    Serial.print("Button state 7: ");
    Serial.println(buttonState[7]);   
    Serial.print("Encoder Button State: ");
    Serial.println(encoderButtonState);
    
    Serial.println(" ");
   
    cycleCount = 0;
    cycleCount1ms = 0;
    cycleCount10ms = 0;
    cycleCount20ms = 0;
    
    cycleCountMillis = currentMillis;
    
    }
}*/



//SEQUENCER PAGE 0 STUFF

      //if it was already on the sequencer page and the track button was pressed and released with no other inputs, increment the track up one
      
      //****doesn't work as expected, rewriting****
      /*
      if(!buttonState[0] && buttonWasPressed[0] && !encoderWasChanged && !encoderButtonState && previousPage == 0 && currentMillis > lastButtonMillis[0] + buttonDelay){
        currentTrack += 1;
        currentTrack = cleanupByteValue(currentTrack,0,5,true); //cleanup for currentTrack values
        lastButtonMillis[0] = currentMillis;
      }
      //if the encoder was spun and the encoder's button delay has passed....
      if(encoderWasChanged && currentMillis > lastButtonMillis[9] + buttonDelay) {
        if(!buttonState[0]) {  //... and if the track button (0) is not being held down ...
          trackContainer[currentTrack][START_NOTE] += encoderState; //... then add or subtrack encoderState value from the currentTrack's START_NOTE
          //cleanup for START_NOTE values
          trackContainer[currentTrack][START_NOTE] = cleanupByteValue(trackContainer[currentTrack][START_NOTE],0,15,true); 
        } else { //... but if the rotory encoder changes while the track button (0) is instead being held
          if(!encoderButtonState){ //... and if the encoder button is not being held down...
            currentTrack += encoderState; //... then change the currentTrack by the encoderState value
            //cleanup for currentTrack values
            currentTrack = cleanupByteValue(currentTrack,0,5,true);
          } else { //... or if the rotory encoder button is instead being held
            trackContainer[currentTrack][NOTE_QTY] += encoderState; //... then add or subtrack a note from the NOTE_QTY of currentTrack
            //cleanup for NOTE_QTY values
            trackContainer[currentTrack][NOTE_QTY] = cleanupByteValue(trackContainer[currentTrack][NOTE_QTY],0,16,true);
          }
        }
        lastButtonMillis[9] = currentMillis; //update rotory encoder delay detection after accepting input
      } */
