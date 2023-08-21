/******************************************************************************

Sundial Midi Sequencer v2.1.1

ToDo:

    changing to save/load page and coming back causes big delays for USB midi (sends?), hanging program
    
    audit timer system to check if it's streamlined
    
    audit variables to see if they can be changed to #defines instead
    
    check speed of program
    
    cleanup notesOff stuff to include actual noteoff midi commands
  
  midi still implementing:
    usb midi and crosstalk with DIN
    re-implement start/stop/continue command handling?
    receive song position over midi (any other?)
    
  Need to make an EEPROM filler program or method for programming new devices with a standard baseline
  

*******************************************************************************/


#include <Arduino.h>

//EEPROM Libraries
#include <Wire.h> //for saving to external chip eeprom
#include <EEPROM.h> //for saving to internal eeprom memory

//MIDI Libraries
#include <MIDI.h>
#include <USB-MIDI.h> //companion to MIDI.h for usb midi usage

// -- MIDI USB + Serial Initialize -- //
// USB
/*
USING_NAMESPACE_MIDI;
typedef USBMIDI_NAMESPACE::usbMidiTransport __umt;
typedef MIDI_NAMESPACE::MidiInterface<__umt> __ss;
__umt usbMIDI(0); // cableNr
__ss MIDIUSB((__umt&)usbMIDI); //names the object MIDIUSB
typedef Message<MIDI_NAMESPACE::DefaultSettings::SysExMaxSize> MidiMessage;
*/
USBMIDI_CREATE_INSTANCE(0,MIDIUSB)
// Serial
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI)


//------------------//
//-- BOARD SELECT --//
//------------------//

//Select which board is being programmed
//#define V0_PROTOTYPE_BOARD
//#define V1_PCB_BOARD
#define V2_PCB_BOARD

#ifdef V2_PCB_BOARD

  // -- PIN DEFINES -- //

  //Encoder Pins
  #define ENCODER_PIN1 12 //MSB pin of encoder
  #define ENCODER_PIN2 6 //LSB pin of encoder
  #define ENCODER_BUTTON_PIN 4 //encoder button pin
  #define BUTTON_PIN A3 //button multiplexer output pin

  //Multiplexer Pins
  #define BUTTON_MUX_PIN_1 A4
  #define BUTTON_MUX_PIN_2 A5
  #define BUTTON_MUX_PIN_3 7

  //LED Shift Register Pins
  #define BRIGHTNESS_PIN 9
  #define CLOCK_PIN 14
  #define LATCH_PIN 15
  #define DATA_PIN 16

  //I2C EEPROM
  #define SCL_PIN 3
  #define SDA_PIN 2

  //MIDI Related Pins
  #define MIDI_SUPPRESS_PIN 11 //controls transistor (base) link between MIDI out and VS1053B in

#endif


//------------------//
//--    GLOBALS   --//
//------------------//

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
#define MIDI_TRACK_SUPPRESS 8
//one more track variable bytes allowed and currently unused for each instument track (12 instrument tracks per save)
//#define unused_variable 9


//-- GLOBAL VARIABLES --//
byte currentPage = 1; //controls what is displayed on the LED ring and what button inputs do
byte currentTrack = 0; //0-11 for 12 separate tracks
bool playing = true; //is the device playing or paused/stopped?
byte currentBeat = 0; //what beat the track is currently at (stopped = 0, music starts on 1, and rolls over from 16 (or trackLength) to 1)
byte saveLoadSlot = 0; //remembers selected save/load slot
bool saveLoadConfirmation = false; //bool to require second click of encoder to save, load, or erase a track

//Button and Detection Variables
bool buttonState[8] = {0,0,0,0,0,0,0,0}; //container to know if buttons are currently pushed or not
bool encoderButtonState = 0; //container to know if encoder button is being pushed
byte encoderButtonWasPushed = 0; //container to know if encoder button was pushed last loop, used to detect between button push or button hold (0: wasn't pushed, 1: was pushed, 2:hold click happened, don't also push )
char encoderState = 0; //-1 is encoder moved down, 1 is encoder moved up, 0 is no change
//unsigned long lastButtonMillis[10] = {0,0,0,0,0,0,0,0,0,0}; //timers to prevent unintentional button represses (8 buttons, encoder button, encoder knobs)
unsigned long lastPlayButtonMillis = 0; //prevents play button over-detection
unsigned long lastEncButtonMillis = 0;  //prevents encoder button click over-detection
unsigned long lastEncSpinMillis = 0;    //prevents encoder spin over-detection
unsigned long millisForPages = 0; //timers to decide when to change back to page 0 automatically
unsigned long millisPlayButtonWasLastOff = 0; //used to detect if play button has been continually held down by tracking the last time it wasn't pushed
unsigned long millisEncButtonWasLastOff = 0;  //used to detect if encoder button has been continually held down by tracking the last time it wasn't pushed
#define PAGE_0_TIMEOUT 4000 //(pageZEROtimeout) ms without any input before display goes back to page 0 automatically
#define BUTTON_DELAY 180 //ms delay between detecting button presses
#define ENCODER_DELAY 100 //ms delay between detecting encoder knob turns
#define ENCODER_HOLD_DELAY 2000 //time required to detect an encoder held-down button press

//Encoder Math Variables
byte lastEncoderPin1State = 0; //for updateEncoder() math, for checking to see if the encoder moved
int lastEncoderMovementValue = 0; //for fast-spinning detection, will only contain +1 or -1 after first encoder use
//FastSpin Variables 
#define ENCODERMSCONTAINER_SIZE 13 //how large the check array is for fastSpin detection (consolodates variables for changing fastSpin detection)
bool encoderMScontainer[ENCODERMSCONTAINER_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0}; //for fast-spinning detection/math, used by loading with timing info to see how fast the encoder is changing states
long unsigned millisForLastEncoderMScheck = 0; //for detecting fastspin
#define FASTSPIN_THRESHOLD 10 //How many fastSpins need to build up to make fastSpin = true (consolodates variables for changing fastSpin detection) 
bool fastSpin = false;

//MIDI and Note Playing Variables
unsigned long lastNoteMillis[TRACK_QTY] = {0,0,0,0,0,0,0,0,0,0,0,0}; //container for each track to hold when last note was played


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
//unsigned long timer10ms = 0; //milli() container for functions that will run every 10 ms
unsigned long timer20ms = 0; //milli() container for functions that will run every 20 ms
bool canRun1ms = true; //functions watch these ints to know if they are allowed to run
//bool canRun10ms = true;
bool canRun20ms = true;


//array of pins to run multiplexer commands
const int muxPins1[] = {
  BUTTON_MUX_PIN_1,BUTTON_MUX_PIN_2,BUTTON_MUX_PIN_3
};


//-- EEPROM VARIABLES --// variables that are saved to EEPROM when saving tracks
byte trackLength = 16; // 1-16 beats per song
int tempo = 1200; //BPM * 10 to be able to use int instead of float, and still increment by .5 BPM. updateTempoValues resolves the math by * 0.1
//tracks 1-12, values are {channel, pitch, start note (1st beat is 0), qty of notes, instrument #, note length (currently beat qty), velocity, note spread, unused, unused}
byte trackContainer[TRACK_QTY][TRACK_VARIABLE_QTY] = { {1,63,0,1,0,1,63,8,0,0}, {2,63,0,0,0,1,63,8,0,0}, {3,63,0,0,0,1,63,8,0,0}, {4,63,0,0,0,1,63,8,0,0}, {5,63,0,0,0,1,63,8,0,0}, {6,63,0,0,0,1,63,8,0,0}, {7,63,0,0,0,1,63,8,0,0}, {8,63,0,0,0,1,63,8,0,0}, {9,63,0,0,0,1,63,8,0,0}, {11,63,0,0,0,1,63,8,0,0}, {12,63,0,0,0,1,63,8,0,0}, {13,63,0,0,0,1,63,8,0,0} };
#define VALID_SAVE_BYTE 99 //validation number to check for in save slot checks and to save to SAVE_BYTE_ADDRESS to mark that a valid save exists in a given slot, used to invalidate old saves after major changes happen to structure

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
byte flashLEDctrl = 0; //countdown timer that controls selective flashing of LED's like when changing tracks on sequencer page or selecting channels/tracks to MIDI suppress
//Brightness Variables
byte ledBrightness = 2;  //controls the PWM to the 595 chips to control overall LED brightness
byte previousLEDBrightness = 2; //used to redo math when ledBrightness changes, and to calculate when to save to EEPROM (and prevent saving too often)
byte beatFlash = 3; //controls the intensity of the beat flashing, 0 to turn off
byte previousBeatFlash = 3; //to calculate when to save to EEPROM
bool changeBrightness = true; //for inputDetections() and ledControl() to communicate with each other about when to change brightness (inputDetection watches previousBrightness and handles saving to EEPROM, ledControl handles actually displaying brightness )
bool allowBeatFlash = true; //allows processes to disable beatFlash'ing temporarily (only stays disabled while actively being set to false, re-enables each ledControl() cycle) 

//Midi Channel/Track suppression
//controls a transistor between ATmega and VS1053B to prevent VS1053B from playing
bool midiSuppressByChannel = true; //true controls midi suppression to VS1053B by channel, false is by track, pulled from EEPROM
unsigned int midiChannelSuppress = 0b0000000000000000; //stores bits to see if a given midi channel needs suppressed
byte channelTrackSuppressSlot = 0;


//-----------------------------------------//
//--            Memory Control           --//
//-- Internal and External EEPROM Saving --//
//-----------------------------------------//

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



//----------------------------------//
//--           Pages              --//
//-- Control Inputs / LED Outputs --//
//----------------------------------//

//Buttons
//a lot of these aren't actually used in code, but keeping here as reference
//(doesn't increase code size)
#define SEQUENCER_BUTTON 0
#define PITCH_BUTTON 1
#define INSTRUMENT_BUTTON 2
#define TEMPO_BUTTON 3
#define OPTIONS_BUTTON 4
#define PLAY_BUTTON 5
#define SAVE_BUTTON 6
#define LOAD_BUTTON 7




//Pages
//Adding new pages:
// Add a page #define number based off button values to reach page, 
// increase PAGE_QTY, add the define to the validPages[] array,
// and add the led and control parameters to pageControl()

//button values for pages: 
// 1 (sequencer), 2 (pitch), 4 (instrument), 8 (tempo), 
// 16 (options), 32 (save), 64 (load), 128 (encoder)

//single button pages
#define SEQUENCER_PAGE 1
#define PITCH_PAGE 2
#define INSTRUMENT_PAGE 4
#define TEMPO_PAGE 8
#define SAVE_PAGE 32
#define LOAD_PAGE 64

//single button encoder-shifted pages
#define VELOCITY_PAGE 130
#define CHANNEL_PAGE 132
#define TRACK_LENGTH_PAGE 136

//two button pages
#define NOTESPREAD_PAGE 3
#define BRIGHTNESS_PAGE 12
#define ERASE_PAGE 96

//two button encoder-shifted pages
#define NOTE_LENGTH_PAGE 131
#define BEATFLASH_PAGE 140

//Options-Shifted pages
#define MIDI_SUPPRESS_PAGE 17
//#define GLOBAL_VOLUME_PAGE 18    //not yet implemented

#define PAGE_QTY 15 //needs to be equal to total number of pages
//each page needs added to this array (location unimportant)
const byte validPages[PAGE_QTY] = {SEQUENCER_PAGE, PITCH_PAGE, TEMPO_PAGE, INSTRUMENT_PAGE, SAVE_PAGE, LOAD_PAGE, ERASE_PAGE, NOTESPREAD_PAGE, BRIGHTNESS_PAGE, NOTE_LENGTH_PAGE, VELOCITY_PAGE, TRACK_LENGTH_PAGE, CHANNEL_PAGE, BEATFLASH_PAGE, MIDI_SUPPRESS_PAGE};






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
    
    beatCheck();
    
    handleNotes();
    
    pageControl();
    
  }


}
//-- END OF LOOP --





//-----------------------------------------//
//--            Memory Control           --//
//-- Internal and External EEPROM Saving --//
//-----------------------------------------//


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
  if (qtyBytes == 1 || qtyBytes == 2) { //only transmit if the request is for 1 or 2 bytes
    Wire.beginTransmission(diskAddress);
    Wire.write((int)(byteAddress >> 8));   // MSB
    Wire.write((int)(byteAddress & 0xFF)); // LSB
 //causes hangup in usb midi
    Wire.endTransmission();
 //causes hangup in usb midi
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


//------------------//
//-- MIDI Control --//
//------------------//


//check midi inputs for messages and forward them between serial midi/usb port
void readMIDI() {
  
  if(MIDI.read()) {
    MIDIUSB.send(MIDI.getType(),MIDI.getChannel(),MIDI.getData1(),MIDI.getData2());
    MidiUSB.flush();
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
        MidiUSB.flush();
      }
    }
  }

  if(beatJustChanged){ //if beatCheck() just advanced to the next beat
    for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){ //cycle through each track
      checkMIDIsuppression("track",trackCycler);
      if(noteCheck(trackCycler, currentBeat - 1)) { //if a track is set to play a note this beat (current beat is 1-16 if playing music, so a -1 is used for the math)
        MIDI.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]); //
        MIDI.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
        
        MIDIUSB.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]);
        MIDIUSB.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
        lastNoteMillis[trackCycler] = currentMillis; //fill this track's lastNoteMillis[] slot with this note's timing
      MidiUSB.flush();
      }
      digitalWrite(MIDI_SUPPRESS_PIN, HIGH);
    }
    beatJustChanged = false; //turns off beatJustChanged toggle after all notes are calculated for the current beat
  }
}

//sends noteOff for each track and empties out lastNoteMillis[]
void notesOff() {
  for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++){
    MIDI.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
    MIDIUSB.sendNoteOff(trackContainer[trackCycler][PITCH],0,trackContainer[trackCycler][CHANNEL]);
    MidiUSB.flush();
    lastNoteMillis[trackCycler] = 0;
  }
}


void checkMIDIsuppression (String mode, byte trackOrChannelToCheck) {
  if(midiSuppressByChannel) { //if channel mode is enabled
    if(mode == "track") { //for checking if a track's channel should be allowed to play or not
      if(bitRead(midiChannelSuppress,trackContainer[trackOrChannelToCheck][CHANNEL] - 1)) { //check the midiChannelSuppress bits for the current track's channel bit
        digitalWrite(MIDI_SUPPRESS_PIN, LOW); //if channel should be suppressed, set transistor to low
      } 
    } else if (mode == "channel") { //for checking if a specific channel should be played or not
      if(bitRead(midiChannelSuppress,trackOrChannelToCheck)) { //check the midiChannelSuppress bits for the current track's channel bit
        digitalWrite(MIDI_SUPPRESS_PIN, LOW); //if channel should be suppressed, set transistor to low
      }
    }
  } else if(mode == "track") { //for checking if a track should be allowed to play or not
      if(trackContainer[trackOrChannelToCheck][MIDI_TRACK_SUPPRESS]) { //if track mode is enabled
        digitalWrite(MIDI_SUPPRESS_PIN, LOW); //if a track should be suppressed, set transistor to low
      }
  } 
}

void usbMIDIout(    ) {
  
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



//checks if a given note in a given track is active or not 
//(0,0) is the first note of the first track
bool noteCheck (byte checkTrackNumber, byte checkNoteNumber) {
  if (trackContainer[checkTrackNumber][NOTE_QTY] > 0) {
    int nextNoteLocation = 0 + trackContainer[checkTrackNumber][START_NOTE];
    
    if (nextNoteLocation == checkNoteNumber) { return true; }
    
    byte remainingNotes = trackLength; //set the available amount of room to spread notes out to match the song's trackLength
    for(byte noteCounter = 1; noteCounter < trackContainer[checkTrackNumber][NOTE_QTY]; noteCounter++) { //cycle once for each note remaining in the track
      byte nextGap = ceil(remainingNotes / (trackContainer[checkTrackNumber][NOTE_QTY] - noteCounter + 1)); //calculate the distance (in beats) to place the next note by dividing the available room to work with (remainingNotes) by how many notes are left to calculate and rounding up
      if(nextGap > trackContainer[checkTrackNumber][NOTE_SPREAD]) { //if this note's distance would be farther away than the track's NOTE_SPREAD variable...
        nextGap = trackContainer[checkTrackNumber][NOTE_SPREAD];    //then put this note at NOTE_SPREAD's distance instead of the calculated nextGap location
      }
      remainingNotes -= nextGap; //subtract the distance used to place this note from the available space remaining to place more notes
      nextNoteLocation += nextGap; //add the nextGap's distance to the last note's placement to get this note's placement location on the track
      if(nextNoteLocation > (trackLength - 1)) { //if the placement is outside of the track's availalbe beats...
        nextNoteLocation -= trackLength;         //then subract the trackLength to rollover the location to the start of the track
      }
      if (nextNoteLocation == checkNoteNumber) { return true; }
    }
  }
  return false;
}

//---------------//
//-- Utilities --//
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


//----------------------------------//
//--       Pages Handling         --//
//-- Control Inputs / LED Outputs --//
//----------------------------------//

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
  
  //multiplexer commands that match corresponding muxPins
  const bool muxCommands1[8][3] = {
  {0,0,0}, //button0   track change / note length change
  {1,0,0}, //button1   pitch change / velocity change
  {0,1,0}, //button2   tempo change / track length
  {1,1,0}, //button3   channel change / instrument change
  {0,0,1}, //button4   save / confirm save
  {1,0,1}, //button5   load / confirm load
  {0,1,1}, //button6   play or pause
  {1,1,1}, //button7   stop
  };

  //Update Button States
  for (byte i = 0; i < 8; i++) {     //for each button....
    for (byte j = 0; j < 3; j++) {   //set the multiplexer for the correct pot
      digitalWrite(muxPins1[j], muxCommands1[i][j]);
    }
    //delayMicroseconds(5); //delay 5 microseconds (0.005 millis) to let mux stabilize (I can't believe I'm using delay =X )
    buttonState[i] = !digitalRead(BUTTON_PIN);
  }
  encoderButtonState = !digitalRead(ENCODER_BUTTON_PIN);
  if(!buttonState[PLAY_BUTTON]) {
    millisPlayButtonWasLastOff = currentMillis;
  }
  if(!encoderButtonState) {
    millisEncButtonWasLastOff = currentMillis;
  }
  byte pageCounter = 0;
  //buttons 1-8 and encoder button (button 5 is play/stop, not used for page changing and set to value of 0)
  const byte buttonPageValues[9] = {1,2,4,8,16,0,32,64,128};
  //delay page changes a little bit, this allows letting go of buttons without changing the page
  if(currentMillis > millisForPages + 150) {
    //add button values to determine page number
    for(byte buttonCycler = 0; buttonCycler < 8; buttonCycler++) { //button cycler is the quantity of button page values - 1 (for encoder button)
      if (buttonState[buttonCycler]) {
        pageCounter += buttonPageValues[buttonCycler];
      }
    }
    if (encoderButtonState) {
      millisForPages = currentMillis;
      pageCounter += buttonPageValues[8]; //encoder value
    } 
    
    
    //check to make sure a valid page is landed on 
    //(prevents page switching to invalid value if random buttons are pushed)
    //this also has a subtle effect of freeing up the encoder button to be used 
    //as a control input as long as a page button isn't being held down as well
    for (byte pageChecker = 0; pageChecker < PAGE_QTY; pageChecker++) {
      if (pageCounter == validPages[pageChecker]) {
        currentPage = pageCounter;
        millisForPages = currentMillis;
      }
    }
  }

}



//---------------------------------------------//
//--  PAGE DISPLAY / CONTROL INTERPRETATION  --//
//---------------------------------------------//


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





//handles button input interpretation and displaying LEDs
void pageControl() {
  
  
  //used by pages to detect encoder spin input and cut down on redundant checks
  bool validEncoderSpinChange = false; //reference for if the encoder registered a changed in UP/DOWN spin state (also considers timer delay/debounce)
  bool validEncoderButtonPress = false; //reference for if the encoder registered an activation in button state (also considers timer delay/debounce)
  bool validEncoderButtonHold = false; //reference for if the encoder registered a held-down activation in button state (also considers timer delay/debounce)

  if (encoderState != 0) {
    
    saveLoadConfirmation = false;    //reset save/load confirm if encoder position changed
    millisForPages = currentMillis;  //reset clock to auto-return to page 1 if encoder changed
    
    
    //FastSpin Encoder Detection (fastSpin Code Section 1 of 2)
    if(encoderState != lastEncoderMovementValue) { //if encoder changed spinning direction, reset fastspin counter to zero
      for (byte resetCounter = 0; resetCounter < ENCODERMSCONTAINER_SIZE; resetCounter++) {
        encoderMScontainer[resetCounter] = false;
      }
    //if the encoder moved in the same direction as previous movement, add one to fastspin counter
    } else { 
      for(byte moveDigits = 1; moveDigits < ENCODERMSCONTAINER_SIZE; moveDigits++) { //bump all the values down the container 1 slot
        encoderMScontainer[moveDigits - 1] = encoderMScontainer[moveDigits];
      }
      encoderMScontainer[ENCODERMSCONTAINER_SIZE - 1] = true; //mark last slot of container true
    }
    byte encoderFastSpinCounter = 0;
    //count how many trues are in the container to compare against threshold
    for (byte fastSpinCheck = 0; fastSpinCheck < ENCODERMSCONTAINER_SIZE; fastSpinCheck++ ){
      if(encoderMScontainer[fastSpinCheck]){
        encoderFastSpinCounter += 1;
      }
    }
    //if the amount of trues in fastspin container meets or exceeds the fastspin threshold, mark fastspin as true
    if (encoderFastSpinCounter >= FASTSPIN_THRESHOLD) {
      fastSpin = true;
    } else {
      fastSpin = false;
    }      
    
    
    lastEncoderMovementValue = encoderState; //update last encoder change value
    
    //Has enough time passed to register a control input from encoder?
    if(currentMillis > lastEncSpinMillis + ENCODER_DELAY) {
      validEncoderSpinChange = true;  //pages look for this value to be true in order to perform encoder-spin actions
      lastEncSpinMillis = currentMillis; //update rotory encoder delay detection after accepting input
    }    
    
  }
  
  
  //FastSpin Encoder Detection (fastSpin Code Section 2 of 2) (needs to be outisde encoderState check)
  //every time the encoder delay ticks, add a false to the fastspin container
  if (currentMillis > millisForLastEncoderMScheck + ENCODER_DELAY) {
    for(byte moveDigits = 1; moveDigits < ENCODERMSCONTAINER_SIZE; moveDigits++) { //bump all the values down the container 1 slot
      encoderMScontainer[moveDigits - 1] = encoderMScontainer[moveDigits];
    }
    encoderMScontainer[ENCODERMSCONTAINER_SIZE - 1] = false;
    millisForLastEncoderMScheck = currentMillis;
  }

  //check for encoder button presses (for click-based menu items) and button holds
  if(encoderButtonState && currentMillis > lastEncButtonMillis + BUTTON_DELAY) {
    if( encoderButtonWasPushed == 0 ) { encoderButtonWasPushed = 1; } //if fresh push, toggle wasPushed detection to 1 state
    if(currentMillis > millisEncButtonWasLastOff + ENCODER_HOLD_DELAY) {
      validEncoderButtonHold = true;
      millisEncButtonWasLastOff = currentMillis;
      encoderButtonWasPushed = 2; //toggle hold pushbutton state (prevents also registering a standard click when button is released)
      lastEncButtonMillis = currentMillis;
    }
  } else if(!encoderButtonState && encoderButtonWasPushed != 0) {
    if(encoderButtonWasPushed == 1) { validEncoderButtonPress = true; }
    encoderButtonWasPushed = 0;
    lastEncButtonMillis = currentMillis;
  }
  
  //BOOK KEEPING
  
  //reset variables if the page has changed
  if(previousPage != currentPage) {
    saveLoadConfirmation = false; //reset confirmation status for saving and loading
    millisEncButtonWasLastOff = currentMillis; //reset counter for holding down the encoder
    flashLEDctrl = 0; //prevents getting hung if page changed while counting down
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

  //used by some pages to display consistent track colors
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
  
  //PAGE INPUTS/DISPLAYS
  
  if (currentPage == SEQUENCER_PAGE) {
    
    //input controls
     
    //if the encoder was spun and the encoder's button delay has passed....
    if(!buttonState[SEQUENCER_BUTTON]){ //if the track button (0) is not being held down...
      if(validEncoderSpinChange && !encoderButtonState) { //if encoder isn't being held down
        trackContainer[currentTrack][NOTE_QTY] += encoderState; //change the NOTE_QTY for the currentTrack
        //cleanup for NOTE_QTY
        trackContainer[currentTrack][NOTE_QTY] = cleanupByteValue(trackContainer[currentTrack][NOTE_QTY],0,trackLength,true);
      } else if (validEncoderSpinChange && encoderButtonState) {
        trackContainer[currentTrack][START_NOTE] += encoderState; //... then add or subtrack encoderState value from the currentTrack's START_NOTE
        //cleanup for START_NOTE values
        trackContainer[currentTrack][START_NOTE] = cleanupByteValue(trackContainer[currentTrack][START_NOTE],0,trackLength - 1,true);
      }
    } else {
      if(validEncoderSpinChange) {
        currentTrack += encoderState; //... then change the currentTrack by the encoderState value
        //cleanup for currentTrack values
        currentTrack = cleanupByteValue(currentTrack, 0, TRACK_QTY - 1 , true);
      } 
    }
    
    //if brightness or beatflash has changed, save it to EEPROM now 
    //Putting on sequencer page prevents saving every single time the value is changed, 
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
    
    //Turns on LEDs for notes on the current track
    for(byte beatCheck = 0; beatCheck < trackLength; beatCheck++){ //checks currentTrack for illuminating LEDs
      if(noteCheck(currentTrack,beatCheck)){ // if this beat contains a note TRUE...
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
        flashLEDctrl = 10;
      } else {
        flashLEDctrl = 20;
      }
    }
    if (flashLEDctrl > 0) {
      allowBeatFlash = false; //disable beatFlash while flashLEDctrl'ing
      if(flashLEDctrl < 11 || flashLEDctrl > 13) {
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
        //don't fill LED container with anything (simulates a blink if flashLEDctrl is higher than 10 but lower than 14)
      }
    }
    ledPreviousTrack = currentTrack;
    
  } else if ( currentPage == PITCH_PAGE) {
  
    //input controls
  
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
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
    }
    
    //led display
    
    byte tensColor[3] = {0,0,1};
    byte selectionColor[3] = {0,1,0};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][PITCH]);
  
  } else if ( currentPage == INSTRUMENT_PAGE) {

    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      if (fastSpin) { 
        trackContainer[currentTrack][INSTRUMENT] += (encoderState * 5);
      } else {
        trackContainer[currentTrack][INSTRUMENT] += encoderState;
      }
      //cleanup INSTRUMENT value
      trackContainer[currentTrack][INSTRUMENT] = cleanupByteValue(trackContainer[currentTrack][INSTRUMENT],0,127,true);
    }
    
    //led display
    
    byte tensColor[3] = {0,1,0};
    byte selectionColor[3] = {1,0,1};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][INSTRUMENT]);  
    
  } else if ( currentPage == TEMPO_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(tempoInternal) { // only change tempo if using tempoInternal
      if(validEncoderSpinChange) {
        
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
    if(validEncoderSpinChange) {
      if(fastSpin) {
        saveLoadSlot += encoderState * 16;
      } else {
        saveLoadSlot += encoderState;
      }
      saveLoadSlot = cleanupByteValue(saveLoadSlot,0,255,true);
      saveLoadConfirmation = false;
    }
    if(validEncoderButtonPress) {
      if(!saveLoadConfirmation) {
        saveLoadConfirmation = true;
      } else {
        if(currentPage == SAVE_PAGE) {
          saveTrack(DISK1_ADDR, "save");
          currentPage = 1;
          flashLEDctrl = 10;
        } else if (currentPage == LOAD_PAGE) {
          saveTrack(DISK1_ADDR, "load");
          currentPage = 1;
          currentTrack = 0;
          flashLEDctrl = 10;
        } else if (currentPage == ERASE_PAGE) {
          saveTrack(DISK1_ADDR, "erase");
        }
        saveLoadConfirmation = false;
      }
    }
    
    //led display
    
    //RGB for page save, load, erase display
    byte pageCountColor[3] = {0,0,1};  //pages will count upward as blue
    byte saveLoadColors[3] = {1,1,1};  //empty save slots will display as white
    int eepromAddressHelper = saveLoadSlot * 128; //preparing to check is a save slot is occupiued
    
    
    // these eeprom pulls are bogging down the usbmidi somehow
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
    if(validEncoderSpinChange) {
      if(fastSpin) { 
        trackContainer[currentTrack][VELOCITY] += encoderState * 5;
      } else {
        trackContainer[currentTrack][VELOCITY] += encoderState;
      }
      
      //cleanup for VELOCITY values
      trackContainer[currentTrack][VELOCITY] = cleanupByteValue(trackContainer[currentTrack][VELOCITY],0,127,false);
    }  
    
    //led controls
    
    byte tensColor[3] = {1,0,1};
    byte selectionColor[3] = {0,1,0};
    countByTensLED(tensColor, selectionColor, trackContainer[currentTrack][VELOCITY]);
    
  } else if ( currentPage == CHANNEL_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      if(lastNoteMillis[currentTrack] != 0){ //if the track has started playing a note already, stop it
        MIDI.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        lastNoteMillis[currentTrack] = 0;
      }
      trackContainer[currentTrack][CHANNEL] += encoderState; //... then change the Channel of the current track with encoderState
      //cleanup for CHANNEL values 
      trackContainer[currentTrack][CHANNEL] = cleanupByteValue(trackContainer[currentTrack][CHANNEL],1,16,true);
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
    if(validEncoderSpinChange) {
      trackLength += encoderState; //add or subtrack the encoderState value from the amount of beats in the trackLength
      //cleanup for trackLength values
      trackLength = cleanupByteValue(trackLength,1,16,false);
      //recalculate all track START_NOTEs and NOTE_QTYs if length changes
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        trackContainer[trackCycler][NOTE_QTY] = cleanupByteValue(trackContainer[trackCycler][NOTE_QTY],0,trackLength,false);
        trackContainer[trackCycler][START_NOTE] = cleanupByteValue(trackContainer[trackCycler][START_NOTE],0,trackLength - 1,false); 
      }
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
    if(validEncoderSpinChange) {
      trackContainer[currentTrack][NOTE_SPREAD] += encoderState; //... then change the Note Spread of the current track with encoderState
      //cleanup for Note Spread values 
      trackContainer[currentTrack][NOTE_SPREAD] = cleanupByteValue(trackContainer[currentTrack][NOTE_SPREAD],1,8,false);
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
    if(validEncoderSpinChange) {
      ledBrightness += encoderState; //... then change the display's brightness with ledBrightness
      //cleanup for ledBrightness values 
      ledBrightness = cleanupByteValue(ledBrightness,1,16,false);
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
    if(validEncoderSpinChange) {
      trackContainer[currentTrack][NOTE_LENGTH] += encoderState;
      trackContainer[currentTrack][NOTE_LENGTH] = cleanupByteValue(trackContainer[currentTrack][NOTE_LENGTH],1,16,false);
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
    if(validEncoderSpinChange) {
      beatFlash += encoderState; //... then change the intensity of beatFlash with encoderState
      //cleanup for beatFlash values 
      beatFlash = cleanupByteValue(beatFlash,0,16,false);
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
    
  } else if ( currentPage == MIDI_SUPPRESS_PAGE ) {

    //input controls

    if(validEncoderButtonPress) {
      if(midiSuppressByChannel) { //if in channel more
        if(bitRead(midiChannelSuppress,channelTrackSuppressSlot)) { //swap the selected channel's bit
          bitClear(midiChannelSuppress,channelTrackSuppressSlot);
        } else {
          bitSet(midiChannelSuppress,channelTrackSuppressSlot);
        }
        
      } else { //else if in track mode flip the bool
        trackContainer[channelTrackSuppressSlot][MIDI_TRACK_SUPPRESS] = !trackContainer[channelTrackSuppressSlot][MIDI_TRACK_SUPPRESS];
      }
    }
    
    if(validEncoderSpinChange) {
      channelTrackSuppressSlot += encoderState; 
    }
    
    if(validEncoderButtonHold) {
      midiSuppressByChannel = !midiSuppressByChannel;
    }

    //cleanup selection bytes to make sure they fall inside respective ranges
    if(midiSuppressByChannel) {
      channelTrackSuppressSlot = cleanupByteValue(channelTrackSuppressSlot,0,15,true);
    } else {
      channelTrackSuppressSlot = cleanupByteValue(channelTrackSuppressSlot,0,11,true);
    }
    
    //led display
    allowBeatFlash = false;
    byte selectionColor[3] = {1,1,1};
    
    if (midiSuppressByChannel) { //color in LEDs for active (non-suppressed) channels
      byte channelColors[3] = {0,0,1};
      for (byte channelDisplay = 0; channelDisplay < 16; channelDisplay++) {
        if(!bitRead(midiChannelSuppress,channelDisplay)) {
          for (byte colorCheck = 0; colorCheck < 3; colorCheck++){
            if (channelColors[colorCheck]) {
              bitSet(ledContainer[colorCheck], 15 - channelDisplay);
            }
          }
        }
      }
      
    } else { //color in LEDs for active (non-suppressed) tracks
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        if(!trackContainer[trackCycler][MIDI_TRACK_SUPPRESS]) {
          for(byte colorCycler = 0; colorCycler < 3; colorCycler ++) {
            if(trackColors[trackCycler][colorCycler]) {
              bitSet(ledContainer[colorCycler], 15 - trackCycler);
            }
          }
        }
      }  
    }

    //Make slot selected blink white or not
    if(flashLEDctrl == 0) {flashLEDctrl = 20;} //reset counter if run down
    if(flashLEDctrl < 13) { //if outside this threshold...
      for (byte colorCycler = 0; colorCycler < 3; colorCycler++){ // ...white out the selection choice
        bitSet(ledContainer[colorCycler], 15 - channelTrackSuppressSlot);
      }
    }
    

  }  //END OF PAGES 
  
  
  //PLAY/PAUSE BUTTON INPUT
  if(buttonState[PLAY_BUTTON] && currentMillis > lastPlayButtonMillis + (BUTTON_DELAY * 3) && currentMillis - millisPlayButtonWasLastOff < (BUTTON_DELAY * 3)) {
    playing = !playing;
    lastBeatMillis = currentMillis;
    lastPlayButtonMillis = currentMillis;
  }
  //PLAY BUTTON AS STOP BUTTON INPUT
  //Holding play button stops playback and resets beat to 0
  if(buttonState[PLAY_BUTTON] && currentMillis > millisPlayButtonWasLastOff + 1500) {
    playing = false;
    currentBeat = 0;
    notesOff(); 
    lastPlayButtonMillis = currentMillis;
  }

  
  //check if the currentPage needs reset to page 0 due to inactivity
  if(currentPage != SEQUENCER_PAGE){
    if(currentMillis > millisForPages + PAGE_0_TIMEOUT) {
      currentPage = 1;
    }
  }
  
  //flashing LEDs will break if pageControl() loop cycles too quickly because the timer will run down too fast
  //flashing leds designed with this overall loop running every 20ms in mind
  if ( flashLEDctrl > 0 ) { flashLEDctrl -= 1; }
  
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
