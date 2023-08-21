/******************************************************************************

Sundial Midi Sequencer v3.1.1

*changing over to SAMD21 for v3*

ToDo:

  Features:
   
    global settings:
      vs1053b volume
      led brightness/beatflash
      
    
    change track colors manually with encoder - save to eeprom, one color scheme
    
    add in save-transfers between units?
    
  MIDI still implementing:
    fixed? - usb midi and crosstalk with DIN
    re-implement start/stop/continue command handling
    receive song position over midi (any other?)
 
  Auditing:
 
    check speed of program
    
    cleanup notesOff stuff to include actual noteoff midi commands

  Bugs:

    play/pause/stop sends out infinite note off loop
 
    tempo moving backwards locks tempo bumps to 3890 then locks at 6000 + yellow in 0 spot
    fixed? - count by tens showing 16th led illuminated weird on a 10th digit
    fixed? - count by hunds showing 16th led illuminated weird on a 10th digit
  
  Notes:
  
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
USBMIDI_CREATE_INSTANCE(0,MIDIUSB)
// Serial
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI)


//------------------//
//-- BOARD SELECT --//
//------------------//

//Select which board is being programmed
//#define V0_PROTOTYPE_BOARD
//#define V1_PCB_BOARD //ATmega32U4 V1 pinouts
//#define V2_PCB_BOARD //ATmega32U4 V2 pinouts
#define V3_PCB_BOARD  //SAMD21 Pinouts

//LED display type
//#define 595_led_rrrgggbbb //595 based chip with dedicated-color pinout for each chip
//#define 595_led_rgbrgbrgb //595 based chip with the colors each chip
#define WS2812_led //WS2812 based Din Dout leds

#ifdef V3_PCB_BOARD

  // -- PIN DEFINES -- //

  //Encoder Pins
  #define ENCODER_PIN1 12 //MSB pin of encoder
  #define ENCODER_PIN2 4 //LSB pin of encoder
  #define ENCODER_BUTTON_PIN 6 //encoder button pin

  //Multiplexer Pins
  #define BUTTON_PIN A2 //button multiplexer output pin
  #define BUTTON_MUX_PIN_1 A3
  #define BUTTON_MUX_PIN_2 A4
  #define BUTTON_MUX_PIN_3 A5

  //define hat-triggered standby pin
  #define BLOCK_PIN 9

  //I2C EEPROM
  //#define SCL_PIN 3
  //#define SDA_PIN 2

  //MIDI Related Pins
  //#define MIDI_SUPPRESS_PIN 2 //controls transistor (base) link between MIDI out and VS1053B in

#endif



//#ifdef 595_led_rgbrgbrgb

  //LED Shift Register Pins
  //#define BRIGHTNESS_PIN A1
  //#define CLOCK_PIN A4
  //#define LATCH_PIN A3
  //#define DATA_PIN A2
  
//#endif


#ifdef WS2812_led

  #include <Adafruit_NeoPixel.h>
  
  #define PIXEL_PIN 8 //send to Din of first pixel
  #define NUMPIXELS 16 // number of LEDs used (code is not designed for alterations to this)
  byte pixelBrightness = 50;
  
  //notes from example strandtest.ino:
  // Argument 1 = Number of pixels in NeoPixel strip
  // Argument 2 = Arduino pin number (most are valid)
  // Argument 3 = Pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  Adafruit_NeoPixel pixels(NUMPIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);
  
  
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
//two more track variable bytes allowed and currently unused for each instument track (12 instrument tracks per save)
//#define unused_variable 8
//#define unused_variable 9


//-- GLOBAL VARIABLES --//
int currentPage = 1; //controls what is displayed on the LED ring and what button inputs do
int previousPage = currentPage; //page change detection for book keeping
int currentTrack = 0; //0-11 for 12 separate tracks
bool playing = true; //is the device playing or paused/stopped?
byte currentBeat = 0; //what beat the track is currently at (stopped = 0, music starts on 1, and rolls over from 16 (or trackLength) to 1)
byte saveLoadSlot = 0; //remembers selected save/load slot
bool saveLoadConfirmation = false; //bool to require second click of encoder to save, load, or erase a track

//Button and Detection Variables
bool buttonState[8] = {0,0,0,0,0,0,0,0}; //container to know if buttons are currently pushed or not
bool encoderButtonState = 0; //container to know if encoder button is being pushed
byte encoderButtonWasPushed = 0; //container to know if encoder button was pushed last loop, used to detect between button push or button hold (0: wasn't pushed, 1: was pushed, 2:hold click happened, don't also push )
int encoderState = 0; //-1 is encoder moved down, 1 is encoder moved up, 0 is no change
//unsigned long lastButtonMillis[10] = {0,0,0,0,0,0,0,0,0,0}; //timers to prevent unintentional button represses (8 buttons, encoder button, encoder knobs)
unsigned long lastPlayButtonMillis = 0; //prevents play button over-detection
unsigned long lastEncButtonMillis = 0;  //prevents encoder button click over-detection
unsigned long lastEncSpinMillis = 0;    //prevents encoder spin over-detection
unsigned long millisForPages = 0; //timers to decide when to change back to page 0 automatically
unsigned long millisPlayButtonWasLastOff = 0; //used to detect if play button has been continually held down by tracking the last time it wasn't pushed
unsigned long millisEncButtonWasLastOff = 0;  //used to detect if encoder button has been continually held down by tracking the last time it wasn't pushed
#define PAGE_0_TIMEOUT 4000 //(page_ZERO_timeout) ms without any input before display goes back to sequencer page automatically
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
//unsigned long currentMicros = 0;  //for global timing features (microseconds)
//unsigned long timer1ms = 0; //micro() container for functions that will run every 1 millisecond (microseconds needed for accuracy at the 1 milliseond level)
//unsigned long timer10ms = 0; //milli() container for functions that will run every 10 ms
unsigned long timer20ms = 0; //milli() container for functions that will run every 20 ms
//bool canRun1ms = true; //functions watch these ints to know if they are allowed to run
//bool canRun10ms = true;
bool canRun20ms = true;


//array of pins to run multiplexer commands
const int muxPins1[] = {
  BUTTON_MUX_PIN_1,BUTTON_MUX_PIN_2,BUTTON_MUX_PIN_3
};


//-- EEPROM VARIABLES --// variables that are saved to EEPROM when saving tracks
byte trackLength = 16; // 1-16 beats per song
short tempo = 1200; //BPM * 10 to be able to use int instead of float, and still increment by .5 BPM. updateTempoValues resolves the math by * 0.1
//tracks 1-12, values are {channel, pitch, start note (1st beat is 0), qty of notes, instrument #, note length (currently beat qty), velocity, note spread, unused, unused}
byte trackContainer[TRACK_QTY][TRACK_VARIABLE_QTY] = { {1,60,0,1,0,1,63,8,0,0}, {2,60,0,0,0,1,63,8,0,0}, {3,60,0,0,0,1,63,8,0,0}, {4,60,0,0,0,1,63,8,0,0}, {5,60,0,0,0,1,63,8,0,0}, {6,60,0,0,0,1,63,8,0,0}, {7,60,0,0,0,1,63,8,0,0}, {8,60,0,0,0,1,63,8,0,0}, {9,60,0,0,0,1,63,8,0,0}, {11,60,0,0,0,1,63,8,0,0}, {12,60,0,0,0,1,63,8,0,0}, {13,60,0,0,0,1,63,8,0,0} };
#define VALID_SAVE_BYTE 99 //validation number to check for in save slot checks and to save to SAVE_BYTE_ADDRESS to mark that a valid save exists in a given slot, used to invalidate old saves after major changes happen to structure

//Tempo Variables
short lastTempo = 0; //to detect changes in tempo to prevent unneccesary math
#define MINIMUM_TEMPO 300 //lowest tempo can be manually set
#define MAXIMUM_TEMPO 6000 //highest tempo can be manually set
//bool tempoInternal = true; //1:internal source, 0:external source for tempo control 
int msBetweenBeats = 500; //used to fill with tempo-based math for beat timers
long unsigned lastBeatMillis = 0; //when last beat happened to detect if it's time for a new beat to happen
bool beatJustChanged = false;  //triggers once-per-beat actions like playing notes


//LED variables
//int ledPreviousTrack = 0; //for flashing leds on page 0 when a track changes - moved into led control loop
byte ledByteContainer[3][16] = { //stores 0-255 RGB values for LED's (thousands of colors)
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //red
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, //green
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  //blue
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
//bool midiSuppressByChannel = true; //TRUE sets midi suppression to VS1053B to be channel based, false is by track, pulled from EEPROM
//uint16_t midiChannelSuppress = 0b0000000000000000; //stores bits to see if a given midi channel needs suppressed
//byte channelTrackSuppressSlot = 0;


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
#define DISK1_ADDR 0x50  //24LC256 and AT24CM01 I2C address with all address pins wired LOW to GND
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
#define NOTESPREAD_PAGE 136

//two-button (option button) pages
#define BRIGHTNESS_PAGE 17
//#define VOLUME_PAGE 18 //not yet implemented
#define NOTE_LENGTH_PAGE 20
#define TRACK_LENGTH_PAGE 24
#define CAPTURE_SAVES_PAGE 48
#define LOAD_CAPTURES_PAGE 80

//other two-button pages
#define ERASE_PAGE 96  //save+load buttons

//three-button pages
#define ERASE_ALL_PAGE 112  //not yet implemented

//two button encoder-shifted pages
#define BEATFLASH_PAGE 145


//#define MIDI_SUPPRESS_PAGE 17    //permanently removed?

#define PAGE_QTY 17 //needs to be equal to total number of pages
//each page needs added to this array (location unimportant)
const byte validPages[PAGE_QTY] = {SEQUENCER_PAGE, PITCH_PAGE, TEMPO_PAGE, INSTRUMENT_PAGE, SAVE_PAGE, LOAD_PAGE, ERASE_PAGE, NOTESPREAD_PAGE, BRIGHTNESS_PAGE, NOTE_LENGTH_PAGE, VELOCITY_PAGE, TRACK_LENGTH_PAGE, CHANNEL_PAGE, BEATFLASH_PAGE, CAPTURE_SAVES_PAGE, LOAD_CAPTURES_PAGE, ERASE_ALL_PAGE};



//----------------------------------//
//--      Daughterboards          --//
//--       Addons and Hats        --//
//----------------------------------//

#define COPYCAT_ADDRESS 0x9
bool copyCatConnected = false;




//debugging stuff
bool standby = true;







//-- SETUP --
void setup() {

  //for debugging
  //Serial.begin(9200);

  //Begin I2C 
  Wire.begin(0x8);
  Wire.setClock(400000);
  //disable internal pullups
  //digitalWrite(SDA,LOW);
  //digitalWrite(SCL,LOW);

  //enable input pins and turn pullup resistors on
  pinMode(ENCODER_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_PIN2, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  //digitalWrite(ENCODER_PIN1, HIGH);
  //digitalWrite(ENCODER_PIN2, HIGH);
  //digitalWrite(ENCODER_BUTTON_PIN, HIGH);
  for (byte i = 0; i < 3; i++) {
    pinMode(muxPins1[i], OUTPUT);
    digitalWrite(muxPins1[i], HIGH);
  }
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  //digitalWrite(BUTTON_PIN, HIGH);

  //initialize blocking pin to go into standby when CopyCat is in use
  pinMode(BLOCK_PIN, INPUT_PULLUP);
  
//  #ifdef 595_led_rgbrgbrgb
    //Enable LED shift register pins
    //pinMode(DATA_PIN, OUTPUT);
    //pinMode(CLOCK_PIN, OUTPUT); 
    //pinMode(LATCH_PIN, OUTPUT);
    //pinMode(BRIGHTNESS_PIN, OUTPUT);
    //analogWrite(BRIGHTNESS_PIN, 100);
//  #endif
  
  #ifdef WS2812_led
    pixels.begin(); // INITIALIZE NeoPixel object
    pixels.clear();
    pixels.show();
    pixels.setBrightness(pixelBrightness);
  #endif
  
  //MIDI Implementation
  //MIDI.setHandleClock(handleClock);
  //MIDI.setHandleStart(handleStart);
  //MIDI.setHandleStop(handleStop);
  //MIDI.setHandleContinue(handleContinue);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDIUSB.begin(MIDI_CHANNEL_OMNI);

  //to suppress midi signals sent to the VS1053B
  //pinMode(MIDI_SUPPRESS_PIN, OUTPUT);
  //digitalWrite(MIDI_SUPPRESS_PIN, HIGH);

  //clear LEDs and prepare them for next write
  //writes full clear bits out for each of the 6 595 chips
  //digitalWrite(LATCH_PIN, LOW);
  //for(int i = 0; i < 6; i++) {
  //  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 0b00000000);
  //}
  //digitalWrite(LATCH_PIN, HIGH); //cycle the shift registers to display
  //digitalWrite(LATCH_PIN, LOW); //cycle the shift registers to be ready to receive bytes for next display
  
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
  
  //rainbow delay to allow and daughter boards to startup
  //rainbow(10,1);
  
  
  //check for daughterboards
  //Wire.beginTransmission(COPYCAT_ADDRESS);
  //int rdata = Wire.endTransmission();
  //if (rdata == 0) { copyCatConnected = true; };
  //copyCatConnected = true;
  
}
//-- END OF SETUP --


//-- LOOP --
void loop() {

  //enable standby mode if BLOCK_PIN has been brought LOW 
  while (!digitalRead(BLOCK_PIN)) {
    rainbow(10,1);
    delay(10);
  }
  
  timerHandler();
  
  readMIDI();
  
  updateEncoder();

//  if(canRun1ms) { 
    
    
//  }
  
  
  while (standby) {
    rainbow(10,1);
    standby = false;
    
    //check for daughterboards
    Wire.beginTransmission(COPYCAT_ADDRESS);
    int rdata = Wire.endTransmission();
    if (rdata == 0) { copyCatConnected = true; };
  }
  
  
 if(canRun20ms) {

    updateButtons();
  
    updateTempoValues();
    
    beatCheck();
    
    handleNotes();
    
    pageControl();
    
  } 


  //MidiUSB.flush();  //attempt to fix midi usb send bug


}
//-- END OF LOOP --





//-----------------------------------------//
//--            Memory Control           --//
//-- Internal and External EEPROM Saving --//
//-----------------------------------------//


void saveTrack(byte diskAddress, String saveType) {

  int slotOffset =  (saveLoadSlot * 128); //128 bytes allocated for each track save  
  
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
      byte emptyTrackContainer[TRACK_QTY][TRACK_VARIABLE_QTY] =  { {1,60,0,1,0,1,63,8,0,0}, {2,60,0,0,0,1,63,8,0,0}, {3,60,0,0,0,1,63,8,0,0}, {4,60,0,0,0,1,63,8,0,0}, {5,60,0,0,0,1,63,8,0,0}, {6,60,0,0,0,1,63,8,0,0}, {7,60,0,0,0,1,63,8,0,0}, {8,60,0,0,0,1,63,8,0,0}, {9,60,0,0,0,1,63,8,0,0}, {11,60,0,0,0,1,63,8,0,0}, {12,60,0,0,0,1,63,8,0,0}, {13,60,0,0,0,1,63,8,0,0} };
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
    Wire.endTransmission();
    delay(5);
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
      //checkMIDIsuppression("track",trackCycler);
      if(currentBeat > 0) { //prevent moving outside array bounds with [currentBeat - 1] (might be a non-issue)
        if(noteCheck(trackCycler, currentBeat - 1)) { //if a track is set to play a note this beat (current beat is 1-16 if playing music, so a -1 is used for the math)
          MIDI.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]); //
          MIDI.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
          
          MIDIUSB.sendProgramChange(trackContainer[trackCycler][INSTRUMENT],trackContainer[trackCycler][CHANNEL]);
          MIDIUSB.sendNoteOn(trackContainer[trackCycler][PITCH],trackContainer[trackCycler][VELOCITY],trackContainer[trackCycler][CHANNEL]);
          lastNoteMillis[trackCycler] = currentMillis; //fill this track's lastNoteMillis[] slot with this note's timing
        }
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

/*
void checkMIDIsuppression (String mode, byte trackOrChannelToCheck) {
  if(midiSuppressByChannel) { //if channel mode is enabled
    if(mode == "track") { //for checking if a track's channel should be allowed to play or not
      if(bitRead(midiChannelSuppress,trackContainer[trackOrChannelToCheck][CHANNEL] - 1)) { //check the midiChannelSuppress bits for the current track's channel bit
        digitalWrite(MIDI_SUPPRESS_PIN, LOW); //if channel should be suppressed, set transistor to low
      } else { 
        digitalWrite(MIDI_SUPPRESS_PIN, HIGH);
      }
    } else if (mode == "channel") { //for checking if a specific channel should be played or not
      if(bitRead(midiChannelSuppress,trackOrChannelToCheck)) { //check the midiChannelSuppress bits for the current track's channel bit
        digitalWrite(MIDI_SUPPRESS_PIN, LOW); //if channel should be suppressed, set transistor to low
      } else {
        digitalWrite(MIDI_SUPPRESS_PIN, HIGH);
      }
    }
  } else if (mode == "track") { //for checking if a track should be allowed to play or not
    if(trackContainer[trackOrChannelToCheck][MIDI_TRACK_SUPPRESS]) { //if track mode is enabled
      digitalWrite(MIDI_SUPPRESS_PIN, LOW); //if a track should be suppressed, set transistor to low
    } else {
      digitalWrite(MIDI_SUPPRESS_PIN, HIGH);
    }
  } 
}
*/



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
    currentBeat = changeByteValue(currentBeat,1,1,trackLength,true,1); //advance the beat by one
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



//checks if a given beat in a given track has an active note or not 
//and returns TRUE or FALSE
//(0,0) is the first beat of the first track
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


//consolodates program delay timing to one function, rather than a timer for each program funciton
void timerHandler() {
  
  currentMillis = millis();
  //currentMicros = micros();
  //canRun1ms = 0;
  //commenting out 10ms timer, currently nothing uses it
  //canRun10ms = 0; 
  canRun20ms = 0;    

  //if(currentMicros >= timer1ms) {  //timer resolves using microseconds, not milliseconds
  //  timer1ms = currentMicros + 1000;
  //  canRun1ms = 1;
  //}
  //if(currentMillis >= timer10ms) {
  //  timer10ms = currentMillis + 10;
  //  canRun10ms = 1;
  //}
  if(currentMillis >= timer20ms) {
    timer20ms = currentMillis + 20;
    canRun20ms = 1;
  }
}





//cleanupByteValue
//replacement rollover method. instead of checking after
//the value changes, feed it the intended change and the min max values and let
//it make the change itself. Should save program space and be able to turn bytes
//into ints to check for negative ranges easily before doing the math.
//This should lower program size and prevent issues with the "highestVal + 15"
//nonsense in cleanupByteValue()
byte changeByteValue(byte byteToChange, int changeByAmount, byte minVal, byte maxVal, bool rollover, byte fastSpinMultiplier) {
  int intCheck = byteToChange; //change the byte to an int to have negative numbers available
  
  if (fastSpin) {
    changeByAmount = changeByAmount * fastSpinMultiplier;
  }
  
  intCheck += changeByAmount;
  
  //detect if new value will be in range or out of it, and return appropriately
  if ( intCheck < minVal ) {
    if (rollover) {
      return maxVal;
    } else {
      return minVal;
    }
  } else if ( intCheck > maxVal ) {
      if (rollover) {
        return minVal;
      } else {
        return maxVal;
      }
  } else {
    byte rdata = intCheck;
    return rdata;
  }
}


//phase the use of this out, I think changeByteValue can handle everything
//byte related, some remaining cleanupByteValue checks can be done with a 0 change amount

//makes sure a given value falls between a min/max range
//takes given value, checks against min and max values to ensure it falls between
//if it's outside the range, rollover bool controls if it stays at max/mon value
//or rolls over to other side of range
byte cleanupByteValue(byte value, byte lowestVal, byte highestVal, bool rollover) {
  //int intHighestValue = highestVal;
  //int intLowestVal = lowestVal;
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


//rainbow colors
//function to have a something to show while device is waiting on CopyCat 
void rainbow(int wait, int loops) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  //changed to 2 passes
  for(long firstPixelHue = 0; firstPixelHue < loops*65536; firstPixelHue += 256) {
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / NUMPIXELS);
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
    }
    pixels.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
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
//needs to happen frequently or it misses inputs (needs to be faster than 20ms loops)
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
  {1,1,1}  //button7   stop
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
  
  int pageCounter = 0;
  //buttons 1-8 and encoder button (button 5 is play/stop, not used for page changing and set to value of 0)
  const byte buttonPageValues[9] = {1,2,4,8,16,0,32,64,128};
  
  if(currentMillis > millisForPages + 150) {  //delay followup page changes a little bit (150ms), this allows letting go of buttons without changing the page
    //add button values to determine page number
    for(byte buttonCycler = 0; buttonCycler < 8; buttonCycler++) { //button cycler is the quantity of button page values - 1 (for encoder button)
      if (buttonState[buttonCycler]) {
        pageCounter += buttonPageValues[buttonCycler];
      }
    }
    if (encoderButtonState) {
      //millisForPages = currentMillis;
      pageCounter += buttonPageValues[8]; //encoder value
    } 
    
    //check to make sure a valid page is landed on 
    //(prevents page switching to invalid value if random buttons are pushed)
    //this also has a subtle effect of freeing up the encoder button to be used 
    //as a control input as long as a page button isn't being held down as well
    for (byte pageChecker = 0; pageChecker < PAGE_QTY; pageChecker++) {
      if (pageCounter == validPages[pageChecker]) {
        currentPage = pageCounter;
        if (currentPage == CAPTURE_SAVES_PAGE || currentPage == LOAD_CAPTURES_PAGE) {
          if (!copyCatConnected) { currentPage -= buttonPageValues[OPTIONS_BUTTON]; } //if no copycat is connected, don't load copycat pages
        }
        millisForPages = currentMillis;
      }
    }
    
  }

}



//---------------------------------------------//
//--  PAGE DISPLAY / CONTROL INTERPRETATION  --//
//---------------------------------------------//


//this function pushes a variable to the leds, so you can display a number
//by changing the page#Count values, you can break up large numbers into multiple digits for display
//There are two main ways of using this:
//using it in the standard mode, the final selection will be a single LED, and pages will fill for the other digits (selectionVar of 24 with a page1Count of ten will fill the first two LEDs with page1Color and only the fourth led will be filled with selectionColor)
//but instead setting page1Count to 1 will fill the leds up to the selectionVar (selectionVar of 7 with a page1Count of 1 will fill the first 7 LEDs with page1Color)

//if a selectionVar value of 0 should be displayed on the first LED, pass it to the function as "selectionVar + 1"
//overriddePageColor: true: a selectionColor value of 0 will push to the display instead of just being ignored, overriding any page1 or page2 colors under it
//pageModify: non-zero value adds to page1 display so that counting starts at different LED (like for save slots display always showing a page by adding 1)
//example: to count by 100's (diplaying 3 separate digits) set page1Count to 10 and page2Count to 100
//setting a page#Count to zero will prevent a page from being made
void displaySelectionByPage(int selectionVar, byte selectionColor[3], int page1Count, byte page1Color[3], int page2Count, byte page2Color[3], bool overridePageColor, int pageModify) {
 
  byte selection = 0;
  byte page1 = 0;
  byte page2 = 0;
  int remainingVar = selectionVar;
  
  if (page2Count > 0){
    page2 = floor(selectionVar / page2Count);
    remainingVar = remainingVar % page2Count;
  }

  if (page1Count > 0){
    page1 = floor(remainingVar / page1Count);
    remainingVar = remainingVar % page1Count;
    if (pageModify != 0) { page1 += pageModify; }
  }

  if (remainingVar > 0) {
    selection = remainingVar;
  }


  for(byte countPage2 = 0; countPage2 < page2; countPage2++) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(page2Color[colorCheck] > 0) {
        ledByteContainer[colorCheck][countPage2] = page2Color[colorCheck]; 
      }
    }
  }
  for(byte countPage1 = 0; countPage1 < page1; countPage1++) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(page1Color[colorCheck] > 0) {
        ledByteContainer[colorCheck][countPage1] = page1Color[colorCheck];
      }
    }
  }
  if (selection > 0 && selection <= 16) {
    for(byte colorCheck = 0; colorCheck < 3; colorCheck++) { //...and the RGB color set
      if(selectionColor[colorCheck] > 0) {
        ledByteContainer[colorCheck][selection - 1] = selectionColor[colorCheck];
      } else if (overridePageColor) { //if overridePageColor is TRUE then completely override the color on selection's location
        ledByteContainer[colorCheck][selection - 1] = selectionColor[colorCheck];
      }
    }
  }
  
  
}




//handles button input interpretation and displaying LEDs
void pageControl() {
  
  
  int ledPreviousTrack = currentTrack;
  
  //used by pages to detect encoder spin input and cut down on redundant checks
  bool validEncoderSpinChange = false; //reference for if the encoder registered a changed in UP/DOWN spin state (also considers timer delay/debounce)
  bool validEncoderButtonPress = false; //reference for if the encoder registered an activation in button state (also considers timer delay/debounce)
  bool validEncoderButtonHold = false; //reference for if the encoder registered a held-down activation in button state (also considers timer delay/debounce)

  

  if (encoderState != 0) { //if the encoder was spun
    
    saveLoadConfirmation = false;    //reset save/load confirm if encoder position changed
    millisForPages = currentMillis;  //reset timer clock for changing back to sequencer page if encoder changed

    
    
    //FastSpin Encoder Detection (fastSpin Code Section 1 of 2)
    //fastSpin checks to see if someone is turning the encoder quickly (surprise!).
    //Some pages with lots of values use this to increase multiple values
    //or even pages at a time if the encoder is being spun very quickly.
    //It works by loading an array with a 1 each time the encoder changes
    //(will only register a maximum change of once per 20ms due to pageControl()
    //being a 20ms loop) and loads a 0 into the array each time the ENCODER_DELAY
    //passes. If the array has enough 1's in it (FASTSPIN_THRESHOLD), fastSpin is TRUE.
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
  
  
  int brightnessFactor = 16;//less brightness for non-prototype running 595 chips on the 5v rail instead of prototype's 3v 595 design
  //if (prototype) {brightnessFactor = 16;} //previously used to raise brightness for 3v multiplexer setup (5v in use now is birghter)
  
  
  //empty the LED byte display container
  for(int ledTrack = 0; ledTrack < 3; ledTrack++) {
    for(byte led = 0; led < 16; led++) {
      ledByteContainer[ledTrack][led] = 0;
    }
  }
  
  #ifdef WS2812_led
    pixels.clear();
  #endif
  
  //if ledBrightness setting changed, then output the new brightness over PWM
  if(changeBrightness) {
    
    //byte newBrightness = 254 - ((ledBrightness -1) * brightnessFactor);
    //analogWrite(BRIGHTNESS_PIN, newBrightness);
    
    #ifdef WS2812_led
      byte newPixelBrightness = (ledBrightness * brightnessFactor);
      pixels.setBrightness(newPixelBrightness);
    #endif
    
    changeBrightness = false;
  } 
  
  //flashes brightness brighter on new beats, intensity controlled by beatFlash
  if(beatFlash > 0 && allowBeatFlash) {
    if(lastBeatMillis - currentMillis < 200) {
      byte newBrightness = 0;  //initially, set brightness to max
      
      #ifdef WS2812_led
        byte newPixelBrightness = 255;
      #endif
      
      if(ledBrightness > 9) {  //if brightness is over 9 (of 16) then blank out lights instead of flashing brighter
        newBrightness = 255;
        
        #ifdef WS2812_led
          newPixelBrightness = 0;
        #endif
      }
      if(ledBrightness + beatFlash < 17) { //if brightness plus beatflash is higher than 16 (max), leave it at max instead of trying to go higher
        newBrightness = 254 - ((ledBrightness + beatFlash - 1) * brightnessFactor); //if brightness + beatflash was lower than 16, then set exact brightness level

        #ifdef WS2812_led
          newPixelBrightness = ((ledBrightness + beatFlash) * brightnessFactor);
        #endif
        
      }
      //analogWrite(BRIGHTNESS_PIN, newBrightness);
      
      #ifdef WS2812_led
        pixels.setBrightness(newPixelBrightness);
      #endif
      
    } else {   //if it's been longer than 200ms since last beat, set brightness to normal setting
      byte newBrightness = 254 - ((ledBrightness - 1) * brightnessFactor); 
      //analogWrite(BRIGHTNESS_PIN, newBrightness);
      
      #ifdef WS2812_led
        byte newPixelBrightness = ((ledBrightness - 1) * brightnessFactor);
        pixels.setBrightness(newPixelBrightness);
      #endif
      
    }
  }
  allowBeatFlash = true; //bookkeeping to reset beatFlash to be turned on, forces diasabling processes to actively (continually) disable it

  //used by some pages to display consistent track colors
  const byte trackColors[TRACK_QTY][3] = { //RGB colors for each track
    {255, 125,  0}, //orange
    {102,255,102}, //lt green
    {255,  0,255}, //purple
    {  0,255,255}, //aqua
    {  0,255,  0}, //grn
    {255,  0,  0}, //red
    {255,  0,255}, //purple
    {  0,  0,255}, //blue
    {255,255,  0}, //yellow
    {  0,255,255}, //aqua
    {  0,255,  0}, //grn
    {255,  0,  0}  //red
  };
  //helper for placing ignorable color values into functions
  const byte zeroColor[3] = {0,0,0};
  
  //PAGE INPUTS/DISPLAYS
  
  if (currentPage == SEQUENCER_PAGE) {
    
    //input controls
     
    //if the encoder was spun and the encoder's button delay has passed....
    if(!buttonState[SEQUENCER_BUTTON]){ //if the track button (0) is not being held down...
      if(validEncoderSpinChange && !encoderButtonState) { //if encoder isn't being held down
        trackContainer[currentTrack][NOTE_QTY] = changeByteValue(trackContainer[currentTrack][NOTE_QTY],encoderState,0,trackLength,true,1);
      } else if (validEncoderSpinChange && encoderButtonState) {
        trackContainer[currentTrack][START_NOTE] = changeByteValue(trackContainer[currentTrack][START_NOTE],encoderState,0,trackLength - 1,true,1);
      }
    } else {
      if(validEncoderSpinChange) {
        currentTrack = changeByteValue(currentTrack, encoderState, 0, TRACK_QTY - 1, true, 1);
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
        for(byte colorCycle = 0; colorCycle < 3; colorCycle++) { //cycle thru Red, Green, Blue colors of the track
          ledByteContainer[colorCycle][beatCheck] = trackColors[currentTrack][colorCycle];
        }
      }
    }
    
    //make current beat's location LED white
    if(currentBeat > 0) {
      for(byte colorCycle = 0; colorCycle < 3; colorCycle++) {
        ledByteContainer[colorCycle][currentBeat - 1] = 255;
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
            ledByteContainer[colorCheck][beatCounter] = trackColors[currentTrack][colorCheck];
          }
        }
      } // else don't fill LED container with anything (simulates a blink if flashLEDctrl is higher than 10 but lower than 14)
    }
    
  } else if ( currentPage == PITCH_PAGE) {
  
    //input controls
  
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      if(lastNoteMillis[currentTrack] != 0){ //if the track has started playing a note already, stop it
        MIDI.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        MIDIUSB.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        lastNoteMillis[currentTrack] = 0;
      }
      trackContainer[currentTrack][PITCH] = changeByteValue(trackContainer[currentTrack][PITCH], encoderState, 0, 127, false, 5);
    }
    
    //led display
    
    beatFlash = false;
    
    //display the LED like piano notes in groups of 12 (an octave), first LED is a C note
    
    for (byte ledCycler = 0; ledCycler < 12; ledCycler++) {
      
      ledByteContainer[2][ledCycler] = 255; //all piano keys get blue
      
      if (ledCycler != 1 && ledCycler != 3 && ledCycler != 6 && ledCycler != 8 && ledCycler != 10 ) {
        //ledByteContainer[0][ledCycler] = 0; //non-sharp piano keys get some red
        ledByteContainer[1][ledCycler] = 160; //non-sharp piano keys get some green
      }
      
    }
    
    if (flashLEDctrl == 0) { flashLEDctrl = 20; }
    byte selectionColor[3] = {255,255,255};
    byte tensColor[3] = {90,0,0};
    //separate the LED pages (showing octave) and the selection (showing the note selected) to flash the note independantly of the octave
    //lower the octave display by 1 with the -1
    displaySelectionByPage(trackContainer[currentTrack][PITCH] + 1, zeroColor, 12, tensColor, 0, zeroColor, false, -1 );
    if (flashLEDctrl > 10) {
      displaySelectionByPage(trackContainer[currentTrack][PITCH] + 1, selectionColor, 12, zeroColor, 0, zeroColor, false, 0 );
    }
    //special pitch case to simulate -1 octave display
    if (trackContainer[currentTrack][PITCH] < 12) {
      ledByteContainer[0][15] = tensColor[0];
    }
  
  } else if ( currentPage == INSTRUMENT_PAGE) {

    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      trackContainer[currentTrack][INSTRUMENT] = changeByteValue(trackContainer[currentTrack][INSTRUMENT], encoderState, 0, 127, true, 5);
    }
    
    //led display
    
    byte selectionColor[3] = {255,0,255};
    byte tensColor[3] = {0,255,0};
    displaySelectionByPage(trackContainer[currentTrack][INSTRUMENT] + 1, selectionColor, 10, tensColor, 0, zeroColor, false, 0 );  
    
  } else if ( currentPage == TEMPO_PAGE ) {
    
    //input controls
    
    //if the encoder was spun...
  //if(tempoInternal) { // only change tempo if using tempoInternal
    if(validEncoderSpinChange) {
      
      if(fastSpin) {
        tempo += (encoderState * 200);
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
  //}
    
    //led display
    
    byte selectionColor[3] = {0,255,0};
    byte tensColor[3] = {0,0,255};
    byte hundredsColor[3] = {255,0,0};
    
    displaySelectionByPage((tempo * 0.1), selectionColor, 10, tensColor, 100, hundredsColor, false, 0 ); //tempo 8 0.1 to turn it into BPM, -1 to do LED function math propery (zero based counting system)
    
  } else if ( currentPage == SAVE_PAGE || currentPage == LOAD_PAGE || currentPage == ERASE_PAGE ) {
    
    //input controls
    
    allowBeatFlash = false;
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      saveLoadSlot = changeByteValue(saveLoadSlot, encoderState, 0, 255, true, 16);
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
    byte pageCountColor[3] = {0,0,255};  //pages will count upward as blue
    byte saveLoadColors[3] = {255,255,255};  //empty save slots will display as white
    int eepromAddressHelper = saveLoadSlot * 128; //preparing to check is a save slot is occupiued
    
    byte saveByteCheck = pullEepromByte(DISK1_ADDR, eepromAddressHelper, 1);
    if (saveByteCheck == VALID_SAVE_BYTE) { //if saveslot is occupied, change display color (saveLoadColors) to currentPage's (save, load, erase) color
      if (currentPage == SAVE_PAGE) {
        saveLoadColors[2] = 0;
      } else if (currentPage == ERASE_PAGE) {
        saveLoadColors[1] = 0;
        saveLoadColors[2] = 0;
      } else if (currentPage == LOAD_PAGE) {
        saveLoadColors[0] = 0;
        saveLoadColors[2] = 0;
      }
    }
    
    //display final color selections, override the page color with the selection's color. Most of the program's existing code
    //harmonizes counting display colors and allows overlap for clarity, but with the amount of options with save, load, erase, 
    //plus the pageCount color, override works better for clarity in the end. The selection blinks (if saveLodConfirm is false)
    //to make is easy to see page numbers underneath the selection
    if (!saveLoadConfirmation) { 
      if (flashLEDctrl == 0) {flashLEDctrl = 30;}
    } else { flashLEDctrl = 0; }
    
    displaySelectionByPage(saveLoadSlot + 1, zeroColor, 16, pageCountColor, 0, zeroColor, false, 1 );
    if (flashLEDctrl < 15) {
      displaySelectionByPage(saveLoadSlot + 1, saveLoadColors, 16, zeroColor, 0, zeroColor, true, 1 );
    }
    
  } else if ( currentPage == CAPTURE_SAVES_PAGE || currentPage == LOAD_CAPTURES_PAGE || currentPage == ERASE_ALL_PAGE) {

    allowBeatFlash = false;
    
    
    byte captureColor[3] = {200,200,0};
    if ( currentPage == LOAD_CAPTURES_PAGE ) {
      captureColor[0] = 0;
    } else if (currentPage == ERASE_ALL_PAGE) {
      captureColor[1] = 0;
    }
    
    //input controls
    if (validEncoderButtonPress) {
      if (!saveLoadConfirmation) { 
        saveLoadConfirmation = true; 
      } else {
        if (currentPage == CAPTURE_SAVES_PAGE || currentPage == LOAD_CAPTURES_PAGE) {
          saveLoadConfirmation = false;
          Wire.beginTransmission(COPYCAT_ADDRESS);
          if ( currentPage == CAPTURE_SAVES_PAGE ) { 
            Wire.write(55); //code to process a capture
          } else if ( currentPage == LOAD_CAPTURES_PAGE ) {
            Wire.write(75); //code to load capture
          }
          Wire.endTransmission();
        } else if (currentPage == ERASE_ALL_PAGE) {
          
          byte pixelCounter = 0;
          pixels.clear();  //clear the LED register to prepare it to fill it during the erase cycle
          
          for (int saveCounter = 0; saveCounter < 256; saveCounter++) { //cycle through all saveBytes and set them to 0
            saveLoadSlot = saveCounter;
            saveTrack(DISK1_ADDR, "erase");
            delay(5);
            if (saveCounter % 16 == 0) {  //fill up LEDs with red color as process completes
              pixels.setPixelColor(pixelCounter, pixels.Color(255,0,0));
              pixels.show();
              pixelCounter += 1;
            }
          }
          saveLoadSlot = 0;
          currentPage = 1;
        }
      } 
    }
    
    
    if (validEncoderButtonHold && currentPage == CAPTURE_SAVES_PAGE ) {
      saveLoadConfirmation = false;
      Wire.beginTransmission(COPYCAT_ADDRESS);
      Wire.write(65); //code to process a backup
      Wire.endTransmission();
    }
    
    
    //led display
    
    if (!saveLoadConfirmation) { 
      if (flashLEDctrl == 0) {flashLEDctrl = 30;}
    } else { flashLEDctrl = 0; }
    
    if (flashLEDctrl < 15) {
      displaySelectionByPage(15, zeroColor, 1, captureColor, 0, zeroColor, false, 1 );
    }
    
    
  } else if ( currentPage == VELOCITY_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      trackContainer[currentTrack][VELOCITY] = changeByteValue(trackContainer[currentTrack][VELOCITY],encoderState,0,127,false, 5);
    }  
    
    //led controls
    
    byte selectionColor[3] = {0,255,0};
    byte tensColor[3] = {255,0,255};
    displaySelectionByPage(trackContainer[currentTrack][VELOCITY], selectionColor, 10, tensColor, 0, zeroColor, false, 0 );
    
  } else if ( currentPage == CHANNEL_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      if(lastNoteMillis[currentTrack] != 0){ //if the track has started playing a note already, stop it
        MIDI.sendNoteOff(trackContainer[currentTrack][PITCH],0,trackContainer[currentTrack][CHANNEL]);
        lastNoteMillis[currentTrack] = 0;
      }
      trackContainer[currentTrack][CHANNEL] = changeByteValue(trackContainer[currentTrack][CHANNEL],encoderState,1,16,true,1);
    }
    
    //led display
    
    byte trackChannelColors[3] = {0,0,255};
    //page1 is used with a value of 1 per page instead of the usual selection process to fill in all LEDs up to the currentTrack's channel
    displaySelectionByPage(trackContainer[currentTrack][CHANNEL], zeroColor, 1, trackChannelColors, 0, zeroColor, false, 0 );
    
  } else if ( currentPage == TRACK_LENGTH_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      trackLength = changeByteValue(trackLength,encoderState,1,16,false,1);
      //recalculate all track START_NOTEs and NOTE_QTYs if length changes
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        trackContainer[trackCycler][NOTE_QTY] = cleanupByteValue(trackContainer[trackCycler][NOTE_QTY],0,trackLength,false);
        trackContainer[trackCycler][START_NOTE] = cleanupByteValue(trackContainer[trackCycler][START_NOTE],0,trackLength - 1,false); 
      }
    }
    
    //led display
    
    byte trackLengthColors[3] = {255,0,0};
    //page1 is used with a value of 1 per page instead of the usual selection process to fill in all LEDs up to the trackLength's value
    displaySelectionByPage(trackLength, zeroColor, 1, trackLengthColors, 0, zeroColor, false, 0 );
    
    
  } else if ( currentPage == NOTESPREAD_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      trackContainer[currentTrack][NOTE_SPREAD] = changeByteValue(trackContainer[currentTrack][NOTE_SPREAD],encoderState,1,8,false,1);
    }
    
    //led display
    
    byte noteSpreadColors[3] = {0,255,0};
    //page1 is used with a value of 1 per page instead of the usual selection process to fill in all LEDs up to the currentTrack's noteSpread value
    displaySelectionByPage(trackContainer[currentTrack][NOTE_SPREAD], zeroColor, 1, noteSpreadColors, 0, zeroColor, false, 0 );
    
    
  } else if ( currentPage == BRIGHTNESS_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      ledBrightness = changeByteValue(ledBrightness,encoderState,1,16,false,1);
      changeBrightness = true; //trigger to update the brightness;
    }
    
    //led display
    
    byte brightnessColors[3] = {255,255,255};
    //page1 is used with a value of 1 per page instead of the usual selection process to fill in all LEDs up to the ledBrightness value
    displaySelectionByPage(ledBrightness, zeroColor, 1, brightnessColors, 0, zeroColor, false, 0 );
    
    
    
  } else if ( currentPage == NOTE_LENGTH_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      trackContainer[currentTrack][NOTE_LENGTH] = changeByteValue(trackContainer[currentTrack][NOTE_LENGTH],encoderState,1,16,false,1);
    }
    
    //led display
    
    byte noteLengthColors[3] = {255,255,0};
    //page1 is used with a value of 1 per page instead of the usual selection process to fill in all LEDs up to the ledBrightness value
    displaySelectionByPage(trackContainer[currentTrack][NOTE_LENGTH], zeroColor, 1, noteLengthColors, 0, zeroColor, false, 0 );
    
    
  } else if ( currentPage == BEATFLASH_PAGE) {
    
    //input controls
    
    //if the encoder was spun and the encoder's button delay has passed....
    if(validEncoderSpinChange) {
      beatFlash = changeByteValue(beatFlash,encoderState,0,16,false,1);
    }
    
    //led display
    
    byte beatFlashColors[3] = {0,255,255};
    //page1 is used with a value of 1 per page instead of the usual selection process to fill in all LEDs up to the beatFlash value
    displaySelectionByPage(beatFlash, zeroColor, 1, beatFlashColors, 0, zeroColor, false, 0 );
    
    
  
  //MIDI_SUPPRESS_PAGE code slated for removal, as i don't think the suppression transistor will make it to the PCB design
  /*} else if ( currentPage == MIDI_SUPPRESS_PAGE ) {

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
    //purposely left to this method so the suppressSlot changes if the suppressByChannel mode does
    if(midiSuppressByChannel) {
      channelTrackSuppressSlot = cleanupByteValue(channelTrackSuppressSlot,0,15,true);
    } else {
      channelTrackSuppressSlot = cleanupByteValue(channelTrackSuppressSlot,0,11,true);
    }
    
    //led display
    allowBeatFlash = false;
    byte selectionColor[3] = {255,255,255};
    
    if (midiSuppressByChannel) { //color in LEDs for active (non-suppressed) channels
      byte channelColors[3] = {0,0,255};
      for (byte channelDisplay = 0; channelDisplay < 16; channelDisplay++) {
        if(!bitRead(midiChannelSuppress,channelDisplay)) {
          for (byte colorCheck = 0; colorCheck < 3; colorCheck++){
            if (channelColors[colorCheck] > 0) {
              ledByteContainer[colorCheck][channelDisplay] = channelColors[colorCheck];
            }
          }
        }
      }
      
    } else { //color in LEDs for active (non-suppressed) tracks
      for(byte trackCycler = 0; trackCycler < TRACK_QTY; trackCycler++) {
        if(!trackContainer[trackCycler][MIDI_TRACK_SUPPRESS]) {
          for(byte colorCycler = 0; colorCycler < 3; colorCycler ++) {
            if(trackColors[trackCycler][colorCycler] > 0) {
              ledByteContainer[colorCycler][trackCycler] = trackColors[trackCycler][colorCycler];
            }
          }
        }
      }  
    }

    //Make slot selected blink white or not
    if(flashLEDctrl == 0) {flashLEDctrl = 20;} //reset counter if run down
    if(flashLEDctrl < 13) { //if outside this threshold...
      for (byte colorCycler = 0; colorCycler < 3; colorCycler++){ // ...white out the selection choice
        ledByteContainer[colorCycler][channelTrackSuppressSlot] = 255;
      }
    }
  */  
  
  

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

  previousPage = currentPage; //page change detection for book keeping
  
  
  //Push containers out to LEDs
  /*
  //RGB LED push
  unsigned int ledContainer[3] = { //stores on/off RGB values for LED's (7 colors)
    0b0000000000000000,
    0b0000000000000000,
    0b0000000000000000
  };
  for (byte ledCycler = 0; ledCycler < 16; ledCycler++) {
    for (byte colorCycler = 0; colorCycler < 3; colorCycler++){
      if(ledByteContainer[colorCycler][ledCycler] > 0) {
        bitSet(ledContainer[colorCycler], 15 - ledCycler);
      } else {
        bitClear(ledContainer[colorCycler], 15 - ledCycler);
      }
    }
  }
  for(int j = 2; j >= 0; j--) {
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, lowByte(ledContainer[j])); 
    shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, highByte(ledContainer[j]));
  }
  digitalWrite(LATCH_PIN, HIGH);
  digitalWrite(LATCH_PIN, LOW);
  */
  
  //NeoPixel Push
  #ifdef WS2812_led
    for (int pixelCycler =  0; pixelCycler < NUMPIXELS; pixelCycler++){
      pixels.setPixelColor(pixelCycler, pixels.Color(ledByteContainer[0][pixelCycler],ledByteContainer[1][pixelCycler],ledByteContainer[2][pixelCycler]));
    }
    pixels.show();
  #endif
  
  
}
