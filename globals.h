#ifndef HEADER_GLOBALS
  #define HEADER_GLOBALS

  #include "boardSelect.h"

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
  byte currentPage = 1; //controls what is displayed on the LED ring and what button inputs do
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
  //unsigned long timer10ms = 0; //milli() container for functions that will run every 10 ms
  unsigned long timer20ms = 0; //milli() container for functions that will run every 20 ms
  bool canRun1ms = true; //functions watch these ints to know if they are allowed to run
  //bool canRun10ms = true;
  bool canRun20ms = true;
  
  
  //array of pins to run multiplexer commands
  const int muxPins1[] = {
    BUTTON_MUX_PIN_1,BUTTON_MUX_PIN_2,BUTTON_MUX_PIN_3
  };
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
  byte flashTrackLED = 0; //flashes the currentTrack's led colors on the sequencer page 0 (set to 10 for a single 200ms flash, set to 20 for a double flash)
  //Brightness Variables
  byte ledBrightness = 2;  //controls the PWM to the 595 chips to control overall LED brightness
  byte previousLEDBrightness = 2; //used to redo math when ledBrightness changes, and to calculate when to save to EEPROM (and prevent saving too often)
  byte beatFlash = 3; //controls the intensity of the beat flashing, 0 to turn off
  byte previousBeatFlash = 3; //to calculate when to save to EEPROM
  bool changeBrightness = true; //for inputDetections() and ledControl() to communicate with each other about when to change brightness (inputDetection watches previousBrightness and handles saving to EEPROM, ledControl handles actually displaying brightness )
  bool allowBeatFlash = true; //allows processes to disable beatFlash'ing temporarily (only stays disabled while actively being set to false, re-enables each ledControl() cycle) 


#endif