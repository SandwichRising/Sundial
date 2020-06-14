#ifndef HEADER_INPUTSOUTPUTS
  #define HEADER_INPUTSOUTPUTS


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
  //single button pages
  #define SEQUENCER_PAGE 1
  #define PITCH_PAGE 2
  #define INSTRUMENT_PAGE 4
  #define TEMPO_PAGE 8
  #define SAVE_PAGE 16
  #define LOAD_PAGE 32
  
  //single button encoder-shifted pages
  #define VELOCITY_PAGE 130
  #define CHANNEL_PAGE 132
  #define TRACK_LENGTH_PAGE 136
  
  //two button pages
  #define NOTESPREAD_PAGE 3
  #define BRIGHTNESS_PAGE 12
  #define ERASE_PAGE 48
  
  //two button encoder-shifted pages
  #define NOTE_LENGTH_PAGE 131
  #define BEATFLASH_PAGE 140
  
  #define PAGE_QTY 14 //needs to be equal to total number of pages
  //each page needs added to this array
  const byte validPages[PAGE_QTY] = {SEQUENCER_PAGE, PITCH_PAGE, TEMPO_PAGE, INSTRUMENT_PAGE, SAVE_PAGE, LOAD_PAGE, ERASE_PAGE, NOTESPREAD_PAGE, BRIGHTNESS_PAGE, NOTE_LENGTH_PAGE, VELOCITY_PAGE, TRACK_LENGTH_PAGE, CHANNEL_PAGE, BEATFLASH_PAGE};
    
  
  void timerHandler ( void ) ;
  
  byte cleanupByteValue (byte value, byte lowestVal, byte highestVal, bool rollover) ;
  
  void updateEncoder ( void ) ;

  void countByTensLED (byte tensColor[3], byte selectionColor[3], byte LEDvariable) ;

  void countBySixteensLED (byte pageCountColor[3], byte selectionColor[3], int LEDvariable, bool overridePageColor) ;

  void countByHundredsLED (byte hundredsColor[3], byte tensColor[3], byte selectionColor[3], int LEDvariable) ;

  void pageControl ( void );



#endif
