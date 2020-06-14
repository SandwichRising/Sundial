#ifndef HEADER_BOARDSELECT
  #define HEADER_BOARDSELECT


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
  
    //MIDI Related Pins
    #define MIDI_SUPPRESS_PIN 11 //controls transistor (base) link between MIDI out and VS1053B in

  #endif



#endif