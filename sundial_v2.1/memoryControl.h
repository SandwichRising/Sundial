#ifndef HEADER_MEMORYCONTROL
  #define HEADER_MEMORYCONTROL

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



  void saveTrack (byte diskAddress, String saveType) ;

  int pullEepromByte (byte diskAddress, int byteAddress, byte qtyBytes) ;

  void putEepromByte (byte diskAddress, int byteAddress, byte qtyBytes, int data) ;

  void putEepromTrackContainer (byte diskAddress) ;


#endif