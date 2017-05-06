#ifndef QUICK_GPS_H
#define QUICK_GPS_H

#include <stdint.h>
#include <Arduino.h>

class QuickGPS {

 public:
  
  struct Data {
    //UNIX time in milliseconds
    uint64_t time;
    float lon;
    float lat;
    float alt;
    bool lock;
  };
  
  QuickGPS(HardwareSerial* _serial);
  bool begin();
  /*
   * reads a single char from the serial stream and, if it's '\n' parses the string
   * and returns true if the data has updated.
   */
  bool update();
  Data readPosition();

  static bool parseNMEA(const char* str, Data* data);
  
 private:

  void enableMsg(const char* msg, bool enable);
  void sendConfig();
  static void appendChecksum(char* msg);
  
  const static int NMEA_BUFF_SIZE = 1024;
  
  HardwareSerial* serial;
  char nmea_buffer[NMEA_BUFF_SIZE];
  int cur_buff_index;
  Data data;
  //record number of messages we recieve without lock
  //so we can 'poke' gps to try again
  unsigned int no_lock_messages;
  
};

#endif
