#include "QuickGPS.h"
#include "string.h"
#include "stdlib.h"
#include <time.h>

typedef enum {
  GLL,
  GGA,
  RMC
} NMEAType;

typedef enum {
  RATE = 40,
  CONFIG = 41
} PubxType;

typedef enum {
  IN_UBX = 1,
  IN_NMEA = 2
} ConfigInProto;

typedef enum {
  OUT_UBX = 1,
  OUT_NMEA = 2
} ConfigOutProto;

const int QuickGPS::NMEA_BUFF_SIZE;

QuickGPS::QuickGPS(HardwareSerial* _serial)
  : serial(_serial),
    cur_buff_index(0),
    no_lock_messages(0) {
  data.lock = false;
}

void QuickGPS::appendChecksum(char* msg) {
  unsigned char cs = 0;
  //start at 1 to ignore the '$'
  for (int i=1; msg[i] != '\0' && i < QuickGPS::NMEA_BUFF_SIZE; ++i) {
    cs ^= msg[i];
  }
  char msg_cpy[NMEA_BUFF_SIZE];
  strcpy(msg_cpy, msg);
  sprintf(msg, "%s*%02X", msg_cpy, cs);
}

void QuickGPS::enableMsg(const char* msg, bool enable) {
  char out[QuickGPS::NMEA_BUFF_SIZE];
  sprintf(out, "$PUBX,40,%s,%d,0,0,0,0,0\r\n", msg, enable ? 1 : 0);
  appendChecksum(out);
  serial->print(out);
}

void QuickGPS::sendConfig() {
  char out[QuickGPS::NMEA_BUFF_SIZE];
  sprintf(out, "$PUBX,41,1,%04d,%04d,9600,0\r\n", IN_NMEA, OUT_NMEA);
  appendChecksum(out);
  serial->print(out);
  enableMsg("RMC", true);
  enableMsg("VTG", false);
  enableMsg("GSV", false);
  enableMsg("GSA", false);
}

bool QuickGPS::begin() {
  serial->begin(9600);
  sendConfig();
  return true;
}

//Set maximum time so that we can quit in case serial takes too long
const static int MAX_MILLIS = 2;
const static int MAX_NO_LOCK_MSGS = 20;
bool QuickGPS::update() {
  
  bool ret = false;
  long cur_millis = millis();
  long prev_millis = cur_millis;
  while (serial->available()) {
    cur_millis = millis();
    if (cur_millis - prev_millis > MAX_MILLIS)
      break;
    
    if (cur_buff_index >= NMEA_BUFF_SIZE) {
      //Serial.println("OVERFLOW");
      cur_buff_index = 0;
    }
    
    char in = serial->read();
    
    if (in == '\n') {
      nmea_buffer[cur_buff_index++] = '\0';
      cur_buff_index = 0;
      //Serial.print("NMEA string: ");
      //Serial.println(nmea_buffer);
      ret = parseNMEA(nmea_buffer, &data);

      if (no_lock_messages > MAX_NO_LOCK_MSGS) {
	//Serial.println("Sending poke!");
	sendConfig();
	no_lock_messages = 0;
      } else if (data.lock) {
	no_lock_messages = 0;
      } else {
	++no_lock_messages;
      }
      
    } else {
      nmea_buffer[cur_buff_index++] = in;
    }
    
  }
  return ret;
}

QuickGPS::Data QuickGPS::readPosition() {
  return data;
}

static bool parseLatLon(const char* str, float* ll) {
  int len = strlen(str);
  if (len == 10 || len == 11) {
    char deg[4];
    char min[9];
    if (len == 10) {
      strncpy(deg, str, 2);
      deg[2] = '\0';
      strcpy(min, str+2);
    } else {
      strncpy(deg, str, 3);
      deg[3] = '\0';
      strcpy(min, str+3);
    }
    *ll = atoi(deg);
    *ll += atof(min)/60.0;
    return true;
  } else {
    return false;
  }
}

static bool parseNS(const char* str, QuickGPS::Data* data) {
  if (strlen(str) != 1) return false;
  switch(str[0]) {
  case 'N':
    break;
  case'S':
    data->lat = -data->lat;
    break;
  default:
    return false;
  }
  return true;
}

static bool parseEW(const char* str, QuickGPS::Data* data) {
  if (strlen(str) != 1) return false;
  switch(str[0]) {
  case 'E':
    break;
  case'W':
    data->lon = -data->lon;
    break;
  default:
    return false;
  }
  return true;
}

static uint64_t getTime(struct tm* time, uint64_t millis_offset) {

  uint64_t t = mktime(time);
  return t*1000LL + millis_offset;
  
};

static bool parseTime(const char* str, struct tm* time, uint64_t* millis_offset) {
  
  if (strlen(str) != 9) return false;
  
  char hours[3];
  char minutes[3];
  char seconds[3];
  char hund_seconds[3];
  
  strncpy(hours, str, 2);
  strncpy(minutes, str+2, 2);
  strncpy(seconds, str+4, 2);
  strncpy(hund_seconds, str+7, 2);
  hours[2] = '\0';
  minutes[2] = '\0';
  seconds[2] = '\0';
  hund_seconds[2] = '\0';
  
  *millis_offset = atoi(hund_seconds)*10LL;
  time->tm_sec = atoi(seconds);
  time->tm_min = atoi(minutes);
  time->tm_hour = atoi(hours);
  
  return true;
}

static bool parseDate(const char* str, struct tm* time) {

  if (strlen(str) != 6) return false;

  char days[3];
  char months[3];
  char years[3];

  strncpy(days, str, 2);
  strncpy(months, str+2, 2);
  strncpy(years, str+4, 2);
  days[2] = '\0';
  months[2] = '\0';
  years[2] = '\0';

  time->tm_mday = atoi(days);
  time->tm_mon = atoi(months) - 1;
  time->tm_year = 100 + atoi(years);

  return true;
}

bool QuickGPS::parseNMEA(const char* str, QuickGPS::Data* data) {

  struct tm time;
  uint64_t millis_offset = 0;
  QuickGPS::Data new_data = *data;
  NMEAType type;
  
  if (str[0] != '$') return false;
  
  //start by verifying checksum
  unsigned char checksum = 0x00;
  int index = 0;
  while (str[++index] != '\0' && str[index] != '*' && index < QuickGPS::NMEA_BUFF_SIZE) {
    checksum ^= str[index];
  }
  char given_check = strtol(str + index + 1, NULL, 16);
  if (str[index] == '\0' || given_check != checksum) {
    //Serial.println("Invalid checksum");
    return false;
  }
  
  const char* headless_str = str + 3;
  if (strncmp(headless_str, "GLL", 3) == 0) {
    type = GLL;
  } else if (strncmp(headless_str, "GGA", 3) == 0) {
    type = GGA;
  } else if (strncmp(headless_str, "RMC", 3) == 0) {
    type = RMC;
  } else {
    return true;
  }
  
  //assume the data doesn't work.
  //if everything checks out copying
  //new_data to data will set the lock
  data->lock = false;
  new_data.lock = true;

  char* tok_str = strdup(str);
  char* sub;
  index = 0;
  while((sub = strsep(&tok_str,",")) != NULL) {
    switch(type) {
      
    case GLL:
      switch(index) {
	
      case 1:
	if (!parseLatLon(sub,&new_data.lat)) return true;
	break;
	
      case 2:
	if (!parseNS(sub, &new_data))return true;
	break;
	
      case 3:
	if (!parseLatLon(sub,&new_data.lon)) return true;
	break;
	
      case 4:
	if (!parseEW(sub, &new_data)) return true;
	break;

      case 6:
	if (strlen(sub) != 1 || sub[0] != 'A')
	  return true;
	break;
	
      }
      break;

    case GGA:
      switch(index) {
	
      case 2:
	if (!parseLatLon(sub,&new_data.lat)) return true;
	break;
	
      case 3:
	if (!parseNS(sub, &new_data))return true;
	break;
	
      case 4:
	if (!parseLatLon(sub,&new_data.lon)) return true;
	break;
	
      case 5:
	if (!parseEW(sub, &new_data)) return true;
	break;

      case 9:
	char* end;
	new_data.alt = strtod(sub, &end);
	if (end == sub) return true;
	break;

      case 10:
	if (strlen(sub) != 1 || sub[0] != 'M')
	  return true;
	break;
	
      }
      break;
      
    case RMC:
      switch(index) {

      case 1:
	if (!parseTime(sub, &time, &millis_offset)) return true;
	break;

      case 2:
	if (strlen(sub) != 1 || sub[0] != 'A')
	  return true;
	break;
	
      case 3:
	if (!parseLatLon(sub,&new_data.lat)) return true;
	break;
	
      case 4:
	if (!parseNS(sub, &new_data))return true;
	break;
	
      case 5:
	if (!parseLatLon(sub,&new_data.lon)) return true;
	break;
	
      case 6:
	if (!parseEW(sub, &new_data)) return true;
	break;

      case 9:
	if (!parseDate(sub, &time)) return true;
	new_data.time = getTime(&time, millis_offset);
	break;
      }
      break;
    }
    ++index;
  }

  *data = new_data;
  return true;
}

