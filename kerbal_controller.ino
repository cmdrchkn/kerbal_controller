//---------------//
// -- Imports -- //
//---------------//
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <KerbalSimpit.h>
#include <LiquidCrystal_I2C.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"



//-----------------//
// -- Constants -- //
//-----------------//
// -- LED Gauges
#define GAUGE_LED_ARRAY_DATA_PIN 14
#define GAUGE_LED_ARRAY_COUNT 64
const int GAUGE_LED_FUEL_INDICES[] = { 0, 15, 1, 14, 2, 13, 3, 12, 4, 11, 5, 10, 6, 9, 7, 8 };
const int GAUGE_LED_OXI_INDICES[] = { 16, 31, 17, 30, 18, 29, 19, 28, 20, 27, 21, 26, 22, 25, 23, 24 };
const int GAUGE_LED_MONO_INDICES[] = { 32, 47, 33, 46, 34, 45, 35, 44, 36, 43, 37, 42, 38, 41, 39, 40 };
const int GAUGE_LED_ELECTRIC_INDICES[] = { 48, 63, 49, 62, 50, 61, 51, 60, 52, 59, 53, 58, 54, 57, 55, 56 };

// -- LED Status Panel
#define STATUS_LED_ARRAY_DATA_PIN 15
#define STATUS_LED_ARRAY_COUNT 16
const int STATUS_LED_MASTER_CAUTION_A = 3;
const int STATUS_LED_LOW_FUEL = 2;
const int STATUS_LED_MASTER_CAUTION_B = 4;
const int STATUS_LED_MASTER_CAUTION_TRIGGERS[] = { 0, 1, 2, 5, 6, 7 };
const int STATUS_LED_LOW_OXI = 1;
const int STATUS_LED_LOW_ELEC = 0;
const int STATUS_LED_HIGH_TEMP = 5;
const int STATUS_LED_HIGH_G_FORCE = 6;
const int STATUS_LED_LOW_MONO = 7;
const int STATUS_LED_SUB_ORBITAL = 11;
const int STATUS_LED_COMM_SIGNAL = 12;
const int STATUS_LED_NODE_EXEC = 13;
#define SITUATION_LANDED 0
#define SITUATION_SPLASHED 1
#define SITUATION_PRELAUNCH 2
#define SITUATION_FLYING 3
// #define SITUATION_SUB_ORBITAL 4
#define SITUATION_SUB_ORBITAL 16  // docs are wrong?
#define SITUATION_REENTRY 8       // docs are wrong?
#define SITUATION_ORBITING 5
#define SITUATION_ESCAPING 6
#define SITUATION_DOCKED 7

// -- Char LCD
const char CLEAR_LINE[] = "                    ";


// -- LED Numeric Displays
#define LED_ALT_0_I2C_ADDRESS 0x74
#define LED_ALT_1_I2C_ADDRESS 0x70
#define LED_ALT_2_I2C_ADDRESS 0x73
#define LED_SPD_0_I2C_ADDRESS 0x75
#define LED_SPD_1_I2C_ADDRESS 0x71
#define LED_SPD_2_I2C_ADDRESS 0x72

// -- Lockout Switch
#define LOCKOUT_SWITCH_PIN 0

// -- Buttons
#define BUTTON_STAGE_PIN 1







//---------------//
// -- Globals -- //
//---------------//
bool MASTER_CAUTION = false;
int STATUS_LED_STATE[STATUS_LED_ARRAY_COUNT];


//--------------------//
// -- Constructors -- //
//--------------------//
// -- Simpit
KerbalSimpit mySimpit(Serial);

// -- OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// - Status LED Panel
Adafruit_NeoPixel STATUS_LED_ARRAY(STATUS_LED_ARRAY_COUNT, STATUS_LED_ARRAY_DATA_PIN, NEO_GRBW + NEO_KHZ800);

// -- LED Gauges
Adafruit_NeoPixel GAUGE_LED_ARRAY(GAUGE_LED_ARRAY_COUNT, GAUGE_LED_ARRAY_DATA_PIN, NEO_GRB + NEO_KHZ800);

// -- Char LCD
LiquidCrystal_I2C CHAR_LCD(0x3f, 20, 4);

// -- LED Numeric Displays
Adafruit_AlphaNum4 LED_ALT_0 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_ALT_1 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_ALT_2 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_SPD_0 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_SPD_1 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_SPD_2 = Adafruit_AlphaNum4();



//-------------------------//
// -- Runtime Constants -- //
//-------------------------//
// -- LED Colors
#define RED GAUGE_LED_ARRAY.Color(255, 0, 0)
#define GREEN GAUGE_LED_ARRAY.Color(0, 255, 0)
#define BLUE GAUGE_LED_ARRAY.Color(0, 0, 255)
#define CYAN GAUGE_LED_ARRAY.Color(0, 255, 255)
#define PURPLE GAUGE_LED_ARRAY.Color(255, 0, 255)
#define YELLOW GAUGE_LED_ARRAY.Color(255, 255, 0)
#define ORANGE GAUGE_LED_ARRAY.Color(255, 128, 0)
#define WHITE GAUGE_LED_ARRAY.Color(255, 255, 255)
#define BLACK GAUGE_LED_ARRAY.Color(0, 0, 0)
#define WHITE_W STATUS_LED_ARRAY.Color(255, 0, 0, 0)
#define BLACK_W STATUS_LED_ARRAY.Color(0, 0, 0, 0)

//-----------------------//
// -- Runtime Globals -- //
//-----------------------//
Adafruit_AlphaNum4 ALTITUDE_LED_DISPLAYS[] = { LED_ALT_0, LED_ALT_1, LED_ALT_2 };
Adafruit_AlphaNum4 SPEED_LED_DISPLAYS[] = { LED_SPD_0, LED_SPD_1, LED_SPD_2 };


//-----------------//
// -- Functions -- //
//-----------------//
void u8g2_prepare(void)
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  //u8g2.setFlipMode(0);

  u8g2.clearBuffer();
  u8g2.drawStr(0, 0, "Waiting for KSP");
  u8g2.sendBuffer();
}


void char_lcd_prepare()
{
  CHAR_LCD.init();
  CHAR_LCD.backlight();
  CHAR_LCD.clear();
  CHAR_LCD.setCursor(0,0);
  CHAR_LCD.print("    Waiting for");
  CHAR_LCD.setCursor(0,1);
  CHAR_LCD.print("    KerbalSimpit");
  CHAR_LCD.setCursor(0,2);
  CHAR_LCD.print("     Connection");
  CHAR_LCD.setCursor(0,3);
}


void char_lcd_clear_line(int line)
{
  CHAR_LCD.setCursor(0,line);
  CHAR_LCD.print(CLEAR_LINE);
  CHAR_LCD.setCursor(0,line);
}


void led_segment_prepare()
{
  // test pattern for LED segments
  char test[] = "TEST";

  // -- Altitude Displays
  LED_ALT_0.begin(LED_ALT_0_I2C_ADDRESS);
  LED_ALT_0.setBrightness(8);
  LED_ALT_0.clear();
  LED_ALT_0.writeDisplay();
  LED_ALT_1.begin(LED_ALT_1_I2C_ADDRESS);
  LED_ALT_1.setBrightness(8);
  LED_ALT_1.clear();
  LED_ALT_1.writeDisplay();
  LED_ALT_2.begin(LED_ALT_2_I2C_ADDRESS);
  LED_ALT_2.setBrightness(8);
  LED_ALT_2.clear();
  LED_ALT_2.writeDisplay();

  // -- Speed Displays
  LED_SPD_0.begin(LED_SPD_0_I2C_ADDRESS);
  LED_SPD_0.setBrightness(1);
  LED_SPD_0.clear();
  LED_SPD_0.writeDisplay();
  LED_SPD_1.begin(LED_SPD_1_I2C_ADDRESS);
  LED_SPD_1.setBrightness(1);
  LED_SPD_1.clear();
  LED_SPD_1.writeDisplay();
  LED_SPD_2.begin(LED_SPD_2_I2C_ADDRESS);
  LED_SPD_2.setBrightness(1);
  LED_SPD_2.clear();
  LED_SPD_2.writeDisplay();

  // -- write test pattern
  for(int i = 0; i < 4; i++)
  {
  	    LED_ALT_0.writeDigitAscii(i, test[i]);
        LED_ALT_1.writeDigitAscii(i, test[i]);
        LED_ALT_2.writeDigitAscii(i, test[i]);
        LED_SPD_0.writeDigitAscii(i, test[i]);
        LED_SPD_1.writeDigitAscii(i, test[i]);
        LED_SPD_2.writeDigitAscii(i, test[i]);
  } 

  // -- Display test pattern
  LED_ALT_0.writeDisplay();
  LED_ALT_1.writeDisplay();
  LED_ALT_2.writeDisplay();
  LED_SPD_0.writeDisplay();
  LED_SPD_1.writeDisplay();
  LED_SPD_2.writeDisplay();
}


void update_led_segment_display(Adafruit_AlphaNum4 display, char* value)
{
  for (int i=0; i<4; i++)
  {
    display.writeDigitAscii(i, value[i]);
  }
  display.writeDisplay();
  CHAR_LCD.setCursor(0,3);
  CHAR_LCD.print(value);
  CHAR_LCD.display();
}


char* format_distance_value(float distance, char* buffer)
{
  char distance_chars[11];
  char unit[3];
  char mode[3];
  strcpy(unit, " M");
  
  if (abs(distance) > 999999999)
  {
    distance = distance / 1000 / 1000;
    strcpy(unit, "MM");
  }
  else if (abs(distance) > 999999)
  {
    distance = distance / 1000;
    strcpy(unit, "Km");
  }
  
  sprintf(distance_chars, " %6d %s", int(distance), unit);
  // if (distance < 0.0)
  // {
  //   distance_chars[0] = '-';
  // }
  strcpy(buffer, distance_chars);
  return buffer;
}


void update_altiude(float alt)
{
  char alt_chars[13];
  char mode[3];
  strcpy(mode, " S");
  format_distance_value(alt, alt_chars);
  
  LED_ALT_0.writeDigitAscii(0, alt_chars[0]);
  LED_ALT_0.writeDigitAscii(1, alt_chars[1]);
  LED_ALT_0.writeDigitAscii(2, alt_chars[2]);
  LED_ALT_0.writeDigitAscii(3, alt_chars[3]);
  LED_ALT_1.writeDigitAscii(0, alt_chars[4]);
  LED_ALT_1.writeDigitAscii(1, alt_chars[5]);
  LED_ALT_1.writeDigitAscii(2, alt_chars[6]);
  LED_ALT_1.writeDigitAscii(3, alt_chars[7]);
  LED_ALT_2.writeDigitAscii(0, alt_chars[8]);
  LED_ALT_2.writeDigitAscii(1, alt_chars[9]);
  LED_ALT_2.writeDigitAscii(2, mode[0]);
  LED_ALT_2.writeDigitAscii(3, mode[1]);
}


void update_apoapsis(float apo)
{
  char label[] = "Appoapsis:";
  char apo_val[11];
  char buffer[strlen(label) + strlen(apo_val) + 2];
  format_distance_value(apo, apo_val);
  sprintf(buffer, "%s %s", label, apo_val);

  u8g2.drawStr(3, 1, buffer);
  u8g2.drawFrame(0, 0, u8g2.getDisplayWidth(), 1);
  u8g2.drawFrame(0, 22, u8g2.getDisplayWidth(), 1);
}


void update_apoapsis_time(int apo)
{
  char buffer[22];
  char final_buffer[22];
  duration_in_seconds_to_dhms_string(apo, buffer);
  sprintf(final_buffer, "%21s", buffer);
  u8g2.drawStr(3, 13, final_buffer);
}


void update_periapsis(float peri)
{
  char label[] = "Periapsis:";
  char peri_val[11];
  char buffer[strlen(label) + strlen(peri_val) + 2];
  format_distance_value(peri, peri_val);
  sprintf(buffer, "%s %s", label, peri_val);

  u8g2.drawStr(3, 28, buffer);
  u8g2.drawFrame(0, 51, u8g2.getDisplayWidth(), 1);
}


void update_periapsis_time(int peri)
{
  char buffer[22];
  char final_buffer[22];
  duration_in_seconds_to_dhms_string(peri, buffer);
  sprintf(final_buffer, "%21s", buffer);
  u8g2.drawStr(3, 41, final_buffer);
}


void update_led_gauge(float total, float available, const int indices[], uint32_t on_color, uint32_t off_color, uint32_t empty_color, int low_status_led_idx, uint32_t low_status_color)
{
  float resource_amount_percent = available / total * 100;
  int resource_amount_led_count = map(available, 0, total, 0, 16);
  int gauge_start_idx = indices[0];

  for (int i=0; i<16; i++)
  {
    int this_led_idx = indices[i];
    if (i <= resource_amount_led_count)
    {
      GAUGE_LED_ARRAY.setPixelColor(this_led_idx, on_color);
    }
    else
    {
      GAUGE_LED_ARRAY.setPixelColor(this_led_idx, off_color);
    }
  }

  if (resource_amount_percent <= 15.0)
  {
    update_status_led(low_status_led_idx, low_status_color);
  }
  else if (resource_amount_percent < 1.0)
  {
    GAUGE_LED_ARRAY.setPixelColor(gauge_start_idx, empty_color);
    update_status_led(low_status_led_idx, RED);
  }
  else
  {
    update_status_led(low_status_led_idx, BLACK_W);
  }
}


void update_status_led(const int index, uint32_t color)
{
  STATUS_LED_ARRAY.setPixelColor(index, color);
  STATUS_LED_STATE[index] = color;
}


void update_status_comm(float comm_strength)
{
  if (comm_strength >= 80.0)
  {
    update_status_led(STATUS_LED_COMM_SIGNAL, GREEN);
  }
  else if (comm_strength >= 50.0)
  {
    update_status_led(STATUS_LED_COMM_SIGNAL, BLUE);
  }
  else if (comm_strength >= 20.0)
  {
    update_status_led(STATUS_LED_COMM_SIGNAL, YELLOW);
  }
  else if (comm_strength > 0)
  {
    update_status_led(STATUS_LED_COMM_SIGNAL, ORANGE);
  }
  else
  {
    update_status_led(STATUS_LED_COMM_SIGNAL, RED);
  }
}


void update_status_high_g_force(float g_force)
{
  if (g_force >= 5.0)
  {
    update_status_led(STATUS_LED_HIGH_G_FORCE, RED);
  }
  else if (g_force >= 4.0)
  {
    update_status_led(STATUS_LED_HIGH_G_FORCE, ORANGE);
  }
  else if (g_force >= 3.0)
  {
    update_status_led(STATUS_LED_HIGH_G_FORCE, YELLOW);
  }
  else
  {
    update_status_led(STATUS_LED_HIGH_G_FORCE, BLACK_W);
  }
}


void update_status_high_temp(float temp_limit_percentage)
{
  // mySimpit.printToKSP(String(temp_limit_percentage), PRINT_TO_SCREEN);
  if (temp_limit_percentage >= 95.0)
  {
    update_status_led(STATUS_LED_HIGH_TEMP, RED);
  }
  else if (temp_limit_percentage >= 90.0)
  {
    update_status_led(STATUS_LED_HIGH_TEMP, ORANGE);
  }
  else if (temp_limit_percentage >= 85.0)
  {
    update_status_led(STATUS_LED_HIGH_TEMP, YELLOW);
  }
  else
  {
    update_status_led(STATUS_LED_HIGH_TEMP, BLACK_W);
  }
}


void update_status_sub_orbit(int vessel_situation)
{
  // mySimpit.printToKSP(String(vessel_situation), PRINT_TO_SCREEN);
  if (vessel_situation == SITUATION_SUB_ORBITAL)
  {
    update_status_led(STATUS_LED_SUB_ORBITAL, ORANGE);
  }
  else if (vessel_situation == SITUATION_REENTRY)
  {
    update_status_led(STATUS_LED_SUB_ORBITAL, RED);
  }
  else
  {
    update_status_led(STATUS_LED_SUB_ORBITAL, BLACK_W);
  }
}

void check_master_caution()
{
  update_status_led(STATUS_LED_MASTER_CAUTION_A, BLACK_W);
  update_status_led(STATUS_LED_MASTER_CAUTION_B, BLACK_W);
  MASTER_CAUTION = false;

  for (int i : STATUS_LED_MASTER_CAUTION_TRIGGERS)
  {
    if (STATUS_LED_STATE[i] != BLACK_W)
    {
      MASTER_CAUTION = true;
      update_status_led(STATUS_LED_MASTER_CAUTION_A, ORANGE);
      update_status_led(STATUS_LED_MASTER_CAUTION_B, ORANGE);
      break;
    }
  }
}

char* duration_in_seconds_to_dhms_string(int duration, char* buffer)
{
  int days;
  int hours = duration / 3600;
  int minutes = (duration / 60) % 60;
  int seconds = duration % 60;

  if (hours >= 24)
  {
    days = hours / 24;
    hours = hours - 24;
    sprintf(buffer, "%dd %dh %dm %ds", days, hours, minutes, seconds);
  }
  else if (hours > 0)
  {
    sprintf(buffer, "%dh %dm %ds", hours, minutes, seconds);
  }
  else if (minutes > 0)
  {
    sprintf(buffer, "%dm %ds", minutes, seconds);
  }
  else if (seconds > 0)
  {
    sprintf(buffer, "%ds", seconds);
  }
  else  // something broke, fall back to raw duration
  {
    sprintf(buffer, "%fs", duration);
  }

  return buffer;
}


void Handle_Simpit_Message(byte messageType, byte message[], byte msgSize)
{
  switch (messageType)
  {
    case ALTITUDE_MESSAGE:
      if (msgSize == sizeof(altitudeMessage))
      {
        altitudeMessage myAltitude;
        myAltitude = parseMessage<altitudeMessage>(message);
        update_altiude(myAltitude.surface);
      }
      break;
    case APSIDESTIME_MESSAGE:
      if (msgSize == sizeof(apsidesTimeMessage))
      {
        apsidesTimeMessage myApsidesTime;
        myApsidesTime = parseMessage<apsidesTimeMessage>(message);
        update_apoapsis_time(myApsidesTime.apoapsis);
        update_periapsis_time(myApsidesTime.periapsis);
      }
      break;
    case APSIDES_MESSAGE:
      if (msgSize == sizeof(apsidesMessage))
      {
        apsidesMessage myApsides;
        myApsides = parseMessage<apsidesMessage>(message);
        update_apoapsis(myApsides.apoapsis);
        update_periapsis(myApsides.periapsis);
      }
      break;
    case LF_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
      {
        resourceMessage myLFuel;
        myLFuel = parseMessage<resourceMessage>(message);
        update_led_gauge(myLFuel.total, myLFuel.available, GAUGE_LED_FUEL_INDICES, YELLOW, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
      }
    break;
    case LF_STAGE_MESSAGE:
      break;
    case OX_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
      {
        resourceMessage myOxi;
        myOxi = parseMessage<resourceMessage>(message);
        update_led_gauge(myOxi.total, myOxi.available, GAUGE_LED_OXI_INDICES, CYAN, BLACK, RED, STATUS_LED_LOW_OXI, YELLOW);
      }
      break;
    case OX_STAGE_MESSAGE:
      break;
    case SF_MESSAGE:
      break;
    case SF_STAGE_MESSAGE:
      break;
    case MONO_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
      {
        resourceMessage myMono;
        myMono = parseMessage<resourceMessage>(message);
        update_led_gauge(myMono.total, myMono.available, GAUGE_LED_MONO_INDICES, WHITE, BLACK, RED, STATUS_LED_LOW_MONO, YELLOW);
      }
      break;
    case ELECTRIC_MESSAGE:
      if (msgSize == sizeof(resourceMessage))
      {
        resourceMessage myElectric;
        myElectric = parseMessage<resourceMessage>(message);
        update_led_gauge(myElectric.total, myElectric.available, GAUGE_LED_ELECTRIC_INDICES, GREEN, BLACK, RED, STATUS_LED_LOW_ELEC, YELLOW);
      }
      break;
    case ACTIONSTATUS_MESSAGE:
      if (message[0] & SAS_ACTION)
      {
        // code to execute if SAS is active
      }
      if (message[0] & STAGE_ACTION)
      {
        // code to execute if staging is active
      }
      if (message[0] & GEAR_ACTION)
      {
        // code to execute if gear is active
      }
      if (message[0] & LIGHT_ACTION)
      {
        // code to execute if lights is active
      }
      if (message[0] & RCS_ACTION)
      {
        // code to execute if rcs is active
      }
      if (message[0] & BRAKES_ACTION)
      {
        // code to execute if brakes is active
      }
      if (message[0] & ABORT_ACTION)
      {
        // code to execute if abort is active
      }
      break;
    case FLIGHT_STATUS_MESSAGE:
      if (msgSize == sizeof(flightStatusMessage))
      {
        flightStatusMessage myFlightStatus;
        myFlightStatus = parseMessage<flightStatusMessage>(message);
        update_status_comm(myFlightStatus.commNetSignalStrenghPercentage);
        update_status_sub_orbit(myFlightStatus.vesselSituation);
      }
      break;
    case AIRSPEED_MESSAGE:
      if (msgSize == sizeof(airspeedMessage))
      {
        airspeedMessage myAirSpeed;
        myAirSpeed = parseMessage<airspeedMessage>(message);
        update_status_high_g_force(myAirSpeed.gForces);
      }
      break;
    case MANEUVER_MESSAGE:
      if (msgSize == sizeof(maneuverMessage))
      {
        maneuverMessage myManeuver;
        myManeuver = parseMessage<maneuverMessage>(message);
      }
      break;
    case TEMP_LIMIT_MESSAGE:
      if (msgSize == sizeof(tempLimitMessage))
      {
        tempLimitMessage myTempLimit;
        myTempLimit = parseMessage<tempLimitMessage>(message);
        update_status_high_temp(myTempLimit.skinTempLimitPercentage);
      }
      break;
  }
}


void test_led_guage()
{
  update_led_gauge(1, 1, GAUGE_LED_FUEL_INDICES, YELLOW, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_OXI_INDICES, PURPLE, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_MONO_INDICES, BLUE, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_ELECTRIC_INDICES, RED, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
}


void test_status_led()
{
  STATUS_LED_ARRAY.fill(BLUE, 0, STATUS_LED_ARRAY_COUNT);
}



//-------------//
// -- Setup -- //
//-------------//
void setup(void)
{
  // -- LED Gauge Setup
  GAUGE_LED_ARRAY.begin();
  STATUS_LED_ARRAY.begin();
  GAUGE_LED_ARRAY.setBrightness(3);
  STATUS_LED_ARRAY.setBrightness(10);
  test_led_guage();
  test_status_led();
  GAUGE_LED_ARRAY.show();
  STATUS_LED_ARRAY.show();

  // -- Lockout Switch Setup
  pinMode(LOCKOUT_SWITCH_PIN, INPUT_PULLUP);

  // -- Button Setup
  pinMode(BUTTON_STAGE_PIN, INPUT_PULLUP);

  // -- OLED setup
  u8g2_prepare();

  // -- Char LCD setup
  char_lcd_prepare();

  // -- LED Segment setup
  led_segment_prepare();

  // -- Simpit setup
  Serial.begin(115200);
  int connection_attempts = 0;
  
  while (!mySimpit.init())
  {
    u8g2.clearBuffer();
    if (connection_attempts % 2 == 0)
    {
      u8g2.drawStr(0, 0, "Waiting for KSP...");
      update_status_led(STATUS_LED_MASTER_CAUTION_A, ORANGE);
      update_status_led(STATUS_LED_MASTER_CAUTION_B, ORANGE);
      update_status_led(STATUS_LED_COMM_SIGNAL, RED);
      CHAR_LCD.print(".");
    }
    else
    {
      u8g2.drawStr(0, 0, "Waiting for KSP.. ");
      update_status_led(STATUS_LED_MASTER_CAUTION_A, BLACK_W);
      update_status_led(STATUS_LED_MASTER_CAUTION_B, BLACK_W);
      update_status_led(STATUS_LED_COMM_SIGNAL, BLACK_W);
    }
    u8g2.sendBuffer();
    GAUGE_LED_ARRAY.show();
    STATUS_LED_ARRAY.show();
    delay(100);

    connection_attempts++;
    if (connection_attempts > 20)
    {
      connection_attempts = 0;
      char_lcd_clear_line(3);
    }
  }
  mySimpit.printToKSP("Connected to CKN Industries Controller", PRINT_TO_SCREEN);
  mySimpit.registerChannel(ALTITUDE_MESSAGE);
  mySimpit.registerChannel(APSIDES_MESSAGE);
  mySimpit.registerChannel(LF_MESSAGE);
  mySimpit.registerChannel(OX_MESSAGE);
  mySimpit.registerChannel(MONO_MESSAGE);
  mySimpit.registerChannel(ELECTRIC_MESSAGE);
  mySimpit.registerChannel(APSIDESTIME_MESSAGE);
  mySimpit.registerChannel(MANEUVER_MESSAGE);
  mySimpit.registerChannel(FLIGHT_STATUS_MESSAGE);
  mySimpit.registerChannel(TEMP_LIMIT_MESSAGE);
  mySimpit.registerChannel(AIRSPEED_MESSAGE);
  mySimpit.inboundHandler(Handle_Simpit_Message);

  GAUGE_LED_ARRAY.fill(BLACK, 0, GAUGE_LED_ARRAY_COUNT);
  STATUS_LED_ARRAY.fill(BLACK_W, 0, STATUS_LED_ARRAY_COUNT);
  CHAR_LCD.clear();
}



//------------//
// -- Loop -- //
//------------//
void loop(void)
{
  // -- Fetch updates
  mySimpit.update();
  check_master_caution();

  // -- Display updates
  u8g2.sendBuffer();
  GAUGE_LED_ARRAY.show();
  STATUS_LED_ARRAY.show();
  LED_ALT_0.writeDisplay();
  LED_ALT_1.writeDisplay();
  LED_ALT_2.writeDisplay();
  LED_SPD_0.writeDisplay();
  LED_SPD_1.writeDisplay();
  LED_SPD_2.writeDisplay();

  //-- Sleep before next update
  delay(150);

  // -- Reset OLED frame buffer
  u8g2.clearBuffer();
}
