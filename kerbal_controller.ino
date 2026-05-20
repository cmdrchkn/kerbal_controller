//---------------//
// -- Imports -- //
//---------------//
#include "Adafruit_Debounce.h"
#include "Adafruit_LEDBackpack.h"
#include <Adafruit_GFX.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <KerbalSimpit.h>
#include <LiquidCrystal_I2C.h>
#include <U8g2lib.h>
#include <Wire.h>

//-----------------//
// -- Constants -- //
//-----------------//
#define LOOP_DELAY_MS 10

// -- LED Gauges
// #define GAUGE_LED_ARRAY_DATA_PIN 14
// #define GAUGE_LED_ARRAY_COUNT 64
const int GAUGE_LED_FUEL_INDICES[] = {79, 80, 78, 81, 77, 82, 76, 83, 75, 84, 74, 85, 73, 86, 72, 87};
const int GAUGE_LED_FUEL_INDEX_COUNT = 16;
const int GAUGE_LED_OXI_INDICES[] = {95, 96, 94, 97, 93, 98, 92, 99, 91, 100, 90, 101, 89, 102, 88, 103};
const int GAUGE_LED_OXI_INDEX_COUNT = 16;
const int GAUGE_LED_SF_INDICES[] = {64, 65, 66, 67, 68, 69, 70, 71};
const int GAUGE_LED_SF_INDEX_COUNT = 8;
const int GAUGE_LED_XE_INDICES[] = {112, 113, 114, 115, 116, 117, 118, 119};
const int GAUGE_LED_XE_INDEX_COUNT = 8;
const int GAUGE_LED_MONO_INDICES[] = {111, 110, 109, 108, 107, 106, 105, 104};
const int GAUGE_LED_MONO_INDEX_COUNT = 8;
const int GAUGE_LED_ELECTRIC_INDICES[] = {127, 126, 125, 124, 123, 122, 121, 120};
const int GAUGE_LED_ELECTRIC_INDEX_COUNT = 8;

// -- LED Status Panel
#define STATUS_LED_ARRAY_DATA_PIN 15
#define STATUS_LED_ARRAY_COUNT 128
// end with -1 to indicate end of array for update_status_led()
const int STATUS_LED_MASTER_CAUTION[] = {56, 55, 40, 39, 24, 23, 8, 7,
                                         57, 54, 41, 38, 25, 22, 9, 6, -1};
const int STATUS_LED_LOW_FUEL[] = {58, 59, 53, 52, -1};
const int STATUS_LED_LOW_OXI[] = {42, 43, 37, 36, -1};
const int STATUS_LED_LOW_ELEC[] = {60, 61, 51, 50, -1};
const int STATUS_LED_HIGH_TEMP[] = {28, 29, 19, 18, -1};
const int STATUS_LED_HIGH_G_FORCE[] = {26, 27, 21, 20, -1};
const int STATUS_LED_LOW_MONO[] = {44, 45, 35, 34, -1};
const int STATUS_LED_SUB_ORBITAL[] = {10, 11, 5, 4, -1};
const int STATUS_LED_COMM_SIGNAL[] = {12, 3, 13, 2, -1};
const int STATUS_LED_NODE_EXEC[] = {62, 63, 49, 48, -1};
const int STATUS_LED_SAS_STABLIZE[] = {33, -1};
const int STATUS_LED_SAS_MANUVER[] = {32, -1};
const int STATUS_LED_SAS_TARGET[] = {30, -1};
const int STATUS_LED_SAS_ANTITARGET[] = {31, -1};
const int STATUS_LED_SAS_PROGRADE[] = {17, -1};
const int STATUS_LED_SAS_RETROGRADE[] = {16, -1};
const int STATUS_LED_SAS_NORMAL[] = {1, -1};
const int STATUS_LED_SAS_ANTINORMAL[] = {0, -1};
const int STATUS_LED_SAS_RADIAL_IN[] = {14, -1};
const int STATUS_LED_SAS_RADIAL_OUT[] = {15, -1};
const int EMPTY_STATUS_LED[] = {-1};
const int STATUS_LED_MASTER_CAUTION_TRIGGERS[] = {58, 59, 53, 52, 42, 43, 37, 36, 60, 61, 51, 50, 44, 45, 35, 34, 28, 29, 19, 18, 26, 27, 21, 20, 10, 11, 5, 4};

#define SITUATION_LANDED 0
#define SITUATION_SPLASHED 1
#define SITUATION_PRELAUNCH 2
#define SITUATION_FLYING 3
// #define SITUATION_SUB_ORBITAL 4
#define SITUATION_SUB_ORBITAL 16 // docs are wrong?
#define SITUATION_REENTRY 8      // docs are wrong?
#define SITUATION_ORBITING 5
#define SITUATION_ESCAPING 6
#define SITUATION_DOCKED 7

#define SAS_MODE_OFF 255
#define SAS_MODE_STABILITYASSIST 0
#define SAS_MODE_PROGRADE 1
#define SAS_MODE_RETROGRADE 2
#define SAS_MODE_NORMAL 3
#define SAS_MODE_ANTINORMAL 4
#define SAS_MODE_RADIALIN 5
#define SAS_MODE_RADIALOUT 6
#define SAS_MODE_TARGET 7
#define SAS_MODE_ANTITARGET 8
#define SAS_MODE_MANEUVER 9

// -- Char LCD
const char CLEAR_LINE[] = "                    ";

// -- OLED
// #define OLED_RESET_MS 5000

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
#define BUTTON_STAGE_PIN 16
#define SWITCH_ABORT_PIN 17
#define SWITCH_GEAR_PIN 18
#define SWITCH_LIGHTS_PIN 19
#define SWITCH_RCS_PIN 20
#define SWITCH_SAS_PIN 21
#define SWITCH_BRAKES_PIN 22
#define SWITCH_REPORT_STAGE_AMOUNTS_PIN 11
#define SWITCH_SCREEN_MODE_UP_PIN 12
#define SWITCH_SCREEN_MODE_DOWN_PIN 13

//---------------//
// -- Globals -- //
//---------------//
bool MASTER_CAUTION = false;
int STATUS_LED_STATE[STATUS_LED_ARRAY_COUNT];
int LOOP_COUNTER = 0;
bool REPORT_STAGE_AMOUNTS = false;
int CURRENT_SCREEN_MODE = 0; // 0 = orbit, 1 = manuver, 2 = target

//--------------------//
// -- Constructors -- //
//--------------------//
// -- Simpit
KerbalSimpit mySimpit(Serial);

// -- OLED
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// - Status LED Panel
Adafruit_NeoPixel STATUS_LED_ARRAY(STATUS_LED_ARRAY_COUNT, STATUS_LED_ARRAY_DATA_PIN, NEO_GRB + NEO_KHZ800);

// -- LED Gauges
// Adafruit_NeoPixel GAUGE_LED_ARRAY(GAUGE_LED_ARRAY_COUNT, GAUGE_LED_ARRAY_DATA_PIN, NEO_GRB + NEO_KHZ800); // now part of the same led string as Status

// -- Char LCD
LiquidCrystal_I2C CHAR_LCD(0x3f, 20, 4);

// -- LED Numeric Displays
Adafruit_AlphaNum4 LED_ALT_0 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_ALT_1 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_ALT_2 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_SPD_0 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_SPD_1 = Adafruit_AlphaNum4();
Adafruit_AlphaNum4 LED_SPD_2 = Adafruit_AlphaNum4();

// -- Buttons & Switches
Adafruit_Debounce SWITCH_GEAR(SWITCH_GEAR_PIN, LOW);
Adafruit_Debounce SWITCH_LIGHTS(SWITCH_LIGHTS_PIN, LOW);
Adafruit_Debounce SWITCH_RCS(SWITCH_RCS_PIN, LOW);
Adafruit_Debounce SWITCH_SAS(SWITCH_SAS_PIN, LOW);
Adafruit_Debounce SWITCH_BRAKES(SWITCH_BRAKES_PIN, LOW);
Adafruit_Debounce SWITCH_REPORT_STAGE_AMOUNTS(SWITCH_REPORT_STAGE_AMOUNTS_PIN, LOW);
Adafruit_Debounce SWITCH_SCREEN_MODE_UP(SWITCH_SCREEN_MODE_UP_PIN, LOW);
Adafruit_Debounce SWITCH_SCREEN_MODE_DOWN(SWITCH_SCREEN_MODE_DOWN_PIN, LOW);

//-------------------------//
// -- Runtime Constants -- //
//-------------------------//
// -- LED Colors
#define RED STATUS_LED_ARRAY.Color(255, 0, 0)
#define GREEN STATUS_LED_ARRAY.Color(0, 255, 0)
#define BLUE STATUS_LED_ARRAY.Color(0, 0, 255)
#define CYAN STATUS_LED_ARRAY.Color(0, 255, 255)
#define PURPLE STATUS_LED_ARRAY.Color(255, 0, 255)
#define YELLOW STATUS_LED_ARRAY.Color(255, 255, 0)
#define ORANGE STATUS_LED_ARRAY.Color(255, 128, 0)
#define WHITE STATUS_LED_ARRAY.Color(255, 255, 255)
#define BLACK STATUS_LED_ARRAY.Color(0, 0, 0)
// #define WHITE_W STATUS_LED_ARRAY.Color(255, 0, 0, 0)
// #define BLACK_W STATUS_LED_ARRAY.Color(0, 0, 0, 0)

//-----------------------//
// -- Runtime Globals -- //
//-----------------------//
Adafruit_AlphaNum4 ALTITUDE_LED_DISPLAYS[] = {LED_ALT_0, LED_ALT_1, LED_ALT_2};
Adafruit_AlphaNum4 SPEED_LED_DISPLAYS[] = {LED_SPD_0, LED_SPD_1, LED_SPD_2};

//-----------------//
// -- Functions -- //
//-----------------//
// -- Prepare
void u8g2_prepare(void)
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  // u8g2.setFont(u8g2_font_guildenstern_nbp_t_all);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
  // u8g2.setFlipMode(0);

  u8g2.clearBuffer();
  u8g2.drawStr(0, 0, "Waiting for KSP");
  u8g2.sendBuffer();
}

void char_lcd_prepare()
{
  CHAR_LCD.init();
  CHAR_LCD.backlight();
  CHAR_LCD.clear();
  CHAR_LCD.setCursor(0, 0);
  CHAR_LCD.print("    Waiting for");
  CHAR_LCD.setCursor(0, 1);
  CHAR_LCD.print("    KerbalSimpit");
  CHAR_LCD.setCursor(0, 2);
  CHAR_LCD.print("     Connection");
  CHAR_LCD.setCursor(0, 3);
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
  for (int i = 0; i < 4; i++)
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

void prepare_buttons()
{
  SWITCH_GEAR.begin();
  SWITCH_LIGHTS.begin();
  SWITCH_RCS.begin();
  SWITCH_SAS.begin();
  SWITCH_BRAKES.begin();
}

// -- Helpers
void char_lcd_clear_line(int line)
{
  CHAR_LCD.setCursor(0, line);
  CHAR_LCD.print(CLEAR_LINE);
  CHAR_LCD.setCursor(0, line);
}

void update_led_segment_display(Adafruit_AlphaNum4 display, char *value)
{
  for (int i = 0; i < 4; i++)
  {
    display.writeDigitAscii(i, value[i]);
  }
  display.writeDisplay();
  // CHAR_LCD.setCursor(0, 3);
  // CHAR_LCD.print(value);
  // CHAR_LCD.display();
}

char *format_distance_value(float distance, char *buffer)
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

char *duration_in_seconds_to_dhms_string(int duration, char *buffer)
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
  else // something broke, fall back to raw duration
  {
    sprintf(buffer, "%ds", int(duration));
  }

  return buffer;
}

void update_altiude(float alt)
{
  char alt_chars[13];
  char mode[3];
  strcpy(mode, " T");
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

void update_velocity(float vel)
{
  char vel_chars[13];
  char mode[3];
  char unit[3];
  strcpy(mode, " O");
  strcpy(unit, "Ms");
  sprintf(vel_chars, " %6d %s%s", int(vel), unit, mode);

  LED_SPD_0.writeDigitAscii(0, vel_chars[0]);
  LED_SPD_0.writeDigitAscii(1, vel_chars[1]);
  LED_SPD_0.writeDigitAscii(2, vel_chars[2]);
  LED_SPD_0.writeDigitAscii(3, vel_chars[3]);
  LED_SPD_1.writeDigitAscii(0, vel_chars[4]);
  LED_SPD_1.writeDigitAscii(1, vel_chars[5]);
  LED_SPD_1.writeDigitAscii(2, vel_chars[6]);
  LED_SPD_1.writeDigitAscii(3, vel_chars[7]);
  LED_SPD_2.writeDigitAscii(0, vel_chars[8], true);
  LED_SPD_2.writeDigitAscii(1, vel_chars[9]);
  LED_SPD_2.writeDigitAscii(2, vel_chars[10]);
  LED_SPD_2.writeDigitAscii(3, vel_chars[11]);
}

void update_status_led(const int *indexes, uint32_t color)
{
  for (int i = 0; i <= 16; i++)
  {
    if (indexes[i] == -1)
    {
      break;
    }
    STATUS_LED_ARRAY.setPixelColor(indexes[i], color);
    STATUS_LED_STATE[indexes[i]] = color;
  }
}

// -- Handle Simpit Messages
void update_apoapsis(float apo)
{
  char label[] = "Appoapsis:";
  char apo_val[11];
  char buffer[strlen(label) + strlen(apo_val) + 2];
  format_distance_value(apo, apo_val);
  sprintf(buffer, "%s %s", label, apo_val);

  u8g2.setDrawColor(0);
  u8g2.drawBox(3, 0, u8g2.getDisplayWidth(), 9);
  u8g2.setDrawColor(1);
  u8g2.drawStr(3, 1, buffer);
  u8g2.drawLine(0, 0, u8g2.getDisplayWidth(), 0);
  u8g2.drawLine(0, 20, u8g2.getDisplayWidth(), 20);
}

void update_apoapsis_time(int apo)
{
  char buffer[22];
  char final_buffer[22];
  duration_in_seconds_to_dhms_string(apo, buffer);
  sprintf(final_buffer, "%21s", buffer);
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 11, u8g2.getDisplayWidth(), 9);
  u8g2.setDrawColor(1);
  u8g2.drawStr(0, 11, final_buffer);
}

void update_periapsis(float peri)
{
  char label[] = "Periapsis:";
  char peri_val[11];
  char buffer[strlen(label) + strlen(peri_val) + 2];
  format_distance_value(peri, peri_val);
  sprintf(buffer, "%s %s", label, peri_val);

  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 22, u8g2.getDisplayWidth(), 9);
  u8g2.setDrawColor(1);
  u8g2.drawStr(3, 22, buffer);
  u8g2.drawLine(0, 40, u8g2.getDisplayWidth(), 40);
}

void update_periapsis_time(int peri)
{
  char buffer[22];
  char final_buffer[22];
  duration_in_seconds_to_dhms_string(peri, buffer);
  sprintf(final_buffer, "%21s", buffer);

  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 31, u8g2.getDisplayWidth(), 9);
  u8g2.setDrawColor(1);
  u8g2.drawStr(3, 31, final_buffer);
}

void update_maneuver(int time_to_next, float delta_v_next, int duration_next)
{
  char time_to_mnv_buffer[22];
  char final_buffer[22];
  char duration_buffer[11];
  duration_in_seconds_to_dhms_string(time_to_next, time_to_mnv_buffer);
  duration_in_seconds_to_dhms_string(duration_next, duration_buffer);
  if (delta_v_next > 0.0)
  {
    sprintf(final_buffer, "MNV|%s|%dΔ|%s", time_to_mnv_buffer, int(delta_v_next), duration_buffer);
  }
  else
  {
    sprintf(final_buffer, "No Maneuver Planned");
  }
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 53, u8g2.getDisplayWidth(), 9);
  u8g2.setDrawColor(1);
  u8g2.drawStr(0, 53, final_buffer);
}

void update_led_gauge(float total, float available, const int indices[], const int index_count, uint32_t on_color, uint32_t off_color, uint32_t empty_color, const int low_status_led_idx[], uint32_t low_status_color)
{
  float resource_amount_percent = available / total * 100;
  int resource_amount_led_count = map(available, 0, total, 0, index_count);
  int gauge_start_idx = indices[0];

  for (int i = 0; i < index_count; i++)
  {
    int this_led_idx = indices[i];
    if (i < resource_amount_led_count)
    {
      STATUS_LED_ARRAY.setPixelColor(this_led_idx, on_color);
    }
    else
    {
      STATUS_LED_ARRAY.setPixelColor(this_led_idx, off_color);
    }
  }

  if (resource_amount_percent <= 15.0)
  {
    update_status_led(low_status_led_idx, low_status_color);
  }
  else if (resource_amount_percent < 1.0)
  {
    STATUS_LED_ARRAY.setPixelColor(gauge_start_idx, empty_color);
    update_status_led(low_status_led_idx, RED);
  }
  else
  {
    update_status_led(low_status_led_idx, BLACK);
  }
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
    update_status_led(STATUS_LED_HIGH_G_FORCE, BLACK);
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
    update_status_led(STATUS_LED_HIGH_TEMP, BLACK);
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
    update_status_led(STATUS_LED_SUB_ORBITAL, BLACK);
  }
}

void check_master_caution()
{
  update_status_led(STATUS_LED_MASTER_CAUTION, BLACK);
  MASTER_CAUTION = false;

  for (int i : STATUS_LED_MASTER_CAUTION_TRIGGERS)
  {
    if (STATUS_LED_STATE[i] != BLACK)
    {
      MASTER_CAUTION = true;
      update_status_led(STATUS_LED_MASTER_CAUTION, ORANGE);
      break;
    }
  }
}

void check_buttons()
{
  SWITCH_GEAR.update();
  if (SWITCH_GEAR.justPressed())
  {
    mySimpit.activateAction(GEAR_ACTION);
  }
  else if (SWITCH_GEAR.justReleased())
  {
    mySimpit.deactivateAction(GEAR_ACTION);
  }

  SWITCH_LIGHTS.update();
  if (SWITCH_LIGHTS.justPressed())
  {
    mySimpit.activateAction(LIGHT_ACTION);
  }
  else if (SWITCH_LIGHTS.justReleased())
  {
    mySimpit.deactivateAction(LIGHT_ACTION);
  }

  SWITCH_RCS.update();
  if (SWITCH_RCS.justPressed())
  {
    mySimpit.activateAction(RCS_ACTION);
  }
  else if (SWITCH_RCS.justReleased())
  {
    mySimpit.deactivateAction(RCS_ACTION);
  }

  SWITCH_SAS.update();
  if (SWITCH_SAS.justPressed())
  {
    mySimpit.activateAction(SAS_ACTION);
  }
  else if (SWITCH_SAS.justReleased())
  {
    mySimpit.deactivateAction(SAS_ACTION);
  }

  SWITCH_BRAKES.update();
  if (SWITCH_BRAKES.justPressed())
  {
    mySimpit.activateAction(BRAKES_ACTION);
  }
  else if (SWITCH_BRAKES.justReleased())
  {
    mySimpit.deactivateAction(BRAKES_ACTION);
  }

  SWITCH_REPORT_STAGE_AMOUNTS.update();
  if (SWITCH_REPORT_STAGE_AMOUNTS.justPressed())
  {
    REPORT_STAGE_AMOUNTS = false;
    mySimpit.printToKSP("Stage Amount Reporting OFF", PRINT_TO_SCREEN);
    zero_led_gauges();
    request_resource_refresh();
  }
  else if (SWITCH_REPORT_STAGE_AMOUNTS.justReleased())
  {
    REPORT_STAGE_AMOUNTS = true;
    mySimpit.printToKSP("Stage Amount Reporting ON", PRINT_TO_SCREEN);
    zero_led_gauges();
    request_resource_refresh();
  }

  SWITCH_SCREEN_MODE_UP.update();
  if (SWITCH_SCREEN_MODE_UP.justReleased())
  {
    CURRENT_SCREEN_MODE = (CURRENT_SCREEN_MODE + 1) % 3;
  }

  SWITCH_SCREEN_MODE_DOWN.update();
  if (SWITCH_SCREEN_MODE_DOWN.justReleased())
  {
    CURRENT_SCREEN_MODE = (CURRENT_SCREEN_MODE - 1 + 3) % 3;
  }
}

void zero_led_gauges()
{
  update_led_gauge(1, 1, GAUGE_LED_FUEL_INDICES, GAUGE_LED_FUEL_INDEX_COUNT, BLACK, BLACK, BLACK, STATUS_LED_LOW_FUEL, BLACK);
  update_led_gauge(1, 1, GAUGE_LED_OXI_INDICES, GAUGE_LED_OXI_INDEX_COUNT, BLACK, BLACK, BLACK, STATUS_LED_LOW_OXI, BLACK);
  update_led_gauge(1, 1, GAUGE_LED_SF_INDICES, GAUGE_LED_SF_INDEX_COUNT, BLACK, BLACK, BLACK, STATUS_LED_LOW_FUEL, BLACK);
  update_led_gauge(1, 1, GAUGE_LED_XE_INDICES, GAUGE_LED_XE_INDEX_COUNT, BLACK, BLACK, BLACK, STATUS_LED_LOW_FUEL, BLACK);
  update_led_gauge(1, 1, GAUGE_LED_MONO_INDICES, GAUGE_LED_MONO_INDEX_COUNT, BLACK, BLACK, BLACK, STATUS_LED_LOW_MONO, BLACK);
  update_led_gauge(1, 1, GAUGE_LED_ELECTRIC_INDICES, GAUGE_LED_ELECTRIC_INDEX_COUNT, BLACK, BLACK, BLACK, STATUS_LED_LOW_ELEC, BLACK);
}

void request_resource_refresh()
{
  mySimpit.requestMessageOnChannel(ELECTRIC_MESSAGE);
  mySimpit.requestMessageOnChannel(MONO_MESSAGE);
  
  if (REPORT_STAGE_AMOUNTS)
  {
    mySimpit.requestMessageOnChannel(LF_STAGE_MESSAGE);
    mySimpit.requestMessageOnChannel(OX_STAGE_MESSAGE);
    mySimpit.requestMessageOnChannel(SF_STAGE_MESSAGE);
    mySimpit.requestMessageOnChannel(XENON_GAS_STAGE_MESSAGE);
  }
  else
  {
    mySimpit.requestMessageOnChannel(LF_MESSAGE);
    mySimpit.requestMessageOnChannel(OX_MESSAGE);
    mySimpit.requestMessageOnChannel(SF_MESSAGE);
    mySimpit.requestMessageOnChannel(XENON_GAS_MESSAGE);
  }
}

void update_sas_mode_status_led(int sas_mode)
{
  mySimpit.printToKSP(String(sas_mode), PRINT_TO_SCREEN);
  reset_sas_indicators();
  switch (sas_mode)
  {
    case SAS_MODE_OFF:
      break;
    case SAS_MODE_ANTINORMAL:
      update_status_led(STATUS_LED_SAS_ANTINORMAL, CYAN);
      break;
    case SAS_MODE_NORMAL:
      update_status_led(STATUS_LED_SAS_NORMAL, CYAN);
      break;
    case SAS_MODE_PROGRADE:
      update_status_led(STATUS_LED_SAS_PROGRADE, GREEN);
      break;
    case SAS_MODE_RETROGRADE:
      update_status_led(STATUS_LED_SAS_RETROGRADE, GREEN);
      break;
    case SAS_MODE_RADIALIN:
      update_status_led(STATUS_LED_SAS_RADIAL_IN, PURPLE);
      break;
    case SAS_MODE_RADIALOUT:
      update_status_led(STATUS_LED_SAS_RADIAL_OUT, PURPLE);
      break;
    case SAS_MODE_TARGET:
      update_status_led(STATUS_LED_SAS_TARGET, PURPLE);
      break;
    case SAS_MODE_ANTITARGET:
      update_status_led(STATUS_LED_SAS_ANTITARGET, PURPLE);
      break;
    case SAS_MODE_MANEUVER:
      update_status_led(STATUS_LED_SAS_MANUVER, BLUE);
      break;
    case SAS_MODE_STABILITYASSIST:
      update_status_led(STATUS_LED_SAS_STABLIZE, WHITE);
      break;
    default:
      break;
  }
}

void reset_sas_indicators()
{
  update_status_led(STATUS_LED_SAS_STABLIZE, BLACK);
  update_status_led(STATUS_LED_SAS_PROGRADE, BLACK);
  update_status_led(STATUS_LED_SAS_RETROGRADE, BLACK);
  update_status_led(STATUS_LED_SAS_NORMAL, BLACK);
  update_status_led(STATUS_LED_SAS_ANTINORMAL, BLACK);
  update_status_led(STATUS_LED_SAS_RADIAL_IN, BLACK);
  update_status_led(STATUS_LED_SAS_RADIAL_OUT, BLACK);
  update_status_led(STATUS_LED_SAS_TARGET, BLACK);
  update_status_led(STATUS_LED_SAS_ANTITARGET, BLACK);
  update_status_led(STATUS_LED_SAS_MANUVER, BLACK);
}

// -- Main Simpit Message Handler
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
  case VELOCITY_MESSAGE:
    if (msgSize == sizeof(velocityMessage))
    {
      velocityMessage myVelocity;
      myVelocity = parseMessage<velocityMessage>(message);
      update_velocity(myVelocity.orbital);
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
    if (msgSize == sizeof(resourceMessage) && REPORT_STAGE_AMOUNTS == false)
    {
      resourceMessage myLFuel;
      myLFuel = parseMessage<resourceMessage>(message);
      update_led_gauge(myLFuel.total, myLFuel.available, GAUGE_LED_FUEL_INDICES, GAUGE_LED_FUEL_INDEX_COUNT, YELLOW, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
    }
    break;
  case LF_STAGE_MESSAGE:
    if (msgSize == sizeof(resourceMessage) && REPORT_STAGE_AMOUNTS == true)
    {
      resourceMessage myLFuel;
      myLFuel = parseMessage<resourceMessage>(message);
      update_led_gauge(myLFuel.total, myLFuel.available, GAUGE_LED_FUEL_INDICES, GAUGE_LED_FUEL_INDEX_COUNT, YELLOW, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
    }
    break;
  case OX_MESSAGE:
    if (msgSize == sizeof(resourceMessage) && REPORT_STAGE_AMOUNTS == false)
    {
      resourceMessage myOxi;
      myOxi = parseMessage<resourceMessage>(message);
      update_led_gauge(myOxi.total, myOxi.available, GAUGE_LED_OXI_INDICES, GAUGE_LED_OXI_INDEX_COUNT, CYAN, BLACK, RED, STATUS_LED_LOW_OXI, YELLOW);
    }
    break;
  case OX_STAGE_MESSAGE:
    if (msgSize == sizeof(resourceMessage) && REPORT_STAGE_AMOUNTS == true)
    {
      resourceMessage myOxi;
      myOxi = parseMessage<resourceMessage>(message);
      update_led_gauge(myOxi.total, myOxi.available, GAUGE_LED_OXI_INDICES, GAUGE_LED_OXI_INDEX_COUNT, CYAN, BLACK, RED, STATUS_LED_LOW_OXI, YELLOW);
    }
    break;
  case SF_MESSAGE:
    if (msgSize == sizeof(resourceMessage) && REPORT_STAGE_AMOUNTS == false)
    {
      resourceMessage mySFuel;
      mySFuel = parseMessage<resourceMessage>(message);
      update_led_gauge(mySFuel.total, mySFuel.available, GAUGE_LED_SF_INDICES, GAUGE_LED_SF_INDEX_COUNT, RED, BLACK, ORANGE, STATUS_LED_LOW_FUEL, YELLOW);
    }
    break;
  case SF_STAGE_MESSAGE:
    if (msgSize == sizeof(resourceMessage) && REPORT_STAGE_AMOUNTS == true)
    {
      resourceMessage mySFuel;
      mySFuel = parseMessage<resourceMessage>(message);
      update_led_gauge(mySFuel.total, mySFuel.available, GAUGE_LED_SF_INDICES, GAUGE_LED_SF_INDEX_COUNT, RED, BLACK, ORANGE, STATUS_LED_LOW_FUEL, YELLOW);
    }
    break;
  case MONO_MESSAGE:
    if (msgSize == sizeof(resourceMessage))
    {
      resourceMessage myMono;
      myMono = parseMessage<resourceMessage>(message);
      update_led_gauge(myMono.total, myMono.available, GAUGE_LED_MONO_INDICES, GAUGE_LED_MONO_INDEX_COUNT, WHITE, BLACK, RED, STATUS_LED_LOW_MONO, YELLOW);
    }
    break;
  case ELECTRIC_MESSAGE:
    if (msgSize == sizeof(resourceMessage))
    {
      resourceMessage myElectric;
      myElectric = parseMessage<resourceMessage>(message);
      update_led_gauge(myElectric.total, myElectric.available, GAUGE_LED_ELECTRIC_INDICES, GAUGE_LED_ELECTRIC_INDEX_COUNT, GREEN, BLACK, RED, STATUS_LED_LOW_ELEC, YELLOW);
    }
    break;
  case XENON_GAS_MESSAGE:
    if (msgSize == sizeof(resourceMessage))
    {
      resourceMessage myXenon;
      myXenon = parseMessage<resourceMessage>(message);
      update_led_gauge(myXenon.total, myXenon.available, GAUGE_LED_XE_INDICES, GAUGE_LED_XE_INDEX_COUNT, PURPLE, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
    }
    break;
  case XENON_GAS_STAGE_MESSAGE:
    if (msgSize == sizeof(resourceMessage))
    {
      resourceMessage myXenon;
      myXenon = parseMessage<resourceMessage>(message);
      update_led_gauge(myXenon.total, myXenon.available, GAUGE_LED_XE_INDICES, GAUGE_LED_XE_INDEX_COUNT, PURPLE, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
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
  case SAS_MODE_INFO_MESSAGE:
    if (msgSize == sizeof(SASInfoMessage))
    {
      SASInfoMessage mySasModeInfo;
      mySasModeInfo = parseMessage<SASInfoMessage>(message);
      update_sas_mode_status_led(mySasModeInfo.currentSASMode);
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
      update_maneuver(myManeuver.timeToNextManeuver, myManeuver.deltaVNextManeuver, myManeuver.durationNextManeuver);
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

// -- Tests
void test_led_gauge()
{
  update_led_gauge(1, 1, GAUGE_LED_FUEL_INDICES, GAUGE_LED_FUEL_INDEX_COUNT, YELLOW, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_OXI_INDICES, GAUGE_LED_OXI_INDEX_COUNT, BLUE, BLACK, RED, STATUS_LED_LOW_OXI, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_SF_INDICES, GAUGE_LED_SF_INDEX_COUNT, RED, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_XE_INDICES, GAUGE_LED_XE_INDEX_COUNT, CYAN, BLACK, RED, STATUS_LED_LOW_FUEL, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_MONO_INDICES, GAUGE_LED_MONO_INDEX_COUNT, WHITE, BLACK, RED, STATUS_LED_LOW_MONO, YELLOW);
  update_led_gauge(1, 1, GAUGE_LED_ELECTRIC_INDICES, GAUGE_LED_ELECTRIC_INDEX_COUNT, GREEN, BLACK, GREEN, STATUS_LED_LOW_ELEC, YELLOW);
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
  // GAUGE_LED_ARRAY.begin();
  STATUS_LED_ARRAY.begin();
  // GAUGE_LED_ARRAY.setBrightness(255);
  STATUS_LED_ARRAY.setBrightness(255);
  test_status_led();
  test_led_gauge();
  // GAUGE_LED_ARRAY.show();
  STATUS_LED_ARRAY.show();

  // -- Lockout Switch Setup
  pinMode(LOCKOUT_SWITCH_PIN, INPUT_PULLUP);

  // -- Button Setup
  prepare_buttons();

  // -- OLED setup
  u8g2_prepare();

  // -- Char LCD setup
  char_lcd_prepare();

  // -- LED Segment setup
  led_segment_prepare();

  // -- Simpit setup
  Serial.begin(115200);
  int connection_attempts = 0;

  update_status_led(STATUS_LED_LOW_FUEL, YELLOW);
  update_status_led(STATUS_LED_LOW_ELEC, YELLOW);
  update_status_led(STATUS_LED_LOW_OXI, YELLOW);
  update_status_led(STATUS_LED_LOW_MONO, YELLOW);

  while (!mySimpit.init())
  {
    u8g2.clearBuffer();
    u8g2.sendF("ca", 0xd5, 0xF0);
    if (connection_attempts % 2 == 0)
    {
      u8g2.drawStr(0, 0, "Waiting for KSP...");
      update_status_led(STATUS_LED_MASTER_CAUTION, ORANGE);
      update_status_led(STATUS_LED_COMM_SIGNAL, RED);
      CHAR_LCD.print(".");
    }
    else
    {
      u8g2.drawStr(0, 0, "Waiting for KSP.. ");
      update_status_led(STATUS_LED_MASTER_CAUTION, BLACK);
      update_status_led(STATUS_LED_COMM_SIGNAL, BLACK);
    }
    u8g2.sendBuffer();
    // GAUGE_LED_ARRAY.show();
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
  mySimpit.registerChannel(VELOCITY_MESSAGE);
  mySimpit.registerChannel(APSIDES_MESSAGE);
  mySimpit.registerChannel(LF_MESSAGE);
  mySimpit.registerChannel(LF_STAGE_MESSAGE);
  mySimpit.registerChannel(SF_MESSAGE);
  mySimpit.registerChannel(SF_STAGE_MESSAGE);
  mySimpit.registerChannel(OX_MESSAGE);
  mySimpit.registerChannel(OX_STAGE_MESSAGE);
  mySimpit.registerChannel(XENON_GAS_MESSAGE);
  mySimpit.registerChannel(XENON_GAS_STAGE_MESSAGE);
  mySimpit.registerChannel(MONO_MESSAGE);
  mySimpit.registerChannel(ELECTRIC_MESSAGE);
  mySimpit.registerChannel(APSIDESTIME_MESSAGE);
  mySimpit.registerChannel(MANEUVER_MESSAGE);
  mySimpit.registerChannel(FLIGHT_STATUS_MESSAGE);
  mySimpit.registerChannel(SAS_MODE_INFO_MESSAGE);
  mySimpit.registerChannel(TEMP_LIMIT_MESSAGE);
  mySimpit.registerChannel(AIRSPEED_MESSAGE);
  mySimpit.inboundHandler(Handle_Simpit_Message);

  // GAUGE_LED_ARRAY.fill(BLACK, 0, GAUGE_LED_ARRAY_COUNT);
  STATUS_LED_ARRAY.fill(BLACK, 0, STATUS_LED_ARRAY_COUNT);
  CHAR_LCD.clear();
  u8g2.clearBuffer();
}

//------------//
// -- Loop -- //
//------------//
void loop(void)
{
  LOOP_COUNTER++;

  // Reset OLED frame buffer periodically
  // if (LOOP_COUNTER * LOOP_DELAY_MS / OLED_RESET_MS == 0 || LOOP_COUNTER > 100)
  // {
  //   u8g2.clearBuffer();
  //   LOOP_COUNTER = 0;
  // }
  // -- Fetch updates
  mySimpit.update();
  check_master_caution();
  check_buttons();

  // -- Display updates
  u8g2.sendBuffer();
  // GAUGE_LED_ARRAY.show(); // now part of the same LED string as Status
  STATUS_LED_ARRAY.show();
  LED_ALT_0.writeDisplay();
  LED_ALT_1.writeDisplay();
  LED_ALT_2.writeDisplay();
  LED_SPD_0.writeDisplay();
  LED_SPD_1.writeDisplay();
  LED_SPD_2.writeDisplay();

  //-- Sleep before next update
  delay(LOOP_DELAY_MS);
}
