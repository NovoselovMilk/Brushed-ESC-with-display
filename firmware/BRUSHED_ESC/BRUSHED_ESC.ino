/*
    ESC BRUSHED OLED CONTROLLER (driver TA6586)
    Created by Novosleov https://github.com/NovoselovMilk/Brushed-ESC-with-display
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <DNSServer.h>
#include <EEPROM.h>

#include <GyverOLED.h>
GyverOLED<SSD1306_128x32, OLED_BUFFER> oled;

// #define DEBUG

#define PIN_PWM_READ 13 
#define PIN_FWD 4
#define PIN_BKWD 5
#define PIN_LED 2
#define PIN_BRAKE_LIGHT 16
#define PIN_REVERSE_LIGHT 15
#define PIN_OLED_SDA 12
#define PIN_OLED_SCL 14

#define USE_SCREEN
#define USE_BAT_CONTROL
#define USE_ESC
// #define INVERT_LIGHT

#define PWM_MIN 988
#define PWM_MAX 2012
#define PWM_MIN_WIDE 800
#define PWM_MAX_WIDE 2200
#define START_LOW_PWM 1400
#define START_HI_PWM 1600

#define TIME_WIFI_ENABLE 60000
#define DELAY_FOR_BREAK 500

#define STD_PWM 0
#define WIDE_PWM 1
#define PERCENT_OF_BUTE_60 155
#define PERCENT_OF_BUTE_80 205
#define PERCENT_OF_BUTE_100 255

#define EE_INIT 0xFF
#define EE_ADDR_INIT 0
#define EE_ADDR_MAX_SPEED 1
#define EE_ADDR_PWM_MODE 2
#define EE_ADDR_USE_BAT 3
#define EE_ADDR_MIN_VOLT_CELL 4
#define EE_ADDR_MAX_VOLT_CELL 8
#define EE_ADDR_BAT_TYPE 12
#define EE_ADDR_VOLT_CAL 13
#define EE_ADDR_LED_ENABLE 17
#define EE_ADDR_START_FORWARD_PWM 18
#define EE_ADDR_START_BACKWARD_PWM 22
#define EE_ADDR_RATE_BACKWARD 26
#define EE_ADDR_BRAKE_MODE 27

#define BAT_1S 1
#define BAT_2S 2
#define BAT_3S 3
#define BRAKE_OFF 0
#define BRAKE_ON 1
#define SWITCHABLE 2

#ifdef INVERT_LIGHT
#define ON_LIGHT LOW
#define OFF_LIGHT HIGH
#else
#define ON_LIGHT HIGH
#define OFF_LIGHT LOW
#endif

volatile uint32_t last_time;
uint32_t last_loop_time = millis();
uint32_t last_time_forward = millis();
uint32_t last_time_2 = millis();
uint32_t time_oled = millis();
uint32_t last_time_check_bat = millis();
uint32_t last_time_signal = millis();
uint32_t last_led_flash = millis();

volatile int16_t width_pwm = 0;

volatile bool state = false;

IRAM_ATTR void readPWM();
uint16_t expRunningAverage(float newVal);
void drawIcon7x7(byte index);
void drawBattery(int x, int y, uint8_t state);
void drawArcSpeed(uint8_t x, uint8_t y, uint8_t speed, uint8_t max_speed);
void checkBattery();
void printOled();
void printAnimation();
static uint16_t ICACHE_RAM_ATTR fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void ledFlash(uint8_t mode);

void handleRoot();
void handleSave();
String createInputField(String label, String name, String value, String type, String min, String max, String comment, String extra = "");
String createSelectField(String label, String name, String selectedValue, std::vector<String> options, String comment);
void saveSettings();
void loadSettings();

uint8_t max_speed_on_display = 50;
bool PWM_mode = WIDE_PWM;
bool use_bat_control = true;
float min_voltage_cell = 2.9;
float max_voltage_cell = 4.2;
uint8_t batt_type = BAT_2S;
uint8_t max_pwm_backward = PERCENT_OF_BUTE_80;
float volt_calibrate = 0.0;
bool led_enable = true;
uint8_t brake_mode = SWITCHABLE;

uint16_t max_pwm;
uint16_t min_pwm;
uint16_t start_forward_pwm = START_HI_PWM;
uint16_t start_backward_pwm = START_LOW_PWM;

bool bat_low = false;
uint8_t current_speed = 0;
uint8_t state_bat = 3;
float battery_voltage = max_voltage_cell * batt_type;

const char *ssid = "ESC_SETTINGS";
const char *password = "12345678";

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
DNSServer dnsServer;

const static uint8_t icons_7x7[][7] PROGMEM = {
    {0x04, 0x0c, 0x1c, 0x3c, 0x1c, 0x0c, 0x04}, // arrow icon
    {0x02, 0x09, 0x25, 0x15, 0x25, 0x09, 0x02}, // wifi icon
};

int16_t pwm_output;

bool set_brake = false;
bool flag_set_break = false;
bool braking = false;

bool wifiEnable = false;

uint16_t old_width_pwm;

uint16_t current_mode;

enum
{
  BACKWARD = 0,
  BRAKE,
  NO_PULSES,
  FORWARD,
  NO_PWM_INPUT,
  WIFI
};

void setup()
{
  EEPROM.begin(512);

  if (EEPROM.read(EE_ADDR_INIT) != EE_INIT)
  {
    saveSettings();
    EEPROM.write(EE_ADDR_INIT, EE_INIT);
  }
  else
    loadSettings();

  pinMode(PIN_PWM_READ, INPUT_PULLUP);
  pinMode(PIN_BRAKE_LIGHT, OUTPUT);
  pinMode(PIN_REVERSE_LIGHT, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_BRAKE_LIGHT, OFF_LIGHT);
  digitalWrite(PIN_REVERSE_LIGHT, OFF_LIGHT);
  digitalWrite(PIN_LED, HIGH);

#ifdef USE_SCREEN
  oled.init(PIN_OLED_SDA, PIN_OLED_SCL);
  Wire.setClock(800000L);
  oled.clear();
#endif

  current_mode = NO_PULSES;
  if (brake_mode == BRAKE_ON)
    set_brake = true;
  else if (brake_mode == BRAKE_OFF)
    set_brake = false;

#ifdef DEBUG
  Serial.begin(115200);
#endif
  pinMode(PIN_FWD, OUTPUT);
  pinMode(PIN_BKWD, OUTPUT);
  digitalWrite(PIN_FWD, LOW);
  digitalWrite(PIN_BKWD, LOW);

  analogWriteFreq(500);
  analogWriteResolution(8);
  delay(10);
  attachInterrupt(PIN_PWM_READ, readPWM, RISING);
  printAnimation();
  last_time = micros();
}

void loop()
{

#ifdef USE_ESC
  if ((millis() - last_loop_time) >= 10)
  {
    last_loop_time = millis();
    if (led_enable)
      ledFlash(current_mode);

    if (current_mode != NO_PWM_INPUT)
    {
      if (!bat_low)
        last_time_signal = millis();
    }
    if (width_pwm > 2500)
      width_pwm = old_width_pwm;

    old_width_pwm = width_pwm;
    if (width_pwm > max_pwm)
      width_pwm = max_pwm;

    if (width_pwm == 0)
    {
      current_mode = NO_PULSES;
    }
    else if ((width_pwm > start_backward_pwm) && (width_pwm < start_forward_pwm))
    {
      braking = false;
      if (set_brake)
        current_mode = BRAKE;
      else
        current_mode = NO_PULSES;

      last_time_2 = millis();
    }

    else if (width_pwm >= start_forward_pwm)
    {
      current_mode = FORWARD;
      last_time_forward = millis();
      flag_set_break = false;
    }

    else if (width_pwm <= start_backward_pwm && (millis() - last_time_forward) < DELAY_FOR_BREAK)
    {
      current_mode = BRAKE;
      braking = true;
    }
    else if (width_pwm <= start_backward_pwm && ((millis() - last_time_forward) >= DELAY_FOR_BREAK) && !braking)
    {
      current_mode = BACKWARD;
    }

    if ((micros() - last_time) >= 25000) // Режим NO_PWM_INPUT, если пропал PWM сигнал
      current_mode = NO_PWM_INPUT;

    if (brake_mode == SWITCHABLE && width_pwm < 1200 && (millis() - last_time_2) > 3000 && !flag_set_break && current_mode == BRAKE)
    {
      last_time_2 = millis();
      set_brake = !set_brake;
      flag_set_break = true;
    }

    if (bat_low)
      current_mode = NO_PULSES;

    switch (current_mode)
    {
    case BACKWARD:
    {
      pwm_output = map(width_pwm, min_pwm, start_backward_pwm, ((~max_pwm_backward) & 0xFF), 255);
      if (pwm_output < 0)
        pwm_output = 0;
      if (pwm_output > 255)
        pwm_output = 255;

#ifdef DEBUG
      Serial.print("BACKWARD\t");
      Serial.print(pwm_output);
      Serial.print("\t");
      Serial.println(width_pwm);
#endif
      // analogWrite(PIN_FWD, 1023);
      digitalWrite(PIN_FWD, HIGH);
      analogWrite(PIN_BKWD, pwm_output);

      digitalWrite(PIN_REVERSE_LIGHT, ON_LIGHT);
      digitalWrite(PIN_BRAKE_LIGHT, OFF_LIGHT);

      break;
    }

    case BRAKE:
    {
#ifdef DEBUG
      Serial.print("BRAKE ");
      Serial.print("\t\t");
      Serial.println(width_pwm);
#endif
      // analogWrite(PIN_BKWD, 1023);
      // analogWrite(PIN_FWD, 1023);
      digitalWrite(PIN_FWD, HIGH);
      digitalWrite(PIN_BKWD, HIGH);

      digitalWrite(PIN_REVERSE_LIGHT, OFF_LIGHT);
      digitalWrite(PIN_BRAKE_LIGHT, ON_LIGHT);

      break;
    }

    case NO_PULSES:
    {
#ifdef DEBUG
      Serial.print("NO_PULSES\t ");
      Serial.print("\t\t");
      Serial.println(width_pwm);
#endif
      digitalWrite(PIN_BKWD, LOW);
      digitalWrite(PIN_FWD, LOW);

      digitalWrite(PIN_REVERSE_LIGHT, OFF_LIGHT);
      digitalWrite(PIN_BRAKE_LIGHT, OFF_LIGHT);

      break;
    }

    case FORWARD:
    {
      if (width_pwm > max_pwm)
        width_pwm = max_pwm;
      pwm_output = map(width_pwm, start_forward_pwm, max_pwm, 0, 255);
      if (pwm_output < 0)
        pwm_output = 0;
      if (pwm_output > 255)
        pwm_output = 255;
      pwm_output = (~pwm_output) & 0xFF;

#ifdef DEBUG
      Serial.print("FORWARD\t");
      Serial.print("\t");
      Serial.println(width_pwm);

#endif
      // analogWrite(PIN_BKWD, 1023);
      digitalWrite(PIN_BKWD, HIGH);
      analogWrite(PIN_FWD, pwm_output);

      digitalWrite(PIN_REVERSE_LIGHT, OFF_LIGHT);
      digitalWrite(PIN_BRAKE_LIGHT, OFF_LIGHT);
      break;
    }
    case NO_PWM_INPUT:
    {
#ifdef DEBUG
      Serial.print("NO_PWM_INPUT\t ");
      Serial.print("\t\t");
      Serial.println(width_pwm);
#endif
      digitalWrite(PIN_BKWD, LOW);
      digitalWrite(PIN_FWD, LOW);

      digitalWrite(PIN_REVERSE_LIGHT, OFF_LIGHT);
      digitalWrite(PIN_BRAKE_LIGHT, OFF_LIGHT);

      break;
    }
    }
  }
#endif

#ifdef USE_SCREEN
  if (millis() - time_oled >= 50)
  {
    time_oled = millis();

    if (current_mode == FORWARD || current_mode == BACKWARD)
    {
      current_speed = map(((~pwm_output) & 0xFF), 0, 255, 0, max_speed_on_display);
    }
    else
      current_speed = 0;

    printOled();
  }
#endif

#ifdef USE_BAT_CONTROL
  if (use_bat_control && (millis() - last_time_check_bat) >= 2000)
  {
    last_time_check_bat = millis();
    checkBattery();
  }
#endif

  if (millis() - last_time_signal > TIME_WIFI_ENABLE)
  {
#ifdef DEBUG
    Serial.println("enable Wi-Fi");
#endif
    current_mode = WIFI;
    wifiEnable = true;
    printOled();

    WiFi.softAP(ssid, password);
    IPAddress myIP(192, 168, 2, 1);
    WiFi.softAPConfig(myIP, myIP, IPAddress(255, 255, 255, 0));
#ifdef DEBUG
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
#endif

    dnsServer.start(53, "*", WiFi.softAPIP());
    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    httpUpdater.setup(&server, "/update");
    server.onNotFound(handleRoot);
    server.begin();
    while (true)
    {
      dnsServer.processNextRequest();
      server.handleClient();
      ledFlash(current_mode);
    }
  }
}

float expRunningAverage(float newVal, float oldVal, float k)
{

  oldVal += (newVal - oldVal) * k;

  return oldVal;
}

IRAM_ATTR void readPWM()
{

  if (!state)
  {
    state = true;
    attachInterrupt(PIN_PWM_READ, readPWM, FALLING);
    last_time = micros();
  }
  else
  {
    state = false;
    attachInterrupt(PIN_PWM_READ, readPWM, RISING);
    width_pwm = micros() - last_time;
  }
}

void drawIcon7x7(byte index)
{
  size_t s = sizeof icons_7x7[index];
  for (unsigned int i = 0; i < s; i++)
  {
    oled.drawByte(pgm_read_byte(&(icons_7x7[index][i])));
  }
}

void drawBattery(int x, int y, uint8_t state)
{
  if (!bat_low)
  {
    oled.rect(3 + x, y, 8 + x, 2 + y);
    oled.rect(x, 2 + y, 12 + x, 26 + y, OLED_STROKE);

    if (state == 1 || state == 2 || state == 3)
    {
      oled.rect(3 + x, 19 + y, 8 + x, 23 + y);
    }
    if (state == 2 || state == 3)
    {
      oled.rect(3 + x, 12 + y, 8 + x, 16 + y);
    }
    if (state == 3)
    {
      oled.rect(3 + x, 5 + y, 8 + x, 9 + y);
    }
  }
  else
  {
    oled.setCursorXY(x, y);
    oled.print("bat");
    oled.setCursorXY(x, y + 15);
    oled.print("low");
  }
}

static uint16_t IRAM_ATTR fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return ((x - in_min) * (out_max - out_min) * 2 / (in_max - in_min) + out_min * 2 + 1) / 2;
}

void drawArcSpeed(uint8_t x, uint8_t y, uint8_t speed, uint8_t max_speed = 50)
{
  oled.circle(x, y, 24, OLED_STROKE);
  float sector = fmap(speed, 0, max_speed, 26, 68);
  sector /= 10;
  float start_angle = 2.6, end_angle = sector;
  int r = 23;
  for (float i = start_angle; i < end_angle; i = i + 0.05)
  {
    oled.dot(x + cos(i) * r, y + sin(i) * r);
  }
  for (float i = start_angle; i < end_angle; i = i + 0.05)
  {
    oled.dot(x + cos(i) * (r - 1), y + sin(i) * (r - 1));
  }
  for (float i = start_angle; i < end_angle; i = i + 0.05)
  {
    oled.dot(x + cos(i) * (r - 2), y + sin(i) * (r - 2));
  }
}

void checkBattery()
{
  float diff_voltage = (max_voltage_cell - min_voltage_cell) * batt_type;
  float min_voltage = min_voltage_cell * batt_type;

  float v_now = ((float)analogRead(A0) * 3.15 * 6 / 1024) + volt_calibrate;
  battery_voltage = expRunningAverage(v_now, battery_voltage, 0.8);
  if (battery_voltage > ((diff_voltage * 0.75) + min_voltage))
    state_bat = 3;
  else if (battery_voltage <= ((diff_voltage * 0.75) + min_voltage) && battery_voltage > ((diff_voltage * 0.5) + min_voltage))
    state_bat = 2;
  else if (battery_voltage <= ((diff_voltage * 0.5) + min_voltage) && battery_voltage > ((diff_voltage * 0.25) + min_voltage))
    state_bat = 1;
  else if (battery_voltage <= ((diff_voltage * 0.25) + min_voltage) && battery_voltage > min_voltage)
  {
    state_bat = 0;
    bat_low = false;
  }

  if (battery_voltage <= min_voltage)
    bat_low = true;

#ifdef DEBUG
  Serial.print("VOLTAGE\t");
  Serial.print("\t");
  Serial.println(battery_voltage);
#endif
}

void printOled()
{
  oled.clear();
  drawArcSpeed(64, 25, current_speed, max_speed_on_display);

  oled.home();
  oled.setScale(2);
  current_speed < 10 ? oled.setCursorXY(60, 16) : oled.setCursorXY(54, 16);
  oled.print(current_speed);

  oled.setCursorXY(100, 20);
  oled.setScale(1);
  oled.print("PRND");
  drawBattery(10, 5, state_bat);

  if (set_brake)
  {
    oled.setCursorXY(30, 7);
    oled.print("B");
  }

  switch (current_mode)
  {
  case BACKWARD:
    oled.setCursorXY(104, 10); // R
    break;
  case BRAKE:
    oled.setCursorXY(110, 10); // N
    break;
  case NO_PULSES:
    oled.setCursorXY(110, 10); // N
    break;
  case FORWARD:
    oled.setCursorXY(116, 10); // D
    break;
  case NO_PWM_INPUT:
    oled.setCursorXY(98, 10); // P
    break;
  case WIFI:
  {
    oled.setCursorXY(90, 3);
    drawIcon7x7(1);
    oled.setCursorXY(98, 10); // P
  }
  break;
  }
  drawIcon7x7(0);

  oled.update();
}

void printAnimation()
{
  for (uint8_t i = 0; i < 50; i += 2)
  {
    oled.clear();
    if (i == 12)
    {
      state_bat = 1;
      current_mode = NO_PWM_INPUT;
    }
    else if (i == 24)
    {
      state_bat = 2;
      current_mode = BACKWARD;
    }
    else if (i == 38)
    {
      state_bat = 3;
      current_mode = FORWARD;
    }

    drawArcSpeed(64, 25, i, 50);
    oled.home();
    oled.setScale(2);

    i < 10 ? oled.setCursorXY(60, 16) : oled.setCursorXY(54, 16);
    oled.print(i);

    oled.setCursorXY(100, 20);
    oled.setScale(1);
    oled.print("PRND");
    drawBattery(10, 5, state_bat);

    if (i > 25)
    {
      oled.setCursorXY(30, 7);
      oled.print("B");
      oled.setCursorXY(90, 3);
      drawIcon7x7(1);
    }

    switch (current_mode)
    {
    case BACKWARD:
      oled.setCursorXY(104, 10); // R
      break;
    case BRAKE:
      oled.setCursorXY(110, 10); // N
      break;
    case NO_PULSES:
      oled.setCursorXY(110, 10); // N
      break;
    case FORWARD:
      oled.setCursorXY(116, 10); // D
      break;
    case NO_PWM_INPUT:
      oled.setCursorXY(98, 10); // P
      break;
    }

    drawIcon7x7(0);
    oled.update();
    delay(5);
  }
  current_mode = NO_PWM_INPUT;

  for (uint8_t i = 50; i > 0; i -= 2)
  {
    oled.clear();
    drawArcSpeed(64, 25, i, 50);
    oled.home();
    oled.setScale(2);

    i < 10 ? oled.setCursorXY(60, 16) : oled.setCursorXY(54, 16);
    oled.print(i);
    drawBattery(10, 5, state_bat);

    oled.setCursorXY(100, 20);
    oled.setScale(1);
    oled.print("PRND");

    oled.setCursorXY(98, 10); // P

    drawIcon7x7(0);

    oled.update();
    delay(5);
  }
}

void handleRoot()
{
  String page = "<html><head><title>ESC Settings</title>";
  page += "<style>";
  page += "body { font-family: Arial, sans-serif; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; height: 100vh; background-color: #f4f4f9; }";
  page += ".container { width: 400px; max-height: 90vh; padding: 20px; background: white; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); border-radius: 8px; overflow-y: auto; }";
  page += "h1 { text-align: center; color: #333; }";
  page += "form div { margin-bottom: 15px; }";
  page += "label { display: block; margin-bottom: 5px; color: #333; }";
  page += "input, select { width: 100%; padding: 8px; box-sizing: border-box; border: 1px solid #ddd; border-radius: 4px; }";
  page += "input[type='submit'] { background-color: #4CAF50; color: white; border: none; cursor: pointer; padding: 10px; border-radius: 4px; }";
  page += "input[type='submit']:hover { background-color: #45a049; }";
  page += "span { display: block; margin-top: 5px; font-size: 12px; color: #666; }";
  page += "</style></head><body>";
  page += "<div class='container'>";
  page += "<h1>ESC Settings</h1>";
  page += "<form action='/save' method='POST'>";

  page += createInputField("Max speed OLED", "max_speed_on_display", String(max_speed_on_display), "number", "1", "99", "Maximum speed for OLED screen");
  page += createSelectField("LED", "led_enable", led_enable ? "On" : "Off", {"On", "Off"}, "On-board LED control");
  page += createSelectField("Brake (Auto Hold)", "brake_mode", brake_mode == BRAKE_ON ? "On" : (brake_mode == BRAKE_OFF ? "Off" : "Switchable"), {"On", "Off", "Switchable"}, "Brake control. When you turn on the !Swichable! mode, switching the brake mode is switched from the remote control by holding the brake for more than three seconds ");
  page += createSelectField("PWM mode", "PWM_mode", PWM_mode == STD_PWM ? "std" : "wide", {"std", "wide"}, "std - 988-2012us (ELRS), wide - 800-2200us (other)");
  page += createInputField("Start forward PWM", "start_forward_pwm", String(start_forward_pwm), "number", "1550", "1800", "PWM duration value from which the motor starts to rotate forward", "step='1'");
  page += createInputField("Start backward PWM", "start_backward_pwm", String(start_backward_pwm), "number", "1200", "1450", "PWM duration value from which the motor starts to rotate backward", "step='1'");
  page += createSelectField("Reverse power", "max_pwm_backward", max_pwm_backward == PERCENT_OF_BUTE_60 ? "60%" : (max_pwm_backward == PERCENT_OF_BUTE_80 ? "80%" : "100%"), {"60%", "80%", "100%"}, "Engine power when rotating backwards");
  page += createSelectField("Bat control", "use_bat_control", use_bat_control ? "On" : "Off", {"On", "Off"}, "Enable battery voltage monitoring");
  page += createInputField("Min cell voltage", "min_voltage_cell", String(min_voltage_cell), "number", "1", "20", "PWM value from which the motor starts to rotate", "step='0.1'");
  page += createInputField("Max cell voltage", "max_voltage_cell", String(max_voltage_cell), "number", "1", "20", "Maximum voltage per cell", "step='0.1'");
  page += createSelectField("Battery type", "batt_type", batt_type == BAT_1S ? "1S" : (batt_type == BAT_2S ? "2S" : "3S"), {"1S", "2S", "3S"}, "Number of cells connected in series in a battery");
  page += createInputField("Voltage calibration (optional)", "real_voltage", use_bat_control ? String(battery_voltage) : " ", "number", "0", "20", "Enter the exact current battery voltage here", "step='0.01'");

  page += "<br><input type='submit' value='Save and Reboot'></form>";
  page += "</div></body></html>";

  server.send(200, "text/html", page);
}

String createInputField(String label, String name, String value, String type, String min, String max, String comment, String extra)
{
  return "<div><label>" + label + "</label>"
                                  "<input type='" +
         type + "' name='" + name + "' value='" + value + "' min='" + min + "' max='" + max + "' " + extra + ">"
                                                                                                             "<span>" +
         comment + "</span></div>";
}

String createSelectField(String label, String name, String selectedValue, std::vector<String> options, String comment)
{
  String html = "<div><label>" + label + "</label><select name='" + name + "'>";
  for (String option : options)
  {
    html += "<option value='" + option + "'";
    if (option == selectedValue)
    {
      html += " selected";
    }
    html += ">" + option + "</option>";
  }
  html += "</select><span>" + comment + "</span></div>";
  return html;
}

void handleSave()
{
  float real_voltage;
  if (server.hasArg("max_speed_on_display"))
    max_speed_on_display = server.arg("max_speed_on_display").toInt();
  if (server.hasArg("led_enable"))
    led_enable = (server.arg("led_enable") == "On");
  if (server.hasArg("brake_mode"))
    brake_mode = server.arg("brake_mode") == "On" ? BRAKE_ON : (server.arg("brake_mode") == "Off" ? BRAKE_OFF : SWITCHABLE);
  if (server.hasArg("PWM_mode"))
    PWM_mode = (server.arg("PWM_mode") == "wide");
  if (server.hasArg("start_forward_pwm"))
    start_forward_pwm = server.arg("start_forward_pwm").toInt();
  if (server.hasArg("start_backward_pwm"))
    start_backward_pwm = server.arg("start_backward_pwm").toInt();
  if (server.hasArg("max_pwm_backward"))
    max_pwm_backward = server.arg("max_pwm_backward") == "60%" ? PERCENT_OF_BUTE_60 : (server.arg("max_pwm_backward") == "80%" ? PERCENT_OF_BUTE_80 : PERCENT_OF_BUTE_100);
  if (server.hasArg("use_bat_control"))
    use_bat_control = (server.arg("use_bat_control") == "On");
  if (server.hasArg("min_voltage_cell"))
    min_voltage_cell = server.arg("min_voltage_cell").toFloat();
  if (server.hasArg("max_voltage_cell"))
    max_voltage_cell = server.arg("max_voltage_cell").toFloat();
  if (server.hasArg("batt_type"))
    batt_type = server.arg("batt_type") == "1S" ? 1 : (server.arg("batt_type") == "2S" ? 2 : 3);
  if (server.hasArg("real_voltage"))
  {
    real_voltage = server.arg("real_voltage").toFloat();
    volt_calibrate = real_voltage - (battery_voltage - volt_calibrate);
  }

  saveSettings();
  delay(50);
  server.send(200, "text/html", "<html><body><h1>Settings Saved!</h1><p>Rebooting...</p></body></html>");
  delay(1000);
  ESP.restart();
}

void saveSettings()
{
  EEPROM.put(EE_ADDR_MAX_SPEED, max_speed_on_display);
  EEPROM.put(EE_ADDR_PWM_MODE, PWM_mode);
  EEPROM.put(EE_ADDR_USE_BAT, use_bat_control);
  EEPROM.put(EE_ADDR_MIN_VOLT_CELL, min_voltage_cell);
  EEPROM.put(EE_ADDR_MAX_VOLT_CELL, max_voltage_cell);
  EEPROM.put(EE_ADDR_BAT_TYPE, batt_type);
  EEPROM.put(EE_ADDR_VOLT_CAL, volt_calibrate);
  EEPROM.put(EE_ADDR_LED_ENABLE, led_enable);
  EEPROM.put(EE_ADDR_START_FORWARD_PWM, start_forward_pwm);
  EEPROM.put(EE_ADDR_START_BACKWARD_PWM, start_backward_pwm);
  EEPROM.put(EE_ADDR_RATE_BACKWARD, max_pwm_backward);
  EEPROM.put(EE_ADDR_BRAKE_MODE, brake_mode);
  EEPROM.commit();
}

void loadSettings()
{
  EEPROM.get(EE_ADDR_MAX_SPEED, max_speed_on_display);
  EEPROM.get(EE_ADDR_PWM_MODE, PWM_mode);
  EEPROM.get(EE_ADDR_USE_BAT, use_bat_control);
  EEPROM.get(EE_ADDR_MIN_VOLT_CELL, min_voltage_cell);
  EEPROM.get(EE_ADDR_MAX_VOLT_CELL, max_voltage_cell);
  EEPROM.get(EE_ADDR_BAT_TYPE, batt_type);
  EEPROM.get(EE_ADDR_VOLT_CAL, volt_calibrate);
  EEPROM.get(EE_ADDR_LED_ENABLE, led_enable);
  EEPROM.get(EE_ADDR_START_FORWARD_PWM, start_forward_pwm);
  EEPROM.get(EE_ADDR_START_BACKWARD_PWM, start_backward_pwm);
  EEPROM.get(EE_ADDR_RATE_BACKWARD, max_pwm_backward);
  EEPROM.get(EE_ADDR_BRAKE_MODE, brake_mode);

  if (PWM_mode == STD_PWM)
  {
    max_pwm = 2012;
    min_pwm = 988;
  }
  else
  {
    max_pwm = 2200;
    min_pwm = 800;
  }
}

void ledFlash(uint8_t mode)
{
  static bool led_status = true;

  if (mode < NO_PWM_INPUT)
    digitalWrite(PIN_LED, LOW);

  else if (mode == NO_PWM_INPUT)
  {
    if (millis() - last_led_flash > 500)
    {
      last_led_flash = millis();
      digitalWrite(PIN_LED, led_status);
      led_status = !led_status;
    }
  }
  else if (mode == WIFI)
  {
    if (millis() - last_led_flash > 50)
    {
      last_led_flash = millis();
      digitalWrite(PIN_LED, led_status);
      led_status = !led_status;
    }
  }
}
