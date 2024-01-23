// Modbus libraries
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

// Threading libraries
#include <mbed.h>
#include <rtos.h>
#include <platform/Callback.h>

// Finder 7M parameters
#define FINDER_7M_REG_RUN_TIME 103             // Run time
#define FINDER_7M_REG_FREQUENCY 105            // Frequency
#define FINDER_7M_REG_U1 107                   // Voltage U1
#define FINDER_7M_REG_U2 109                   // Voltage U2
#define FINDER_7M_REG_U3 111                   // Voltage U3
#define FINDER_7M_REG_ACTIVE_POWER_TOTAL 140   // Active Power Total (Pt)
#define FINDER_7M_REG_ENERGY_COUNTER_E1 406    // Energy counter E1
#define FINDER_7M_REG_ENERGY_COUNTER_E2 408    // Energy counter E2
#define FINDER_7M_REG_ENERGY_COUNTER_E3 410    // Energy counter E3
#define FINDER_7M_REG_ENERGY_COUNTER_E4 412    // Energy counter E4
#define FINDER_7M_REG_ENERGY_COUNTER_XK_E1 462 // Energy counter E1 x 1000
#define FINDER_7M_REG_ENERGY_COUNTER_XK_E2 464 // Energy counter E2 x 1000
#define FINDER_7M_REG_ENERGY_COUNTER_XK_E3 466 // Energy counter E3 x 1000
#define FINDER_7M_REG_ENERGY_COUNTER_XK_E4 468 // Energy counter E4 x 1000

// MODBUS parameters
constexpr auto MODBUS_BAUDRATE = 19200;
constexpr auto MODBUS_SERIAL_PARAMETERS = SERIAL_8N2;
constexpr auto MODBUS_BIT_DURATION = 1.f / MODBUS_BAUDRATE;
constexpr auto MODBUS_PRE_DELAY = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;
constexpr auto MODBUS_POST_DELAY = MODBUS_BIT_DURATION * 9.6f * 3.5f * 1e6;

const uint8_t MODBUS_7M_ADDRESS = 2;

// Error handeling
const uint32_t INVALID_DATA = 0xFFFFFFFF;
const char* READ_ERROR = "read error";
const char* NO_DATA = "no data";

// Data
uint32_t total_power;
uint32_t active_power;
uint32_t voltage;
float converted_active_power;
float converted_voltage;
float current;

int buttonState;
bool relayState = false;

// Time variables
unsigned long ms_start = 0;
unsigned long ms_previous = 0;
unsigned long ms_interval = 100;

// Threading parameters
rtos::Thread button;
rtos::Thread data;

void setup()
{
  // monitor
  Serial.begin(9600);

  // Button
  pinMode(BTN_USER, INPUT);

  // Relay
  pinMode(RELAY1, OUTPUT);
 
  // MODBUS
  RS485.setDelays(MODBUS_PRE_DELAY, MODBUS_POST_DELAY);

  ModbusRTUClient.setTimeout(200);

  if (ModbusRTUClient.begin(MODBUS_BAUDRATE, MODBUS_SERIAL_PARAMETERS))
  {
      Serial.println("Modbus RTU client started");
  }
  else
  {
      Serial.println("Failed to start Modbus RTU client: reset board to restart.");
      while (1)
      {
      }
  }
  button.start(mbed::callback(button_thread));
  data.start(mbed::callback(data_thread));
}

void loop()
{
}

void button_thread()
{
  while (true){
    buttonState = digitalRead(BTN_USER);

    if(buttonState == LOW) {
      relayState = !relayState;

      if (relayState) {
        digitalWrite(RELAY1, HIGH);
      } 
      else {
        digitalWrite(RELAY1, LOW);
      }

      while (buttonState == LOW){
        buttonState = digitalRead(BTN_USER);
      }
    }
  }
}

void data_thread()
{
  while (true){
    ms_start = millis();
    delay(1000);
    if (ms_start-ms_previous>ms_interval){
      ms_previous=ms_start;
      print_data();
  }
  }
}

void connection_thread(){
  while (true){
    ms_start = millis();
    delay(1000);
    if (ms_start-ms_previous>ms_interval){
      ms_previous=ms_start;

      print_data();
    }
  }
}

void print_data()
{
  Serial.println("____________________________");
  total_power = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_ENERGY_COUNTER_XK_E1);
  if (total_power != INVALID_DATA) {
    Serial.print("total_power_KWh:");
    Serial.println(total_power);
  }
  else {
    Serial.println("read error");
  }

  active_power = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_ACTIVE_POWER_TOTAL);
  if (active_power != INVALID_DATA) {
    converted_active_power = convert_t6(active_power);
    Serial.print("active_power_W:");
    Serial.println(converted_active_power);
  }
  else {
    Serial.println("read error");
  }

  voltage = modbus_7m_read32(MODBUS_7M_ADDRESS, FINDER_7M_REG_U1);
  if (voltage != INVALID_DATA) {
    converted_voltage = convert_t5(voltage);
    Serial.print("voltage_V:");
    Serial.println(converted_voltage);
  }
  else {
    Serial.println("read error");
  }

  current = converted_active_power/converted_voltage;
  Serial.print("current_A:");
  Serial.println(current);
}

float convert_t5(uint32_t n)
{
  uint32_t s = (n & 0x80000000) >> 31;
  int32_t e = (n & 0x7F000000) >> 24;
  if (s == 1)
  {
      e = e - 0x80;
  }
  uint32_t m = n & 0x00FFFFFF;
  return (float)m * pow(10, e);
}

float convert_t6(uint32_t n)
{
  uint32_t s = (n & 0x80000000) >> 31;
  int32_t e = (n & 0x7F000000) >> 24;
  if (s == 1)
  {
      e = e - 0x80;
  }
  uint32_t ms = (n & 0x00800000) >> 23;
  int32_t mv = (n & 0x007FFFFF);
  if (ms == 1)
  {
      mv = mv - 0x800000;
  }
  return (float)mv * pow(10, e);
}

uint32_t modbus_7m_read16(uint8_t addr, uint16_t reg)
{
  uint32_t attempts = 3;

  while (attempts > 0)
  {
      digitalWrite(LED_D0, HIGH);

      ModbusRTUClient.requestFrom(addr, INPUT_REGISTERS, reg, 1);
      uint32_t data = ModbusRTUClient.read();

      digitalWrite(LED_D0, LOW);

      if (data != INVALID_DATA)
      {
          return data;
      }
      else
      {
          attempts -= 1;
          delay(10);
      }
  }

  return INVALID_DATA;
}

uint32_t modbus_7m_read32(uint8_t addr, uint16_t reg)
{
  uint8_t attempts = 3;

  while (attempts > 0)
  {
      digitalWrite(LED_D0, HIGH);

      ModbusRTUClient.requestFrom(addr, INPUT_REGISTERS, reg, 2);
      uint32_t data1 = ModbusRTUClient.read();
      uint32_t data2 = ModbusRTUClient.read();

      digitalWrite(LED_D0, LOW);

      if (data1 != INVALID_DATA && data2 != INVALID_DATA)
      {
          return data1 << 16 | data2;
      }
      else
      {
          attempts -= 1;
          delay(10);
      }
  }

  return INVALID_DATA;
}
