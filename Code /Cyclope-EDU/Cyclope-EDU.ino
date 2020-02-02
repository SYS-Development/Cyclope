/******************************************************************************
                         Cyclope-EDU sample code
                            by Samuel Bonnard

  This project is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Cyclope sample code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Cyclope-edu code. If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <util/atomic.h>
#include "iface_nrf24l01.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

///////////// Wiring /////////////
#define MOSI_pin  5  // MOSI - 5
#define SCK_pin   7  // SCK  - 7
#define CE_pin    3  // CE   - 3
#define MISO_pin  6 // MISO - 6
#define CS_pin    14 // CS   - 14

#define MOSI_on PORTB |= _BV(5)  // PB5
#define MOSI_off PORTB &= ~_BV(5)// PB5
#define SCK_on PORTB |= _BV(7)   // PB7
#define SCK_off PORTB &= ~_BV(7) // PB7
#define CE_on PORTB |= _BV(3)    //PB3
#define CE_off PORTB &= ~_BV(3)  // PB3
#define CS_on PORTD |= _BV(6)   // PD6
#define CS_off PORTD &= ~_BV(6)  // PD6
#define MISO_on (PINB & _BV(6)) // PB6

#define ledPinB    12 // LED  - D12
#define ledPinR    13 // LED  - D13

#define INTERRUPT_PIN 2
#define Yaw_P A1
#define Throttle_P A0

const int BindPin = A2;
const int POWER_Pin = 1;
const int C_Pin = 4;
const int A_Pin = 22;
const int B_Pin = 23;
const int D_Pin = 15;
const int LOW_Pin = A3;
const int HIGH_Pin = A4;
const int Batt_P = A7;

//////////////////////////////////

///////////// HC06 /////////////
SoftwareSerial HC06(10, 11);// RX, TX
String HC06Value = "";
////////////////////////////////

///////////// MPU6050 /////////////
#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu(0x69);

bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
///////////////////////////////////

///////////// PPM /////////////
#define CHANNELS 12

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

int Pitch = 1500;
int Roll = 1500;
int Throttle;
int Yaw = 1500;

int POWER = 0;

float V_Batt = 0.0;
///////////////////////////////

///////////// NRF24L01 /////////////
#define RF_POWER TX_POWER_158mW

enum {
  PROTO_SYMAX5C1,
  PROTO_SYMAXOLD,
  PROTO_CyclopeRX,
};

enum chan_order {
  THROTTLE,
  AILERON,
  ELEVATOR,
  RUDDER,
  AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
  AUX2,  // (CH6)  flip control
  AUX3,  // (CH7)  still camera (snapshot)
  AUX4,  // (CH8)  video camera
  AUX5,  // (CH9)  headless
  AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
  AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
  AUX8,  // (CH12) Reset / Rebind
};

uint8_t transmitterID[4];
uint8_t current_protocol;
uint8_t packet[32];
static bool reset = true;
static uint16_t ppm[12] = {PPM_MIN, PPM_MIN, PPM_MIN, PPM_MIN, PPM_MID, PPM_MID,
                           PPM_MID, PPM_MID, PPM_MID, PPM_MID, PPM_MID, PPM_MID,
                          };
////////////////////////////////////
unsigned long previousMillis = 0;
const long interval = 2000;

int receiver;
int educ = 0;

void setup()
{
  Serial.begin(9600);
  HC06.begin(9600);
  HC06.write("AT+NAME CyclopeTX");

  ///////////// NRF24L01 /////////////
  randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  ////////////////////////////////////

  ///////////// MPU6050 /////////////
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {

    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    Serial.print(F("Starting Failed !"));
    Serial.print(devStatus);
    Serial.println(F(")"));
    digitalWrite(ledPinR, HIGH);
    digitalWrite(ledPinB, LOW);
  }
  pinMode(ledPinB, OUTPUT);
  pinMode(ledPinR, OUTPUT);
  digitalWrite(ledPinB, LOW);
}
///////////////////////////////////

void loop()
{
  battery();
  mpu6050();
  auxselect();
  eduselect();
  if (educ == 1){
    edu();
  }

  ppm[RUDDER] = Yaw;
  ppm[ELEVATOR] = Pitch;
  ppm[AILERON] = map(Roll, 1000, 2000, 2000, 1000);
  ppm[AUX1] = map(digitalRead(D_Pin), 1, 0, 1000, 2000);
  ppm[AUX2] = map(digitalRead(C_Pin), 1, 0, 1000, 2000);
  ppm[AUX3] = map(digitalRead(LOW_Pin), 1, 0, 1000, 2000);
  ppm[AUX4] = map(digitalRead(HIGH_Pin), 1, 0, 1000, 2000);
  ppm[AUX5] = map(digitalRead(A_Pin), 1, 0, 1000, 2000);


  uint32_t timeout = 0;

  if (reset || digitalRead(BindPin) == LOW) {
    reset = false;
    selectProtocol();
    NRF24L01_Reset();
    NRF24L01_Initialize();
    init_protocol();
    digitalWrite(ledPinB, LOW);
    digitalWrite(ledPinR, HIGH);
    POWER = 1;
  }
  else {
    digitalWrite(ledPinR, LOW);
  }

  switch (current_protocol) {
    case PROTO_SYMAX5C1:
    case PROTO_SYMAXOLD:
      timeout = process_SymaX();
      break;
    case PROTO_CyclopeRX:
      timeout = process_CyclopeRX();
      break;
  }
}


void selectProtocol()
{
  if (receiver == 0) current_protocol = PROTO_CyclopeRX; 
  else if (receiver == 1) current_protocol = PROTO_SYMAX5C1; 
}

void init_protocol()
{
  switch (current_protocol) {

    case PROTO_SYMAX5C1:
    case PROTO_SYMAXOLD:
      Symax_init();
      break;
    case PROTO_CyclopeRX:
      CyclopeRX_init();
      CyclopeRX_bind();
      break;
  }
}
