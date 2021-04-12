#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <SmoothADC.h>
#include "Timer.h"
#include "MovingAverage.h"

#define MAX741_PIN A0
#define D1_PS2X 34 // D1 on ps2 wireless adapter (DATA)
#define D0_PS2X 35 // D0 on ps2 wireless adapter (COMMAND)
#define CS_PS2X 36
#define CLK_PS2X 37
#define BASE_SERVO_PIN 38
#define SHOULDER_SERVO_PIN 39
#define ELBOW_SERVO_PIN 40
#define GRIPPER_SERVO_PIN 41
#define PS2X_LED_PIN LED_BUILTIN
#define POWER_LED_PIN 44
#define BUZZER_PIN 2

// tone notes
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

PS2X ps2x; // create PS2 Controller Class
Servo baseservo;
Servo shoulderservo;
Servo elbowservo;
Servo gripperservo;
SmoothADC    ADC_0;        // SmoothADC instance for Pin 0
Timer t;
MovingAverage <uint16_t, 16> filterADC;

int basedegrees = 90;
int shoulderdegrees = 105;
int elbowdegrees = 160;
int gripperdegrees = 90;
unsigned int  ADC0Value = 0;    // ADC0 final value
int MarioUW_note[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4, NOTE_AS3, NOTE_AS4, 0, 0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4, NOTE_DS3, NOTE_DS4, 0, 0,
};
int MarioUW_duration[] = {
  12, 12, 12, 12, 12, 12, 6, 3,
  12, 12, 12, 12, 12, 12, 6, 3,
};
int vibrate;
int RawADCvalue, LOAD;

//void readAmp(void);

void setup() {
  Serial.begin(115200);
  ADC_0.init(A0, TB_MS, 100);  // Init ADC0 attached to A0 with a 50ms acquisition period
  if (ADC_0.isDisabled()) {
    ADC_0.enable();
  }
  ps2x.config_gamepad(CLK_PS2X, D0_PS2X, CS_PS2X, D1_PS2X, true, true); //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error  pinMode(POWER_LED_PIN, OUTPUT);
  ps2x.read_gamepad(false, 0);          //read controller and set large motor to not to spin
  pinMode(PS2X_LED_PIN, OUTPUT);
  baseservo.attach(BASE_SERVO_PIN);
  baseservo.write(basedegrees);
  shoulderservo.attach(SHOULDER_SERVO_PIN);
  shoulderservo.write(shoulderdegrees);
  elbowservo.attach(ELBOW_SERVO_PIN);
  elbowservo.write(elbowdegrees);
  gripperservo.attach(GRIPPER_SERVO_PIN);
  gripperservo.write(gripperdegrees);
  delay(1000);
  t.oscillate(POWER_LED_PIN, 250, HIGH);
}

void loop() {
  ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed
  ADC_0.serviceADCPin();
  filterADC.add(ADC_0.getADCVal());
  RawADCvalue = filterADC.get();
  LOAD = map(RawADCvalue, 500, 600, 0, 100);
  t.update();

  if (ps2x.Analog(PSS_LX) < 127 || ps2x.Analog(PSS_RX) < 127) { // Left Analog or Right Analog to Left?
    digitalWrite(PS2X_LED_PIN, HIGH);
    basedegrees--;
    basedegrees = constrain(basedegrees, 0, 180);
    baseservo.write(basedegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Analog(PSS_LX) > 128 || ps2x.Analog(PSS_RX) > 128) { // Left Analog or Right Analog to Right?
    digitalWrite(PS2X_LED_PIN, HIGH);
    basedegrees++;
    basedegrees = constrain(basedegrees, 0, 180);
    baseservo.write(basedegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Analog(PSS_LY) < 127) { // Left Analog to Up?
    digitalWrite(PS2X_LED_PIN, HIGH);
    shoulderdegrees++;
    shoulderdegrees = constrain(shoulderdegrees, 70, 155);
    shoulderservo.write(shoulderdegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Analog(PSS_LY) > 127) { // Left Analog to Down?
    digitalWrite(PS2X_LED_PIN, HIGH);
    shoulderdegrees--;
    shoulderdegrees = constrain(shoulderdegrees, 70, 155);
    shoulderservo.write(shoulderdegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Analog(PSS_RY) < 127) { // Right Analog to Up?
    digitalWrite(PS2X_LED_PIN, HIGH);
    elbowdegrees++;
    elbowdegrees = constrain(elbowdegrees, 80, 180);
    elbowservo.write(elbowdegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Analog(PSS_RY) > 127) { // Right Analog to Down?
    digitalWrite(PS2X_LED_PIN, HIGH);
    elbowdegrees--;
    elbowdegrees = constrain(elbowdegrees, 80, 180);
    elbowservo.write(elbowdegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Button(PSB_R1)) { // R1 pressed?
    digitalWrite(PS2X_LED_PIN, HIGH);
    gripperdegrees--;
    gripperdegrees = constrain(gripperdegrees, 60, 160);
    gripperservo.write(gripperdegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Button(PSB_R2)) { // R2 pressed?
    digitalWrite(PS2X_LED_PIN, HIGH);
    gripperdegrees++;
    gripperdegrees = constrain(gripperdegrees, 60, 160);
    gripperservo.write(gripperdegrees);
  }
  else {
    digitalWrite(PS2X_LED_PIN, LOW);
  }
  if (ps2x.Button(PSB_L1)) { // L1 pressed?
    vibrate = 255;
    tone(BUZZER_PIN, NOTE_DS4);
  }
  else {
    vibrate = 0;
    noTone(BUZZER_PIN);
  }
  
  Serial.print("ADC ");
  Serial.print(RawADCvalue);
  Serial.print(", LOAD ");
  Serial.print(LOAD);
  Serial.print(", B ");
  Serial.print(basedegrees);
  Serial.print(", S ");
  Serial.print(shoulderdegrees);
  Serial.print(", E ");
  Serial.print(elbowdegrees);
  Serial.print(", G ");
  Serial.println(gripperdegrees);
  delay(10);
}
