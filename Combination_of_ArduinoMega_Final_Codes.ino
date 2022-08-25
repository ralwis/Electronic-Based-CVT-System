#include<Wire.h>
#include "TimerOne.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Arduino.h>

// eFFL includes
#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>

SoftwareSerial nodemcu(5, 6);

#define ENCA 3 // GREEN
#define ENCB 4 // YELLOW
#define PWM 10
#define IN2 11
#define IN1 12

int right_intr = 0;
const int MPU=0x68; // I2C address of the MPU-6050 
int16_t AcX,AcY;

//encoder motor parameters
volatile int posi = 0; // specify posi as volatile
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//Measure speed and distance
float radius_of_wheel = 0.08;
volatile byte rotation; // variale for interrupt fun must be volatile
float timetaken,rpm,dtime;
float v;
double distance;
unsigned long pevtime;

// object library
Fuzzy *fuzzy = new Fuzzy();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);

  rotation = rpm = pevtime = 0; //Initialize all variable to zero
  attachInterrupt(digitalPinToInterrupt(2), Right_ISR, CHANGE); 
  nodemcu.begin(9600);
  Serial.println("Program started");

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");

  // fuzzy sets 
  //gyroscope //variables //Input
  FuzzyInput *gyroscope = new FuzzyInput(1);
  
  FuzzySet *gyroM0 = new FuzzySet(-12000, -12000, 0, 2400);
  gyroscope->addFuzzySet(gyroM0);
  FuzzySet *gyroM1 = new FuzzySet(0, 2400, 2400, 4800);
  gyroscope->addFuzzySet(gyroM1);
  FuzzySet *gyroM2 = new FuzzySet(2400, 4800, 4800, 7200);
  gyroscope->addFuzzySet(gyroM2);
  FuzzySet *gyroM3 = new FuzzySet(4800, 7200, 7200, 9600);
  gyroscope->addFuzzySet(gyroM3);
  FuzzySet *gyroM4 = new FuzzySet(7200, 9600, 9600, 12000);
  gyroscope->addFuzzySet(gyroM4);
  FuzzySet *gyroM5 = new FuzzySet(9600, 12000, 15000, 15000);
  gyroscope->addFuzzySet(gyroM5);

  fuzzy->addFuzzyInput(gyroscope);

  //speedlm //variables //Input
  FuzzyInput *speedlm = new FuzzyInput(2);
  
  FuzzySet *speedlmM0 = new FuzzySet(0, 15, 15, 30);
  speedlm->addFuzzySet(speedlmM0);
  FuzzySet *speedlmM1 = new FuzzySet(15, 30, 30, 45);
  speedlm->addFuzzySet(speedlmM1);
  FuzzySet *speedlmM2 = new FuzzySet(30, 45, 45, 60);
  speedlm->addFuzzySet(speedlmM2); 
  FuzzySet *speedlmM3 = new FuzzySet(45, 60, 70, 70);
  speedlm->addFuzzySet(speedlmM3);

  fuzzy->addFuzzyInput(speedlm);

  //current //variables //Input
  FuzzyInput *current = new FuzzyInput(3);

//  FuzzySet *currentM0 = new FuzzySet(0, 4, 4, 8);
//  current->addFuzzySet(currentM0);
//  FuzzySet *currentM1 = new FuzzySet(4, 8, 8, 12);
//  current->addFuzzySet(currentM1);
//  FuzzySet *currentM2 = new FuzzySet(8, 12, 12, 17);
//  current->addFuzzySet(currentM2); 
//  FuzzySet *currentM3 = new FuzzySet(12, 17, 18, 18);
//  current->addFuzzySet(currentM3);
  
  FuzzySet *currentM0 = new FuzzySet(0, 0.425, 0.425, 0.85);
  current->addFuzzySet(currentM0);
  FuzzySet *currentM1 = new FuzzySet(0.425, 0.85, 0.85, 1.275);
  current->addFuzzySet(currentM1);
  FuzzySet *currentM2 = new FuzzySet(0.85, 1.275, 1.275, 1.7);
  current->addFuzzySet(currentM2); 
  FuzzySet *currentM3 = new FuzzySet(1.275, 1.7, 1.8, 1.8);
  current->addFuzzySet(currentM3);

  fuzzy->addFuzzyInput(current);


  //encoder //variables //Output
  FuzzyOutput *encoder = new FuzzyOutput(1);
  
  FuzzySet *encoderM0 = new FuzzySet(0, 120, 120, 240);
  encoder->addFuzzySet(encoderM0);
  FuzzySet *encoderM1 = new FuzzySet(120, 240, 240, 360);
  encoder->addFuzzySet(encoderM1);
  FuzzySet *encoderM2 = new FuzzySet(240, 360, 360, 480);
  encoder->addFuzzySet(encoderM2); 
  FuzzySet *encoderM3 = new FuzzySet(360, 480, 480, 600);
  encoder->addFuzzySet(encoderM3);
  FuzzySet *encoderM4 = new FuzzySet(480, 600, 600, 720);
  encoder->addFuzzySet(encoderM4);
  FuzzySet *encoderM5 = new FuzzySet(600, 720, 720, 840);
  encoder->addFuzzySet(encoderM5);
  FuzzySet *encoderM6 = new FuzzySet(720, 840, 840, 960);
  encoder->addFuzzySet(encoderM6);
  FuzzySet *encoderM7 = new FuzzySet(840, 960, 960, 1080);
  encoder->addFuzzySet(encoderM7);
  FuzzySet *encoderM8 = new FuzzySet(960, 1080, 1080, 1200);
  encoder->addFuzzySet(encoderM8); 
  FuzzySet *encoderM9 = new FuzzySet(1080, 1200, 1200, 1320);
  encoder->addFuzzySet(encoderM9);
  FuzzySet *encoderM10 = new FuzzySet(1200, 1320, 1320, 1440);
  encoder->addFuzzySet(encoderM10);
  FuzzySet *encoderM11 = new FuzzySet(1320, 1400, 1440, 1440);
  encoder->addFuzzySet(encoderM11);

  fuzzy->addFuzzyOutput(encoder);

  // if gyroscope is gyroM0 and speedlm is speedlmM0 and current is currentM0 then encoder is encoderM3
  FuzzyRuleAntecedent *if_A1_And_G1 = new FuzzyRuleAntecedent();
  if_A1_And_G1->joinWithAND(gyroM0, speedlmM0);
  FuzzyRuleAntecedent *if_K1 = new FuzzyRuleAntecedent();
  if_K1->joinSingle(currentM0);
  FuzzyRuleAntecedent *if_A1_And_G1_And_K1 = new FuzzyRuleAntecedent();
  if_A1_And_G1_And_K1->joinWithAND(if_A1_And_G1, if_K1);
  FuzzyRuleConsequent *E1M3 = new FuzzyRuleConsequent();
  E1M3->addOutput(encoderM3);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, if_A1_And_G1_And_K1, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // if gyroscope is gyroM0 and speedlm is speedlmM1 and current is currentM0 then encoder is encoderM2
  FuzzyRuleAntecedent *if_A2_And_H2 = new FuzzyRuleAntecedent();
  if_A2_And_H2->joinWithAND(gyroM0, speedlmM1); 
  FuzzyRuleAntecedent *if_A2_And_H2_And_K2 = new FuzzyRuleAntecedent();
  if_A2_And_H2_And_K2->joinWithAND(if_A2_And_H2, if_K1); 
  FuzzyRuleConsequent *E2M2 = new FuzzyRuleConsequent();
  E2M2->addOutput(encoderM2);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, if_A2_And_H2_And_K2, E2M2);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // if gyroscope is gyroM0 and speedlm is speedlmM2 and current is currentM0 then encoder is encoderM1
  FuzzyRuleAntecedent *if_A3_And_I3 = new FuzzyRuleAntecedent();
  if_A3_And_I3->joinWithAND(gyroM0, speedlmM2);
  FuzzyRuleAntecedent *if_A3_And_I3_And_K3 = new FuzzyRuleAntecedent();
  if_A3_And_I3_And_K3->joinWithAND(if_A3_And_I3, if_K1);
  FuzzyRuleConsequent *E3M1 = new FuzzyRuleConsequent();
  E3M1->addOutput(encoderM1);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, if_A3_And_I3_And_K3, E3M1);
  fuzzy->addFuzzyRule(fuzzyRule3);

  // if gyroscope is gyroM0 and speedlm is speedlmM3 and current is currentM0 then encoder is encoderM0
  FuzzyRuleAntecedent *if_A4_And_J4 = new FuzzyRuleAntecedent();
  if_A4_And_J4->joinWithAND(gyroM0, speedlmM3);
  FuzzyRuleAntecedent *if_A4_And_J4_And_K4 = new FuzzyRuleAntecedent();
  if_A4_And_J4_And_K4->joinWithAND(if_A4_And_J4, if_K1);
  FuzzyRuleConsequent *E4M0 = new FuzzyRuleConsequent();
  E4M0->addOutput(encoderM0);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, if_A4_And_J4_And_K4, E4M0);
  fuzzy->addFuzzyRule(fuzzyRule4);

  // if gyroscope is gyroM0 and speedlm is speedlmM0 and current is currentM1 then encoder is encoderM4
//  FuzzyRuleAntecedent *if_L5 = new FuzzyRuleAntecedent();
//  if_L5->joinSingle(currentM1); 
  FuzzyRuleAntecedent *if_A5_And_G5_And_L5 = new FuzzyRuleAntecedent();
  if_A5_And_G5_And_L5->joinWithAND(if_A1_And_G1, currentM1);
  FuzzyRuleConsequent *E5M4 = new FuzzyRuleConsequent();
  E5M4->addOutput(encoderM4);
  FuzzyRule *fuzzyRule5 = new FuzzyRule(5, if_A5_And_G5_And_L5, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule5);

  // if gyroscope is gyroM0 and speedlm is speedlmM1 and current is currentM1 then encoder is encoderM3
  FuzzyRuleAntecedent *if_A6_And_H6_And_L6 = new FuzzyRuleAntecedent();
  if_A6_And_H6_And_L6->joinWithAND(if_A2_And_H2, currentM1);
  FuzzyRule *fuzzyRule6 = new FuzzyRule(6, if_A6_And_H6_And_L6, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule6);

  // if gyroscope is gyroM0 and speedlm is speedlmM2 and current is currentM1 then encoder is encoderM2
  FuzzyRuleAntecedent *if_A7_And_I7_And_L7 = new FuzzyRuleAntecedent();
  if_A7_And_I7_And_L7->joinWithAND(if_A3_And_I3, currentM1);
  FuzzyRule *fuzzyRule7 = new FuzzyRule(7, if_A7_And_I7_And_L7, E2M2);
  fuzzy->addFuzzyRule(fuzzyRule7);

  // if gyroscope is gyroM0 and speedlm is speedlmM3 and current is currentM1 then encoder is encoderM1
  FuzzyRuleAntecedent *if_A8_And_J8_And_L8 = new FuzzyRuleAntecedent();
  if_A8_And_J8_And_L8->joinWithAND(if_A4_And_J4, currentM1);
  FuzzyRule *fuzzyRule8 = new FuzzyRule(8, if_A8_And_J8_And_L8, E3M1);
  fuzzy->addFuzzyRule(fuzzyRule8);

  // if gyroscope is gyroM0 and speedlm is speedlmM0 and current is currentM2 then encoder is encoderM5
//  FuzzyRuleAntecedent *if_M9 = new FuzzyRuleAntecedent();
//  if_M9->joinSingle(currentM2);
  FuzzyRuleAntecedent *if_A9_And_G9_And_M9 = new FuzzyRuleAntecedent();
  if_A9_And_G9_And_M9->joinWithAND(if_A1_And_G1, currentM2);
  FuzzyRuleConsequent *E9M5 = new FuzzyRuleConsequent();
  E9M5->addOutput(encoderM5);
  FuzzyRule *fuzzyRule9 = new FuzzyRule(9, if_A9_And_G9_And_M9, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule9);

  // if gyroscope is gyroM0 and speedlm is speedlmM1 and current is currentM2 then encoder is encoderM4
  FuzzyRuleAntecedent *if_A10_And_H10_And_M10 = new FuzzyRuleAntecedent();
  if_A10_And_H10_And_M10->joinWithAND(if_A2_And_H2, currentM2);
  FuzzyRule *fuzzyRule10 = new FuzzyRule(10, if_A10_And_H10_And_M10, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule10);

  // if gyroscope is gyroM0 and speedlm is speedlmM2 and current is currentM2 then encoder is encoderM3
  FuzzyRuleAntecedent *if_A11_And_I11_And_M11 = new FuzzyRuleAntecedent();
  if_A11_And_I11_And_M11->joinWithAND(if_A3_And_I3, currentM2);
  FuzzyRule *fuzzyRule11 = new FuzzyRule(11, if_A11_And_I11_And_M11, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule11);

  // if gyroscope is gyroM0 and speedlm is speedlmM3 and current is currentM2 then encoder is encoderM2
  FuzzyRuleAntecedent *if_A12_And_J12_And_M12 = new FuzzyRuleAntecedent();
  if_A12_And_J12_And_M12->joinWithAND(if_A4_And_J4, currentM2);
  FuzzyRule *fuzzyRule12 = new FuzzyRule(12, if_A12_And_J12_And_M12, E2M2);
  fuzzy->addFuzzyRule(fuzzyRule12);

  // if gyroscope is gyroM0 and speedlm is speedlmM0 and current is currentM3 then encoder is encoderM6
//  FuzzyRuleAntecedent *if_N13 = new FuzzyRuleAntecedent();
//  if_N13->joinSingle(currentM3);
  FuzzyRuleAntecedent *if_A13_And_G13_And_N13 = new FuzzyRuleAntecedent();
  if_A13_And_G13_And_N13->joinWithAND(if_A1_And_G1, currentM3);
  FuzzyRuleConsequent *E13M6 = new FuzzyRuleConsequent();
  E13M6->addOutput(encoderM6);
  FuzzyRule *fuzzyRule13 = new FuzzyRule(13, if_A13_And_G13_And_N13, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule13);

  // if gyroscope is gyroM0 and speedlm is speedlmM1 and current is currentM3 then encoder is encoderM5
  FuzzyRuleAntecedent *if_A14_And_H14_And_N14 = new FuzzyRuleAntecedent();
  if_A14_And_H14_And_N14->joinWithAND(if_A2_And_H2, currentM3);
  FuzzyRule *fuzzyRule14 = new FuzzyRule(14, if_A14_And_H14_And_N14, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule14);

    // if gyroscope is gyroM0 and speedlm is speedlmM2 and current is currentM3 then encoder is encoderM4
  FuzzyRuleAntecedent *if_A15_And_I15_And_N15 = new FuzzyRuleAntecedent();
  if_A15_And_I15_And_N15->joinWithAND(if_A3_And_I3, currentM3);
  FuzzyRule *fuzzyRule15 = new FuzzyRule(15, if_A15_And_I15_And_N15, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule15);

  // if gyroscope is gyroM0 and speedlm is speedlmM3 and current is currentM3 then encoder is encoderM3
  FuzzyRuleAntecedent *if_A16_And_J16_And_N16 = new FuzzyRuleAntecedent();
  if_A16_And_J16_And_N16->joinWithAND(if_A4_And_J4, currentM3);
  FuzzyRule *fuzzyRule16 = new FuzzyRule(16, if_A16_And_J16_And_N16, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule16);

  // if gyroscope is gyroM1 and speedlm is speedlmM0 and current is currentM0 then encoder is encoderM4
  FuzzyRuleAntecedent *if_B17_And_G17 = new FuzzyRuleAntecedent();
  if_B17_And_G17->joinWithAND(gyroM1, speedlmM0);
//  FuzzyRuleAntecedent *if_K2 = new FuzzyRuleAntecedent();
//  if_K2->joinSingle(currentM0);
  FuzzyRuleAntecedent *if_B17_And_G17_And_K17 = new FuzzyRuleAntecedent();
  if_B17_And_G17_And_K17->joinWithAND(if_B17_And_G17, if_K1);
  FuzzyRule *fuzzyRule17 = new FuzzyRule(17, if_B17_And_G17_And_K17, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule17);

   // if gyroscope is gyroM1 and speedlm is speedlmM1 and current is currentM0 then encoder is encoderM3
  FuzzyRuleAntecedent *if_B18_And_H18 = new FuzzyRuleAntecedent();
  if_B18_And_H18->joinWithAND(gyroM1, speedlmM1);
  FuzzyRuleAntecedent *if_B18_And_H18_And_K18 = new FuzzyRuleAntecedent();
  if_B18_And_H18_And_K18->joinWithAND(if_B18_And_H18, if_K1);
  FuzzyRule *fuzzyRule18 = new FuzzyRule(18, if_B18_And_H18_And_K18, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule18);

  // if gyroscope is gyroM1 and speedlm is speedlmM2 and current is currentM0 then encoder is encoderM2
  FuzzyRuleAntecedent *if_B19_And_I19 = new FuzzyRuleAntecedent();
  if_B19_And_I19->joinWithAND(gyroM1, speedlmM2);
  FuzzyRuleAntecedent *if_B19_And_I19_And_K19 = new FuzzyRuleAntecedent();
  if_B19_And_I19_And_K19->joinWithAND(if_B19_And_I19, if_K1);
  FuzzyRule *fuzzyRule19 = new FuzzyRule(19, if_B19_And_I19_And_K19, E2M2);
  fuzzy->addFuzzyRule(fuzzyRule19);

  // if gyroscope is gyroM1 and speedlm is speedlmM3 and current is currentM0 then encoder is encoderM1
  FuzzyRuleAntecedent *if_B20_And_J20 = new FuzzyRuleAntecedent();
  if_B20_And_J20->joinWithAND(gyroM1, speedlmM3);
  FuzzyRuleAntecedent *if_B20_And_J20_And_K20 = new FuzzyRuleAntecedent();
  if_B20_And_J20_And_K20->joinWithAND(if_B20_And_J20, currentM0);
  FuzzyRule *fuzzyRule20 = new FuzzyRule(20, if_B20_And_J20_And_K20, E3M1);
  fuzzy->addFuzzyRule(fuzzyRule20);

  // if gyroscope is gyroM1 and speedlm is speedlmM0 and current is currentM1 then encoder is encoderM5
  FuzzyRuleAntecedent *if_B21_And_G21_And_L21 = new FuzzyRuleAntecedent();
  if_B21_And_G21_And_L21->joinWithAND(if_B17_And_G17, currentM1);
  FuzzyRule *fuzzyRule21 = new FuzzyRule(21, if_B21_And_G21_And_L21, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule21);

  // if gyroscope is gyroM1 and speedlm is speedlmM1 and current is currentM1 then encoder is encoderM4
  FuzzyRuleAntecedent *if_B22_And_H22_And_L22 = new FuzzyRuleAntecedent();
  if_B22_And_H22_And_L22->joinWithAND(if_B18_And_H18, currentM1);
  FuzzyRule *fuzzyRule22 = new FuzzyRule(22, if_B22_And_H22_And_L22, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule22);

  // if gyroscope is gyroM1 and speedlm is speedlmM2 and current is currentM1 then encoder is encoderM3
  FuzzyRuleAntecedent *if_B23_And_I23_And_L23 = new FuzzyRuleAntecedent();
  if_B23_And_I23_And_L23->joinWithAND(if_B19_And_I19, currentM1);
  FuzzyRule *fuzzyRule23 = new FuzzyRule(23, if_B23_And_I23_And_L23, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule23);

  // if gyroscope is gyroM1 and speedlm is speedlmM3 and current is currentM1 then encoder is encoderM2
  FuzzyRuleAntecedent *if_B24_And_J24_And_L24 = new FuzzyRuleAntecedent();
  if_B24_And_J24_And_L24->joinWithAND(if_B20_And_J20, currentM1);
  FuzzyRule *fuzzyRule24 = new FuzzyRule(24, if_B24_And_J24_And_L24, E2M2);
  fuzzy->addFuzzyRule(fuzzyRule24);

  // if gyroscope is gyroM1 and speedlm is speedlmM0 and current is currentM2 then encoder is encoderM6
  FuzzyRuleAntecedent *if_B25_And_G25_And_M25 = new FuzzyRuleAntecedent();
  if_B25_And_G25_And_M25->joinWithAND(if_B17_And_G17, currentM2);
  FuzzyRule *fuzzyRule25 = new FuzzyRule(25, if_B25_And_G25_And_M25, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule25);

  // if gyroscope is gyroM1 and speedlm is speedlmM1 and current is currentM2 then encoder is encoderM5
  FuzzyRuleAntecedent *if_B26_And_H26_And_M26 = new FuzzyRuleAntecedent();
  if_B26_And_H26_And_M26->joinWithAND(if_B18_And_H18, currentM2);
  FuzzyRule *fuzzyRule26 = new FuzzyRule(26, if_B26_And_H26_And_M26, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule26);

  // if gyroscope is gyroM1 and speedlm is speedlmM2 and current is currentM2 then encoder is encoderM4
  FuzzyRuleAntecedent *if_B27_And_I27_And_M27 = new FuzzyRuleAntecedent();
  if_B27_And_I27_And_M27->joinWithAND(if_B19_And_I19, currentM2);
  FuzzyRule *fuzzyRule27 = new FuzzyRule(27, if_B27_And_I27_And_M27, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule27);

  // if gyroscope is gyroM1 and speedlm is speedlmM3 and current is currentM2 then encoder is encoderM3
  FuzzyRuleAntecedent *if_B28_And_J28_And_M28 = new FuzzyRuleAntecedent();
  if_B28_And_J28_And_M28->joinWithAND(if_B20_And_J20, currentM2);
  FuzzyRule *fuzzyRule28 = new FuzzyRule(28, if_B28_And_J28_And_M28, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule28);

  // if gyroscope is gyroM1 and speedlm is speedlmM0 and current is currentM3 then encoder is encoderM7
  FuzzyRuleAntecedent *if_B29_And_G29_And_N29 = new FuzzyRuleAntecedent();
  if_B29_And_G29_And_N29->joinWithAND(if_B17_And_G17, currentM3);
  FuzzyRuleConsequent *E29M7 = new FuzzyRuleConsequent();
  E29M7->addOutput(encoderM7);
  FuzzyRule *fuzzyRule29 = new FuzzyRule(29, if_B29_And_G29_And_N29, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule29);

  // if gyroscope is gyroM1 and speedlm is speedlmM1 and current is currentM3 then encoder is encoderM6
  FuzzyRuleAntecedent *if_B30_And_H30_And_N30 = new FuzzyRuleAntecedent();
  if_B30_And_H30_And_N30->joinWithAND(if_B18_And_H18, currentM3);
  FuzzyRule *fuzzyRule30 = new FuzzyRule(30, if_B30_And_H30_And_N30, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule30);

  // if gyroscope is gyroM1 and speedlm is speedlmM2 and current is currentM3 then encoder is encoderM5
  FuzzyRuleAntecedent *if_B31_And_I31_And_N31 = new FuzzyRuleAntecedent();
  if_B31_And_I31_And_N31->joinWithAND(if_B19_And_I19, currentM3);
  FuzzyRule *fuzzyRule31 = new FuzzyRule(31, if_B31_And_I31_And_N31, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule31);

  // if gyroscope is gyroM1 and speedlm is speedlmM3 and current is currentM3 then encoder is encoderM4
  FuzzyRuleAntecedent *if_B32_And_J32_And_N32 = new FuzzyRuleAntecedent();
  if_B32_And_J32_And_N32->joinWithAND(if_B20_And_J20, currentM3);
  FuzzyRule *fuzzyRule32 = new FuzzyRule(32, if_B32_And_J32_And_N32, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule32);

  // if gyroscope is gyroM2 and speedlm is speedlmM0 and current is currentM0 then encoder is encoderM5
  FuzzyRuleAntecedent *if_C33_And_G33 = new FuzzyRuleAntecedent();
  if_C33_And_G33->joinWithAND(gyroM2, speedlmM0);
  FuzzyRuleAntecedent *if_C33_And_G33_And_K33 = new FuzzyRuleAntecedent();
  if_C33_And_G33_And_K33->joinWithAND(if_C33_And_G33, currentM0);
  FuzzyRule *fuzzyRule33 = new FuzzyRule(33, if_C33_And_G33_And_K33, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule33);

  // if gyroscope is gyroM2 and speedlm is speedlmM1 and current is currentM0 then encoder is encoderM4
  FuzzyRuleAntecedent *if_C34_And_H34 = new FuzzyRuleAntecedent();
  if_C34_And_H34->joinWithAND(gyroM2, speedlmM1);
  FuzzyRuleAntecedent *if_C34_And_H34_And_K34 = new FuzzyRuleAntecedent();
  if_C34_And_H34_And_K34->joinWithAND(if_C34_And_H34, currentM0);
  FuzzyRule *fuzzyRule34 = new FuzzyRule(34, if_C34_And_H34_And_K34, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule34);

  // if gyroscope is gyroM2 and speedlm is speedlmM2 and current is currentM0 then encoder is encoderM3
  FuzzyRuleAntecedent *if_C35_And_I35 = new FuzzyRuleAntecedent();
  if_C35_And_I35->joinWithAND(gyroM2, speedlmM2);
  FuzzyRuleAntecedent *if_C35_And_I35_And_K35 = new FuzzyRuleAntecedent();
  if_C35_And_I35_And_K35->joinWithAND(if_C35_And_I35, currentM0);
  FuzzyRule *fuzzyRule35 = new FuzzyRule(35, if_C35_And_I35_And_K35, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule35);

  // if gyroscope is gyroM2 and speedlm is speedlmM3 and current is currentM0 then encoder is encoderM2
  FuzzyRuleAntecedent *if_C36_And_J36 = new FuzzyRuleAntecedent();
  if_C36_And_J36->joinWithAND(gyroM2, speedlmM3);
  FuzzyRuleAntecedent *if_C36_And_J36_And_K36 = new FuzzyRuleAntecedent();
  if_C36_And_J36_And_K36->joinWithAND(if_C36_And_J36, currentM0);
  FuzzyRule *fuzzyRule36 = new FuzzyRule(36, if_C36_And_J36_And_K36, E2M2);
  fuzzy->addFuzzyRule(fuzzyRule36);

  // if gyroscope is gyroM2 and speedlm is speedlmM0 and current is currentM1 then encoder is encoderM6
  FuzzyRuleAntecedent *if_C37_And_G37 = new FuzzyRuleAntecedent();
  if_C37_And_G37->joinWithAND(gyroM2, speedlmM0);
  FuzzyRuleAntecedent *if_C37_And_G37_And_L37 = new FuzzyRuleAntecedent();
  if_C37_And_G37_And_L37->joinWithAND(if_C33_And_G33, currentM1);
  FuzzyRule *fuzzyRule37 = new FuzzyRule(37, if_C37_And_G37_And_L37, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule37);

  // if gyroscope is gyroM2 and speedlm is speedlmM1 and current is currentM1 then encoder is encoderM5
  FuzzyRuleAntecedent *if_C38_And_H38_And_L38 = new FuzzyRuleAntecedent();
  if_C38_And_H38_And_L38->joinWithAND(if_C34_And_H34, currentM1);
  FuzzyRule *fuzzyRule38 = new FuzzyRule(38, if_C38_And_H38_And_L38, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule38);

  // if gyroscope is gyroM2 and speedlm is speedlmM2 and current is currentM1 then encoder is encoderM4
  FuzzyRuleAntecedent *if_C39_And_I39_And_L39 = new FuzzyRuleAntecedent();
  if_C39_And_I39_And_L39->joinWithAND(if_C35_And_I35, currentM1);
  FuzzyRule *fuzzyRule39 = new FuzzyRule(39, if_C39_And_I39_And_L39, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule39);

 // if gyroscope is gyroM2 and speedlm is speedlmM3 and current is currentM1 then encoder is encoderM3
  FuzzyRuleAntecedent *if_C40_And_J40_And_L40 = new FuzzyRuleAntecedent();
  if_C40_And_J40_And_L40->joinWithAND(if_C36_And_J36, currentM1);
  FuzzyRule *fuzzyRule40 = new FuzzyRule(40, if_C40_And_J40_And_L40, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule40);
  
  // if gyroscope is gyroM2 and speedlm is speedlmM0 and current is currentM2 then encoder is encoderM7
  FuzzyRuleAntecedent *if_C41_And_G41_And_M41 = new FuzzyRuleAntecedent();
  if_C41_And_G41_And_M41->joinWithAND(if_C33_And_G33, currentM2);
  FuzzyRule *fuzzyRule41 = new FuzzyRule(41, if_C41_And_G41_And_M41, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule41);

  // if gyroscope is gyroM2 and speedlm is speedlmM1 and current is currentM2 then encoder is encoderM6
  FuzzyRuleAntecedent *if_C42_And_H42_And_M42 = new FuzzyRuleAntecedent();
  if_C42_And_H42_And_M42->joinWithAND(if_C34_And_H34, currentM2);
  FuzzyRule *fuzzyRule42 = new FuzzyRule(42, if_C42_And_H42_And_M42, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule42);

  // if gyroscope is gyroM2 and speedlm is speedlmM2 and current is currentM2 then encoder is encoderM5
  FuzzyRuleAntecedent *if_C43_And_I43_And_M43 = new FuzzyRuleAntecedent();
  if_C43_And_I43_And_M43->joinWithAND(if_C35_And_I35, currentM2);
  FuzzyRule *fuzzyRule43 = new FuzzyRule(43, if_C43_And_I43_And_M43, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule43);

  // if gyroscope is gyroM2 and speedlm is speedlmM3 and current is currentM2 then encoder is encoderM4
  FuzzyRuleAntecedent *if_C44_And_J44_And_M44 = new FuzzyRuleAntecedent();
  if_C44_And_J44_And_M44->joinWithAND(if_C36_And_J36, currentM2);
  FuzzyRule *fuzzyRule44 = new FuzzyRule(44, if_C44_And_J44_And_M44, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule44);

  // if gyroscope is gyroM2 and speedlm is speedlmM0 and current is currentM3 then encoder is encoderM8
  FuzzyRuleAntecedent *if_C45_And_G45_And_N45 = new FuzzyRuleAntecedent();
  if_C45_And_G45_And_N45->joinWithAND(if_C33_And_G33, currentM3);
  FuzzyRuleConsequent *E45M8 = new FuzzyRuleConsequent();
  E45M8->addOutput(encoderM8);
  FuzzyRule *fuzzyRule45 = new FuzzyRule(45, if_C45_And_G45_And_N45, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule45);

  // if gyroscope is gyroM2 and speedlm is speedlmM1 and current is currentM3 then encoder is encoderM7
  FuzzyRuleAntecedent *if_C46_And_H46_And_N46 = new FuzzyRuleAntecedent();
  if_C46_And_H46_And_N46->joinWithAND(if_C34_And_H34, currentM3);
  FuzzyRule *fuzzyRule46 = new FuzzyRule(46, if_C46_And_H46_And_N46, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule46);

  // if gyroscope is gyroM2 and speedlm is speedlmM2 and current is currentM3 then encoder is encoderM6
  FuzzyRuleAntecedent *if_C47_And_I47_And_N47 = new FuzzyRuleAntecedent();
  if_C47_And_I47_And_N47->joinWithAND(if_C35_And_I35, currentM3);
  FuzzyRule *fuzzyRule47 = new FuzzyRule(47, if_C47_And_I47_And_N47, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule47);

  // if gyroscope is gyroM2 and speedlm is speedlmM3 and current is currentM3 then encoder is encoderM5
  FuzzyRuleAntecedent *if_C48_And_J48_And_N48 = new FuzzyRuleAntecedent();
  if_C48_And_J48_And_N48->joinWithAND(if_C36_And_J36, currentM3);
  FuzzyRule *fuzzyRule48 = new FuzzyRule(48, if_C48_And_J48_And_N48, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule48);

    // if gyroscope is gyroM3 and speedlm is speedlmM0 and current is currentM0 then encoder is encoderM6
  FuzzyRuleAntecedent *if_D49_And_G49 = new FuzzyRuleAntecedent();
  if_D49_And_G49->joinWithAND(gyroM3, speedlmM0);
  FuzzyRuleAntecedent *if_D49_And_G49_And_K49 = new FuzzyRuleAntecedent();
  if_D49_And_G49_And_K49->joinWithAND(if_D49_And_G49, if_K1);
  FuzzyRule *fuzzyRule49 = new FuzzyRule(49, if_D49_And_G49_And_K49, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule49);

  // if gyroscope is gyroM3 and speedlm is speedlmM1 and current is currentM0 then encoder is encoderM5
  FuzzyRuleAntecedent *if_D50_And_H50 = new FuzzyRuleAntecedent();
  if_D50_And_H50->joinWithAND(gyroM3, speedlmM1);
  FuzzyRuleAntecedent *if_D50_And_H50_And_K50 = new FuzzyRuleAntecedent();
  if_D50_And_H50_And_K50->joinWithAND(if_D50_And_H50, if_K1);
  FuzzyRule *fuzzyRule50 = new FuzzyRule(50, if_D50_And_H50_And_K50, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule50);

  // if gyroscope is gyroM3 and speedlm is speedlmM2 and current is currentM0 then encoder is encoderM4
  FuzzyRuleAntecedent *if_D51_And_I51 = new FuzzyRuleAntecedent();
  if_D51_And_I51->joinWithAND(gyroM3, speedlmM2);
  FuzzyRuleAntecedent *if_D51_And_I51_And_K51 = new FuzzyRuleAntecedent();
  if_D51_And_I51_And_K51->joinWithAND(if_D51_And_I51, if_K1);
  FuzzyRule *fuzzyRule51 = new FuzzyRule(51, if_D51_And_I51_And_K51, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule51);

  // if gyroscope is gyroM3 and speedlm is speedlmM3 and current is currentM0 then encoder is encoderM3
  FuzzyRuleAntecedent *if_D52_And_J52 = new FuzzyRuleAntecedent();
  if_D52_And_J52->joinWithAND(gyroM3, speedlmM3);
  FuzzyRuleAntecedent *if_D52_And_J52_And_K52 = new FuzzyRuleAntecedent();
  if_D52_And_J52_And_K52->joinWithAND(if_D52_And_J52, if_K1);
  FuzzyRule *fuzzyRule52 = new FuzzyRule(52, if_D52_And_J52_And_K52, E1M3);
  fuzzy->addFuzzyRule(fuzzyRule52);

  // if gyroscope is gyroM3 and speedlm is speedlmM0 and current is currentM1 then encoder is encoderM7
  FuzzyRuleAntecedent *if_D53_And_G53_And_L53 = new FuzzyRuleAntecedent();
  if_D53_And_G53_And_L53->joinWithAND(if_D49_And_G49, currentM1);
  FuzzyRule *fuzzyRule53 = new FuzzyRule(53, if_D53_And_G53_And_L53, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule53);

  // if gyroscope is gyroM3 and speedlm is speedlmM1 and current is currentM1 then encoder is encoderM6
  FuzzyRuleAntecedent *if_D54_And_H54_And_L54 = new FuzzyRuleAntecedent();
  if_D54_And_H54_And_L54->joinWithAND(if_D50_And_H50, currentM1);
  FuzzyRule *fuzzyRule54 = new FuzzyRule(54, if_D54_And_H54_And_L54, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule54);

  // if gyroscope is gyroM3 and speedlm is speedlmM2 and current is currentM1 then encoder is encoderM5
  FuzzyRuleAntecedent *if_D55_And_I55_And_L55 = new FuzzyRuleAntecedent();
  if_D55_And_I55_And_L55->joinWithAND(if_D51_And_I51, currentM1);
  FuzzyRule *fuzzyRule55 = new FuzzyRule(55, if_D55_And_I55_And_L55, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule55);

 // if gyroscope is gyroM3 and speedlm is speedlmM3 and current is currentM1 then encoder is encoderM4
  FuzzyRuleAntecedent *if_D56_And_J56_And_L56 = new FuzzyRuleAntecedent();
  if_D56_And_J56_And_L56->joinWithAND(if_D52_And_J52, currentM1);
  FuzzyRule *fuzzyRule56 = new FuzzyRule(56, if_D56_And_J56_And_L56, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule56);
  
  // if gyroscope is gyroM3 and speedlm is speedlmM0 and current is currentM2 then encoder is encoderM8
  FuzzyRuleAntecedent *if_D57_And_G57_And_M57 = new FuzzyRuleAntecedent();
  if_D57_And_G57_And_M57->joinWithAND(if_D49_And_G49, currentM2);
  FuzzyRule *fuzzyRule57 = new FuzzyRule(57, if_D57_And_G57_And_M57, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule57);

  // if gyroscope is gyroM3 and speedlm is speedlmM1 and current is currentM2 then encoder is encoderM7
  FuzzyRuleAntecedent *if_D58_And_H58_And_M58 = new FuzzyRuleAntecedent();
  if_D58_And_H58_And_M58->joinWithAND(if_D50_And_H50, currentM2);
  FuzzyRule *fuzzyRule58 = new FuzzyRule(58, if_D58_And_H58_And_M58, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule58);

  // if gyroscope is gyroM3 and speedlm is speedlmM2 and current is currentM2 then encoder is encoderM6
  FuzzyRuleAntecedent *if_D59_And_I59_And_M59 = new FuzzyRuleAntecedent();
  if_D59_And_I59_And_M59->joinWithAND(if_D51_And_I51, currentM2);
  FuzzyRule *fuzzyRule59 = new FuzzyRule(59, if_D59_And_I59_And_M59, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule59);

  // if gyroscope is gyroM3 and speedlm is speedlmM3 and current is currentM2 then encoder is encoderM5
  FuzzyRuleAntecedent *if_D60_And_J60_And_M60 = new FuzzyRuleAntecedent();
  if_D60_And_J60_And_M60->joinWithAND(if_D52_And_J52, currentM2);
  FuzzyRule *fuzzyRule60 = new FuzzyRule(60, if_D60_And_J60_And_M60, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule60);

  // if gyroscope is gyroM3 and speedlm is speedlmM0 and current is currentM3 then encoder is encoderM9
  FuzzyRuleAntecedent *if_D61_And_G61_And_N61 = new FuzzyRuleAntecedent();
  if_D61_And_G61_And_N61->joinWithAND(if_D49_And_G49, currentM3);
  FuzzyRuleConsequent *E61M9 = new FuzzyRuleConsequent();
  E61M9->addOutput(encoderM9);
  FuzzyRule *fuzzyRule61 = new FuzzyRule(61, if_D61_And_G61_And_N61, E61M9);
  fuzzy->addFuzzyRule(fuzzyRule61);

  // if gyroscope is gyroM3 and speedlm is speedlmM1 and current is currentM3 then encoder is encoderM8
  FuzzyRuleAntecedent *if_D62_And_H62_And_N62 = new FuzzyRuleAntecedent();
  if_D62_And_H62_And_N62->joinWithAND(if_D50_And_H50, currentM3);
  FuzzyRule *fuzzyRule62 = new FuzzyRule(62, if_D62_And_H62_And_N62, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule62);

  // if gyroscope is gyroM3 and speedlm is speedlmM2 and current is currentM3 then encoder is encoderM7
  FuzzyRuleAntecedent *if_D63_And_I63_And_N63 = new FuzzyRuleAntecedent();
  if_D63_And_I63_And_N63->joinWithAND(if_D51_And_I51, currentM3);
  FuzzyRule *fuzzyRule63 = new FuzzyRule(63, if_D63_And_I63_And_N63, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule63);

  // if gyroscope is gyroM3 and speedlm is speedlmM3 and current is currentM3 then encoder is encoderM6
  FuzzyRuleAntecedent *if_D64_And_J64_And_N64 = new FuzzyRuleAntecedent();
  if_D64_And_J64_And_N64->joinWithAND(if_D52_And_J52, currentM3);
  FuzzyRule *fuzzyRule64 = new FuzzyRule(64, if_D64_And_J64_And_N64, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule64);

    // if gyroscope is gyroM4 and speedlm is speedlmM0 and current is currentM0 then encoder is encoderM7
  FuzzyRuleAntecedent *if_E65_And_G65 = new FuzzyRuleAntecedent();
  if_E65_And_G65->joinWithAND(gyroM4, speedlmM0);
  FuzzyRuleAntecedent *if_E65_And_G65_And_K65 = new FuzzyRuleAntecedent();
  if_E65_And_G65_And_K65->joinWithAND(if_E65_And_G65, currentM0);
  FuzzyRule *fuzzyRule65 = new FuzzyRule(65, if_E65_And_G65_And_K65, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule65);

  // if gyroscope is gyroM4 and speedlm is speedlmM1 and current is currentM0 then encoder is encoderM6
  FuzzyRuleAntecedent *if_E66_And_H66 = new FuzzyRuleAntecedent();
  if_E66_And_H66->joinWithAND(gyroM4, speedlmM1);
  FuzzyRuleAntecedent *if_E66_And_H66_And_K66 = new FuzzyRuleAntecedent();
  if_E66_And_H66_And_K66->joinWithAND(if_E66_And_H66, currentM0);
  FuzzyRule *fuzzyRule66 = new FuzzyRule(66, if_E66_And_H66_And_K66, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule66);

  // if gyroscope is gyroM4 and speedlm is speedlmM2 and current is currentM0 then encoder is encoderM5
  FuzzyRuleAntecedent *if_E67_And_I67 = new FuzzyRuleAntecedent();
  if_E67_And_I67->joinWithAND(gyroM4, speedlmM2);
  FuzzyRuleAntecedent *if_E67_And_I67_And_K67 = new FuzzyRuleAntecedent();
  if_E67_And_I67_And_K67->joinWithAND(if_E67_And_I67, currentM0);
  FuzzyRule *fuzzyRule67 = new FuzzyRule(67, if_E67_And_I67_And_K67, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule67);

  // if gyroscope is gyroM4 and speedlm is speedlmM3 and current is currentM0 then encoder is encoderM4
  FuzzyRuleAntecedent *if_E68_And_J68 = new FuzzyRuleAntecedent();
  if_E68_And_J68->joinWithAND(gyroM4, speedlmM3);
  FuzzyRuleAntecedent *if_E68_And_J68_And_K68 = new FuzzyRuleAntecedent();
  if_E68_And_J68_And_K68->joinWithAND(if_E68_And_J68, currentM0);
  FuzzyRule *fuzzyRule68 = new FuzzyRule(68, if_E68_And_J68_And_K68, E5M4);
  fuzzy->addFuzzyRule(fuzzyRule68);

  // if gyroscope is gyroM4 and speedlm is speedlmM0 and current is currentM1 then encoder is encoderM8
  FuzzyRuleAntecedent *if_E69_And_G69_And_L69 = new FuzzyRuleAntecedent();
  if_E69_And_G69_And_L69->joinWithAND(if_E65_And_G65, currentM1);
  FuzzyRule *fuzzyRule69 = new FuzzyRule(69, if_E69_And_G69_And_L69, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule69);

  // if gyroscope is gyroM4 and speedlm is speedlmM1 and current is currentM1 then encoder is encoderM7
  FuzzyRuleAntecedent *if_E70_And_H70_And_L70 = new FuzzyRuleAntecedent();
  if_E70_And_H70_And_L70->joinWithAND(if_E66_And_H66, currentM1);
  FuzzyRule *fuzzyRule70 = new FuzzyRule(70, if_E70_And_H70_And_L70, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule70);

  // if gyroscope is gyroM4 and speedlm is speedlmM2 and current is currentM1 then encoder is encoderM6
  FuzzyRuleAntecedent *if_E71_And_I71_And_L71 = new FuzzyRuleAntecedent();
  if_E71_And_I71_And_L71->joinWithAND(if_E67_And_I67, currentM1);
  FuzzyRule *fuzzyRule71 = new FuzzyRule(71, if_E71_And_I71_And_L71, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule71);

 // if gyroscope is gyroM4 and speedlm is speedlmM3 and current is currentM1 then encoder is encoderM5
  FuzzyRuleAntecedent *if_E72_And_J72_And_L72 = new FuzzyRuleAntecedent();
  if_E72_And_J72_And_L72->joinWithAND(if_E68_And_J68, currentM1);
  FuzzyRule *fuzzyRule72 = new FuzzyRule(72, if_E72_And_J72_And_L72, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule72);
  
  // if gyroscope is gyroM4 and speedlm is speedlmM0 and current is currentM2 then encoder is encoderM9
  FuzzyRuleAntecedent *if_E73_And_G73_And_M73 = new FuzzyRuleAntecedent();
  if_E73_And_G73_And_M73->joinWithAND(if_E65_And_G65, currentM2);
  FuzzyRule *fuzzyRule73 = new FuzzyRule(73, if_E73_And_G73_And_M73, E61M9);
  fuzzy->addFuzzyRule(fuzzyRule73);

  // if gyroscope is gyroM4 and speedlm is speedlmM1 and current is currentM2 then encoder is encoderM8
  FuzzyRuleAntecedent *if_E74_And_H74_And_M74 = new FuzzyRuleAntecedent();
  if_E74_And_H74_And_M74->joinWithAND(if_E66_And_H66, currentM2);
  FuzzyRule *fuzzyRule74 = new FuzzyRule(74, if_E74_And_H74_And_M74, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule74);

  // if gyroscope is gyroM4 and speedlm is speedlmM2 and current is currentM2 then encoder is encoderM7
  FuzzyRuleAntecedent *if_E75_And_I75_And_M75 = new FuzzyRuleAntecedent();
  if_E75_And_I75_And_M75->joinWithAND(if_E67_And_I67, currentM2);
  FuzzyRule *fuzzyRule75 = new FuzzyRule(75, if_E75_And_I75_And_M75, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule75);

  // if gyroscope is gyroM4 and speedlm is speedlmM3 and current is currentM2 then encoder is encoderM6
  FuzzyRuleAntecedent *if_E76_And_J76_And_M76 = new FuzzyRuleAntecedent();
  if_E76_And_J76_And_M76->joinWithAND(if_E68_And_J68, currentM2);
  FuzzyRule *fuzzyRule76 = new FuzzyRule(76, if_E76_And_J76_And_M76, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule76);

  // if gyroscope is gyroM4 and speedlm is speedlmM0 and current is currentM3 then encoder is encoderM10
  FuzzyRuleAntecedent *if_E77_And_G77_And_N77 = new FuzzyRuleAntecedent();
  if_E77_And_G77_And_N77->joinWithAND(if_E65_And_G65, currentM3);
  FuzzyRuleConsequent *EN77M10 = new FuzzyRuleConsequent();
  EN77M10->addOutput(encoderM10);
  FuzzyRule *fuzzyRule77 = new FuzzyRule(77, if_E77_And_G77_And_N77, EN77M10);
  fuzzy->addFuzzyRule(fuzzyRule77);

  // if gyroscope is gyroM4 and speedlm is speedlmM1 and current is currentM3 then encoder is encoderM9
  FuzzyRuleAntecedent *if_E78_And_H78_And_N78 = new FuzzyRuleAntecedent();
  if_E78_And_H78_And_N78->joinWithAND(if_E66_And_H66, currentM3);
  FuzzyRule *fuzzyRule78 = new FuzzyRule(78, if_E78_And_H78_And_N78, E61M9);
  fuzzy->addFuzzyRule(fuzzyRule78);

  // if gyroscope is gyroM4 and speedlm is speedlmM2 and current is currentM3 then encoder is encoderM8
  FuzzyRuleAntecedent *if_E79_And_I79_And_N79 = new FuzzyRuleAntecedent();
  if_E79_And_I79_And_N79->joinWithAND(if_E67_And_I67, currentM3);
  FuzzyRule *fuzzyRule79 = new FuzzyRule(79, if_E79_And_I79_And_N79, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule79);

  // if gyroscope is gyroM4 and speedlm is speedlmM3 and current is currentM3 then encoder is encoderM7
  FuzzyRuleAntecedent *if_E80_And_J80_And_N80 = new FuzzyRuleAntecedent();
  if_E80_And_J80_And_N80->joinWithAND(if_E68_And_J68, currentM3);
  FuzzyRule *fuzzyRule80 = new FuzzyRule(80, if_E80_And_J80_And_N80, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule80);

  // if gyroscope is gyroM5 and speedlm is speedlmM0 and current is currentM0 then encoder is encoderM8
  FuzzyRuleAntecedent *if_F81_And_G81 = new FuzzyRuleAntecedent();
  if_F81_And_G81->joinWithAND(gyroM5, speedlmM0);
  FuzzyRuleAntecedent *if_F81_And_G81_And_K81 = new FuzzyRuleAntecedent();
  if_F81_And_G81_And_K81->joinWithAND(if_F81_And_G81, currentM0);
  FuzzyRule *fuzzyRule81 = new FuzzyRule(81, if_F81_And_G81_And_K81, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule81);

  // if gyroscope is gyroM5 and speedlm is speedlmM1 and current is currentM0 then encoder is encoderM7
  FuzzyRuleAntecedent *if_F82_And_H82 = new FuzzyRuleAntecedent();
  if_F82_And_H82->joinWithAND(gyroM5, speedlmM1);
  FuzzyRuleAntecedent *if_F82_And_H82_And_K82 = new FuzzyRuleAntecedent();
  if_F82_And_H82_And_K82->joinWithAND(if_F82_And_H82, currentM0);
  FuzzyRule *fuzzyRule82 = new FuzzyRule(82, if_F82_And_H82_And_K82, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule82);

  // if gyroscope is gyroM5 and speedlm is speedlmM2 and current is currentM0 then encoder is encoderM6
  FuzzyRuleAntecedent *if_F83_And_I83 = new FuzzyRuleAntecedent();
  if_F83_And_I83->joinWithAND(gyroM5, speedlmM2);
  FuzzyRuleAntecedent *if_F83_And_I83_And_K83 = new FuzzyRuleAntecedent();
  if_F83_And_I83_And_K83->joinWithAND(if_F83_And_I83, currentM0);
  FuzzyRule *fuzzyRule83 = new FuzzyRule(83, if_F83_And_I83_And_K83, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule83);

  // if gyroscope is gyroM5 and speedlm is speedlmM3 and current is currentM0 then encoder is encoderM5
  FuzzyRuleAntecedent *if_F84_And_J84 = new FuzzyRuleAntecedent();
  if_F84_And_J84->joinWithAND(gyroM5, speedlmM3);
  FuzzyRuleAntecedent *if_F84_And_J84_And_K84 = new FuzzyRuleAntecedent();
  if_F84_And_J84_And_K84->joinWithAND(if_F84_And_J84, currentM0);
  FuzzyRule *fuzzyRule84 = new FuzzyRule(84, if_F84_And_J84_And_K84, E9M5);
  fuzzy->addFuzzyRule(fuzzyRule84);

  // if gyroscope is gyroM5 and speedlm is speedlmM0 and current is currentM1 then encoder is encoderM9
  FuzzyRuleAntecedent *if_F85_And_G85_And_L85 = new FuzzyRuleAntecedent();
  if_F85_And_G85_And_L85->joinWithAND(if_F81_And_G81, currentM1);
  FuzzyRule *fuzzyRule85 = new FuzzyRule(85, if_F85_And_G85_And_L85, E61M9);
  fuzzy->addFuzzyRule(fuzzyRule85);

  // if gyroscope is gyroM5 and speedlm is speedlmM1 and current is currentM1 then encoder is encoderM8
  FuzzyRuleAntecedent *if_F86_And_H86_And_L86 = new FuzzyRuleAntecedent();
  if_F86_And_H86_And_L86->joinWithAND(if_F82_And_H82, currentM1);
  FuzzyRule *fuzzyRule86 = new FuzzyRule(86, if_F86_And_H86_And_L86, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule86);

  // if gyroscope is gyroM5 and speedlm is speedlmM2 and current is currentM1 then encoder is encoderM7
  FuzzyRuleAntecedent *if_F87_And_I87_And_L87 = new FuzzyRuleAntecedent();
  if_F87_And_I87_And_L87->joinWithAND(if_F83_And_I83, currentM1);
  FuzzyRule *fuzzyRule87 = new FuzzyRule(87, if_F87_And_I87_And_L87, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule87);

  // if gyroscope is gyroM5 and speedlm is speedlmM3 and current is currentM1 then encoder is encoderM6
  FuzzyRuleAntecedent *if_F88_And_J88_And_L88 = new FuzzyRuleAntecedent();
  if_F88_And_J88_And_L88->joinWithAND(if_F84_And_J84, currentM1);
  FuzzyRule *fuzzyRule88 = new FuzzyRule(88, if_F88_And_J88_And_L88, E13M6);
  fuzzy->addFuzzyRule(fuzzyRule88);
  
  // if gyroscope is gyroM5 and speedlm is speedlmM0 and current is currentM2 then encoder is encoderM10
  FuzzyRuleAntecedent *if_F89_And_G89_And_M89 = new FuzzyRuleAntecedent();
  if_F89_And_G89_And_M89->joinWithAND(if_F81_And_G81, currentM2);
  FuzzyRule *fuzzyRule89 = new FuzzyRule(89, if_F89_And_G89_And_M89, EN77M10);
  fuzzy->addFuzzyRule(fuzzyRule89);

  // if gyroscope is gyroM5 and speedlm is speedlmM1 and current is currentM2 then encoder is encoderM9
  FuzzyRuleAntecedent *if_F90_And_H90_And_M90 = new FuzzyRuleAntecedent();
  if_F90_And_H90_And_M90->joinWithAND(if_F82_And_H82, currentM2);
  FuzzyRule *fuzzyRule90 = new FuzzyRule(90, if_F90_And_H90_And_M90, E61M9);
  fuzzy->addFuzzyRule(fuzzyRule90);

  // if gyroscope is gyroM5 and speedlm is speedlmM2 and current is currentM2 then encoder is encoderM8
  FuzzyRuleAntecedent *if_F91_And_I91_And_M91 = new FuzzyRuleAntecedent();
  if_F91_And_I91_And_M91->joinWithAND(if_F83_And_I83, currentM2);
  FuzzyRule *fuzzyRule91 = new FuzzyRule(91, if_F91_And_I91_And_M91, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule91);

  // if gyroscope is gyroM5 and speedlm is speedlmM3 and current is currentM2 then encoder is encoderM7
  FuzzyRuleAntecedent *if_F92_And_J92_And_M92 = new FuzzyRuleAntecedent();
  if_F92_And_J92_And_M92->joinWithAND(if_F84_And_J84, currentM2);
  FuzzyRule *fuzzyRule92 = new FuzzyRule(92, if_F92_And_J92_And_M92, E29M7);
  fuzzy->addFuzzyRule(fuzzyRule92);

  // if gyroscope is gyroM5 and speedlm is speedlmM0 and current is currentM3 then encoder is encoderM11
  FuzzyRuleAntecedent *if_F93_And_G93_And_N93 = new FuzzyRuleAntecedent();
  if_F93_And_G93_And_N93->joinWithAND(if_F81_And_G81, currentM3);
  FuzzyRuleConsequent *EN93M11 = new FuzzyRuleConsequent();
  EN93M11->addOutput(encoderM11);
  FuzzyRule *fuzzyRule93 = new FuzzyRule(93, if_F93_And_G93_And_N93, EN93M11);
  fuzzy->addFuzzyRule(fuzzyRule93);

  // if gyroscope is gyroM5 and speedlm is speedlmM1 and current is currentM3 then encoder is encoderM10
  FuzzyRuleAntecedent *if_F94_And_H94_And_N94 = new FuzzyRuleAntecedent();
  if_F94_And_H94_And_N94->joinWithAND(if_F82_And_H82, currentM3);
  FuzzyRule *fuzzyRule94 = new FuzzyRule(94, if_F94_And_H94_And_N94, EN77M10);
  fuzzy->addFuzzyRule(fuzzyRule94);

  // if gyroscope is gyroM5 and speedlm is speedlmM2 and current is currentM3 then encoder is encoderM9
  FuzzyRuleAntecedent *if_F95_And_I95_And_N95 = new FuzzyRuleAntecedent();
  if_F95_And_I95_And_N95->joinWithAND(if_F83_And_I83, currentM3);
  FuzzyRule *fuzzyRule95 = new FuzzyRule(95, if_F95_And_I95_And_N95, E61M9);
  fuzzy->addFuzzyRule(fuzzyRule95);

  // if gyroscope is gyroM5 and speedlm is speedlmM3 and current is currentM3 then encoder is encoderM8
  FuzzyRuleAntecedent *if_F96_And_J96_And_N96 = new FuzzyRuleAntecedent();
  if_F96_And_J96_And_N96->joinWithAND(if_F84_And_J84, currentM3);
  FuzzyRule *fuzzyRule96 = new FuzzyRule(96, if_F96_And_J96_And_N96, E45M8);
  fuzzy->addFuzzyRule(fuzzyRule96);
}

void loop(){

  int16_t yval = yvalue();
  double update_yval = (double)yval; 
  Serial.print(" Inclination = "); Serial.print(update_yval);
  Serial.println(" ");

  StaticJsonBuffer<1000> jsonBuffer;
  JsonObject& data = jsonBuffer.createObject();
  
  if(millis()-dtime>500){
  rpm= v = 0; // make rpm and velocity as zero
  dtime=millis();
 }

  v = radius_of_wheel * rpm * 0.376; //0.033 is the radius of the wheel in meter
  distance = (2*3.141*radius_of_wheel) * (right_intr/28);

  data["speed"] = v;
  data["distance"] = distance; 

  data.printTo(nodemcu);
  jsonBuffer.clear();
  
  Serial.print(v); Serial.println(" kmph "); 
  Serial.print(distance); Serial.println(" m "); 

  //Measure current flow
  double current_flow = 0;
  for(int i = 0; i<500; i++){
    current_flow = current_flow + (0.0264* analogRead(A0) - 13.51)/500;
    delay(1);
  }
  Serial.print("current_flow : ");
  Serial.println(current_flow);

  // if the inputs are weird, ignore them
  if (update_yval < -12000 || update_yval > 15000 || v < 0 || v > 70 ) return;

  // fuzzyfication
  fuzzy->setInput(1, update_yval);
  fuzzy->setInput(2, v);
  fuzzy->setInput(3, current_flow);
  fuzzy->fuzzify();

  // defuzzyfication
  double output = fuzzy->defuzzify(1); // defuzzify fuzzy output 1 (encoder)
  Serial.print("Output: ");
  Serial.print(output);
  Serial.println("");
  Serial.println("------");
  delay(500);

  int target = (int)output;
  target = target/5;

  rotate_encoder(target);
}

void rotate_encoder(int target){
  Serial.print("Target is : ");
  Serial.println(target);

  do{
  // set target position
  //int target = 1200;
  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 1.0;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

  if(target>pos-30 && target<pos+30){
    delay(800);
    break;
  }
}while (target<2500);
return 0;
}

int16_t yvalue(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();
  
  return AcY;

}

void Right_ISR(){
  right_intr++; delay(10);
  rotation++;
  dtime=millis();
  if(rotation>=28)
  {
    timetaken = millis()-pevtime; //timetaken in millisec
    rpm=(1000/timetaken)*60;    //formulae to calculate rpm
    pevtime = millis();
    rotation=0;
  }
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
