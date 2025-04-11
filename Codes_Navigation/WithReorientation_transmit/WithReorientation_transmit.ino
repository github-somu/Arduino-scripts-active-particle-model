//ABP towards light source

#include <SparkFun_TB6612.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// Wireless communication
RF24 radio(6, A0);
const byte address[] = "node1";

// Defining motor parameters
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//Mux control pins
int s0 = A5;
int s1 = A2;
int s2 = A1;

//Mux in "Z" pin
int Z_pin = A4;

//IR LED PIN
int IR_LED_pin = 10;

//LDR PIN
int LDR_pin = 2;

// Declaring LDR input pins
int F_LDRpin = 3;
int B_LDRpin = 6;

// Global variables
double Vr, Vl, eta;

// Input parameters
double Vnot = 5.00;   // Constant velocity in cm/s
double etaMax = 5.00;   // Strenght of rotational noise in rad/s
int    dT = 200;       // Delay time in ms

double diff = 2.00;
double I_target = 300.00;

//LDR calibration parameters
double af = 0.0055;
double bf = 0.5932;
double cf = -1.9255;

double ab = 0.0041;
double bb = -0.0377;
double cb = 1.8945;

unsigned long int T,t1,t2,t3,t4,T1,t5,t6;

double I_F, I_B;

// Creating datapack structure containing LDR values
struct datapack{
  int Count;
  unsigned long int t;
  unsigned long int Tr;
};

// Creating object of datapack type
datapack data;

int count = 0;
int reorient = 0;
int start = 1;
int n = 0;
unsigned long int TrSum = 0;

void setup()
{
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(IR_LED_pin, OUTPUT);

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(IR_LED_pin, HIGH);

  pinMode(LDR_pin, OUTPUT);
  digitalWrite(LDR_pin, HIGH);

  radio.begin();
  radio.openWritingPipe(address);
  radio.stopListening();

  randomSeed(analogRead(3));

  Serial.begin(9600);
  delay(2000);
}

///-----------------------------------MAIN LOOP-------------------------------//
void loop() {

  if (start == 1){ // Start time
    t1 = millis();
    start = 0;
  }
  
  I_F = analogRead(F_LDRpin);
  
  if (I_F >= I_target) { // reached target for the first time
    t2 = millis();
    T = t2 - t1;
    data.Count = count;
    data.t = T;
    data.Tr = TrSum;
    radio.write(&data, sizeof(data));
      while(1){
           brake(motor1,motor2);      
      }
  }
  else if(readMux(3) > 700 || readMux(4) > 700 || readMux(5) > 750 || (readMux(3) > 120 && readMux(4) > 120) || (readMux(4) > 120 && readMux(5) > 150)){ //Obstacle at front
    count = count + 1;
    
    t3 = millis();

    motor1.drive(-100); // move back
    motor2.drive(-100);
    delay(400);

    int F_LDR = analogRead(F_LDRpin);
    int B_LDR = analogRead(B_LDRpin);
    int dI1 = F_LDR - B_LDR;

    motor1.drive(115); // Rotate 20 deg
    motor2.drive(-115);
    delay(200);

    F_LDR = analogRead(F_LDRpin);
    B_LDR = analogRead(B_LDRpin);
    int dI2 = F_LDR - B_LDR;

    if (dI2 < dI1){
      do{
        F_LDR = analogRead(F_LDRpin);
        B_LDR = analogRead(B_LDRpin);
        dI1 = F_LDR-B_LDR;
        motor1.drive(-115); // Rotate 20 deg
        motor2.drive(+115);
        delay(200);
        F_LDR = analogRead(F_LDRpin);
        B_LDR = analogRead(B_LDRpin);
        dI2 = F_LDR - B_LDR;
      }while(dI2 > dI1);
      motor1.drive(+115); // Rotate 20 deg
      motor2.drive(-115);
      delay(200);
    }
    else{
      do{
        F_LDR = analogRead(F_LDRpin);
        B_LDR = analogRead(B_LDRpin);
        dI1 = F_LDR-B_LDR;
        motor1.drive(+115); // Rotate 20 deg
        motor2.drive(-115);
        delay(200);
        F_LDR = analogRead(F_LDRpin);
        B_LDR = analogRead(B_LDRpin);
        dI2 = F_LDR - B_LDR;
      }while(dI2 > dI1);
      motor1.drive(-115); // Rotate 20 deg
      motor2.drive(+115);
      delay(200);
    }
    t4 = millis();
    TrSum = TrSum + t4 - t3;
//    brake(motor1,motor2);
//    delay(1000);
  }

  else { // ABP
      eta = random(-100, 101) / (100.00 / etaMax);
      Vr = ((eta * 7.00) + (2.00 * Vnot)) / 2.00; // In cm/s
      Vl = (2.00 * Vnot) - Vr;              // In cm/s

      Vr = (Vr + 0.729) / 0.1114;           // Analog value
      Vl = ((Vl + 0.729) / 0.1114);

      t5 = millis();
      do{
        motor1.drive(Vr-3);
        motor2.drive(Vl+3);
        t6 = millis();
      }while(t6 < (t5 + dT) && analogRead(F_LDRpin) < I_target && readMux(3) < 700 && readMux(4) < 700 && readMux(5) < 700);
  }
}

///////////////////////////////// Functions ////////////////////////////////
//FUNCTION TO READ DATA FROM IR SENSORS
float readMux(int channel) {
  int controlPin[] = {s0, s1, s2};

  int muxChannel[8][3] = {
    {0, 0, 0}, //channel 0
    {1, 0, 0}, //channel 1
    {0, 1, 0}, //channel 2
    {1, 1, 0}, //channel 3
    {0, 0, 1}, //channel 4
    {1, 0, 1}, //channel 5
    {0, 1, 1}, //channel 6
    {1, 1, 1}, //channel 7
  };

  //loop through the 3 sig
  for (int i = 0; i < 3; i ++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the Z pin
  int val = analogRead(Z_pin);

  //  if(val>200){
  //    val = 1;
  //  }
  //  else{
  //    val = 0;
  //  }
  //return the value
  return val;
}
