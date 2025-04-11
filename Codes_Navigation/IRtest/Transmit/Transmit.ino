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

// Creating datapack structure containing LDR values
struct datapack{
  int IR3;
  int IR4;
  int IR5;
};

// Creating object of datapack type
datapack data;

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
}

///-----------------------------------MAIN LOOP-------------------------------//
void loop() {
  data.IR3 = readMux(3);
  data.IR4 = readMux(4);
  data.IR5 = readMux(5);
  radio.write(&data, sizeof(data));
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
