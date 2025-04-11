// Include the motor driver library
#include <SparkFun_TB6612.h>

// Defining motor parameters
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

// Define sensor pins
#define FL_IR 3 // ForntLeft IR sensor
#define F_IR 4 // Fornt IR sensor
#define FR_IR 5 // ForntRight IR sensor
#define BL_IR 7 // BackLeft IR sensor
#define B_IR 0 // Back IR sensor
#define BR_IR 1 // BackRight IR sensor
#define F_LI A3 // Front Li sensor

// Motor offsets
const int offsetA = 1;
const int offsetB = 1;

// Define the motors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//Mux control pins
int s0 = A5;
int s1 = A2;
int s2 = A1;

//Mux in "Z" pin
int Z_pin = A4;

int IR_LED_pin = 10;

// Global variables
int FL, F, FR, BL, B, BR, LI;
unsigned long int t0,t1;
double Vr,Vl,eta;

// Input parameters
double Vnot = 5.00;   // [v] in cm/s
double etaMax = 3.00; // [\eta] in rad/s
int    epsilon = 250;      // [\epsilon] in ms

void setup()
{
  Serial.begin(9600); // Begin serial communication (optional)
  // Set the LI sensor pin mode as input
  pinMode(F_LI,INPUT);
  // Change mode of the multiplexer pins
  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT);
  pinMode(IR_LED_pin,OUTPUT);
  
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(IR_LED_pin,HIGH);
}


void loop() {
  // Reading input from the front IR and LI sensors
  FR  = readMux(FR_IR);
  F   = readMux(F_IR);
  FL  = readMux(FL_IR);
  LI = analogRead(F_LI);

  // Robot will not move at low light
  if(LI < 400){
    digitalWrite(STBY,LOW);
  }
  
  // Robot moves back when close to boundary
  else if(FR > 800 || F > 800 || FL > 800){ // Boundary in front
    t0 = millis(); // Note the start time
    do{
      motor1.drive(-100); // Move back with speed 10 cm/s
      motor2.drive(-100);
      t1 = millis(); // Current time
    }while(t1 <= t0 + 4000 && readMux(BL_IR) < 800 && readMux(B_IR) < 800 && readMux(BR_IR) < 800);
    // Move back until 4s or back is close to the boundary
  }

  // Mimic ABP without translation noise
  else{
    // Generate vl and vr
    eta = random(-100,101)/(100.00/etaMax);
    Vr = ((eta*7.00) + (2.00*Vnot))/2.00; // In cm/s
    Vl = (2.00*Vnot) - Vr;                // In cm/s

    Vr = (Vr + 0.729)/0.1114;             // Analog value
    Vl = ((Vl + 0.729)/0.1114)+3;         // Analog value
    
    t0 = millis(); // Note the start time
    do{
      motor1.drive(Vr);
      motor2.drive(Vl);
      t1 = millis(); // Current time
    }while(t1 <= t0 + epsilon && readMux(FL_IR) < 800 && readMux(F_IR) < 800 && readMux(FR_IR) < 800);
    // Move until epsilon ms or front is close to the boundary
  } 
}

// Function to read input form the IR sensors
float readMux(int channel){
  int controlPin[] = {s0, s1, s2};

  int muxChannel[8][3]={
    {0,0,0}, //channel 0
    {1,0,0}, //channel 1
    {0,1,0}, //channel 2
    {1,1,0}, //channel 3
    {0,0,1}, //channel 4
    {1,0,1}, //channel 5
    {0,1,1}, //channel 6
    {1,1,1}, //channel 7
  };

  //loop through the 3 sig
  for(int i = 0; i < 3; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the Z pin
  int val = analogRead(Z_pin);
  return val;
}
