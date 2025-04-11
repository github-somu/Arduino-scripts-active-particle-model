// Run and Tumble particle with exponentially distributed runTime (\lambda = 3)

//INCLUDE LIBRARY
#include <SparkFun_TB6612.h>


//DEFINE PARAMETERS
//RUN
unsigned int runVelocity = 50; //analog value
unsigned long int runTime[] = {50,100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1050,1100,1150,1200,1250,1300,1350,1400,1450,1500,1550,1600,1650,1700,1750,1800,1850,1900,1950,2000};
unsigned long int runTimeDist[] = {19470,15163,11809,9197,7163,5578,4344,3383,2635,2052,1598,1245,969,755,588,458,357,278,216,168,131,102,80,62,48,38,29,23,18,14,11,8,7,5,4,3,2,2,1,1};
unsigned int runTimeNumber = 30;

//TUMBLE
//------------------Constant tumbleTime----------------------//
float tumbleTime = 400.00; // Do not CHANGE
int restTime = 500; // Only change this
       
// Global variables
int FL, F, FR, BL, B, BR, LI;
unsigned long int t0,t1;


// Defining motor parameters
#define PWMA 3
#define AIN2 4
#define AIN1 5
#define STBY 1
#define BIN1 7
#define BIN2 8
#define PWMB 9

//MOTOR OFFSET
const int offsetA = -1;
const int offsetB = -1;

//DEFINING MOTORS
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

//Mux control pins
int s0 = A5;
int s1 = A2;
int s2 = A1;

//Mux in "Z" pin
int Z_pin = A4;

int IR_LED_pin = 10;

void setup() {
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
  //Define random seed
  randomSeed(analogRead(A2));
}

void loop() {
  // Reading input from the front LI sensors
  LI = analogRead(F_LI);

  // Robot will not move at low light
  if(LI < 400){
    digitalWrite(STBY,LOW);
  }
  else{
    //RUN
    //Generate runTime
    runDelay = generateRunTime();
    t0 = millis(); // Note the start time
    do{
      motor1.drive(runVelocity);
      motor2.drive(runVelocity);
      t1 = millis(); // Current time
    }while(t1 <= t0 + epsilon && readMux(FL_IR) < 800 && readMux(F_IR) < 800 && readMux(FR_IR) < 800);
    // Run until runDelay or front is close to the boundary
    
    // Robot moves back when close to boundary
    if(FR > 800 || F > 800 || FL > 800){ // Boundary in front
      t0 = millis(); // Note the start time
      do{
        motor1.drive(-100); // Move back with speed 10 cm/s
        motor2.drive(-100);
        t1 = millis(); // Current time
      }while(t1 <= t0 + 4000 && readMux(BL_IR) < 800 && readMux(B_IR) < 800 && readMux(BR_IR) < 800);
      // Move back until 4s or back is close to the boundary
    }
    
    else if (t1 - t0 < runDelay){//Complete the run
      motor1.drive(runVelocity);
      motor2.drive(runVelocity);
      delay(runDelay - (t1-t0));
    }

    else{// Tumble
      //DECIDING TUMBLETIME/ANGLE
        int tumbleVelocity = generateTumbleSpeed();
        
    // Wait before tumble
        brake(motor1,motor2);
        delay(restTime - abs(tumbleTime));
   
    //TUMBLE
        motor1.drive(tumbleVelocity);
        motor2.drive(-tumbleVelocity);
        delay(tumbleTime);
    }
  }
}

int generateRunTime()  //Generate runTime according its propability in the mention distribution (higher propability means letter is generated more often)
{
    int pointer;
    int totProbabilities = 0;
    int sum = 0;
    int index = 0;

    for  (int i = 0; i < runTimeNumber; i++) totProbabilities = totProbabilities + runTimeDist[i]; // tot = sum of all probabilities

    pointer = random(0, totProbabilities);

    while (sum <= pointer)    {
        sum = sum + runTimeDist[index];
        index++;
    }
    return runTime[index -1];
}

int generateTumbleSpeed () //Generate tumble speed for constant tumble time for different tumble angle acording to the dist
{
  int tumbleAngle = random(0, 360);
  if (tumbleAngle > 180){
    tumbleAngle = tumbleAngle - 360;
  }
  //Find velocity in deg/sec
  float degsec = (tumbleAngle * 1000.00)/tumbleTime;

  //Convert velocity from deg/sec to analog value
  float analog = (degsec + 15.64)/1.97;
  
  return analog;
}
