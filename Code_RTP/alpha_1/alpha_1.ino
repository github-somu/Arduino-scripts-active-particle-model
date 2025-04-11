// Run and Tumble particle with exponentially distributed runTime (\lambda = 1)
//Diffusion: 0 ; D = 0 // Zero diffusion
//Run: V = 50 ~ 5 cm/sec;
//Tumble: tumble time = 0.5sec; Uniform tumble angle distribution

//INCLUDE LIBRARY
#include <SparkFun_TB6612.h>


//DEFINE PARAMETERS
//RUN
unsigned int runVelocity = 50; //analog value
unsigned int runTime[] = {200,400,600,800,1000,1200,1400,1600,1800,2000,2200,2400,2600,2800,3000,3200,3400,3600,3800,4000,4200,4400,4600,4800,4000,4200,4400,4600,4800,5000,5200,5400,5600,5800,6000};
unsigned int runTimeDist[] = {1637,1341,1098,899,736,602,493,404,331,271,222,181,149,122,100,82,67,55,45,37,30,25,20,16,13,11,9,7,6,5};
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
