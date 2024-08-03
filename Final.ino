//Setup order:Sound->Sensors->Servo->Motors//
//PLEASE FOLLOW THE ORDER TO PREVENT BUG//
//ADDITIONAL SETUP//
#include <ESP32Servo.h>
#define button 23
#define playe =18 
//Sensor Setup//
#define FC1 13 //IR Sensor Front Left - Replace (GPIO) with your GPIO Number
#define FC2 12 //IR Sensor Front Right - Replace (GPIO) with your GPIO Number
#define FC3 14 //IR Sensor Back - Replace (GPIO) with your GPIO Number
#define trigPin 5 // Acvtivate HC-SR04 Pin - Replace (GPIO) with your GPIO Number
#define echoPin 18 // HC-SR04 Receiver pin - Replace (GPIO) with your GPIO 
unsigned long wait = 0;


//(Setup LCD at here)//

//Sound Setup//
 
int recorded=0;
int timerec=0;
unsigned long A=0;

unsigned long timeRecord=0;



//Set state
int FR = 0; //Front Right IR Sensor
int FL = 0; //Front Left IR Sensor
int BW = 0; //Back IR Sensor
int block = 0; //HC-SR04 Sensor
//Store location
int targetX = 0;
int targetY = 0;
//Store distance
int distance = 0;
//For HC-SR04
#define BlockDistance 10 //Detect distance (cm)
#define Rotate 90 //Rotate when detect object (degrees)
#define MAX_DISTANCE 250 //(cm)


//Servo Setup//

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int servoPin = 4;


//Motors Setup//
#define IN1 26
#define IN2 25
#define IN3 32
#define IN4 35
#define ENA 27
#define ENB 33
// control the motor speed with para PWM pin and dutyCycle
void motorSpeed(int PWM_pin,int dutyCycle){
  dutyCycle = constrain(dutyCycle, 50, 80);
  int point = (dutyCycle/100)*255;
  analogWriteResolution(PWM_pin, 8);
  analogWriteFrequency(PWM_pin, 2000);
  analogWrite(PWM_pin,point);
}
//control motor direction.
/*
Input: 
direction = 1: forward
direction = 0: stop
direction = -1: backward
*/
void motorDirection(int pin1,int pin2,int direction){
  direction = constrain(direction, -1, 1);
  switch (direction){
    case 1:
      digitalWrite(pin1, 1);
      digitalWrite(pin2, 0);
      break;
    case -1:
      digitalWrite(pin1, 0);
      digitalWrite(pin2, 1);
      break;
    default:
      digitalWrite(pin1, 0);
      digitalWrite(pin2, 0);
  }

//END OF SETUP//


}
void setup() {

  //ADDITIONAL SETUP//

  //(void setup LCD here)//

  //Sound Setup//
  //pinMode(rec,OUTPUT);
  pinMode(playe,OUTPUT);
  pinMode(button,INPUT);
  Serial.begin(9600);
  while (digitalRead(button)==0){
    continue;
  }
  if  (digitalRead(button)==1){
    timeRecord = pulseInLong(button,HIGH);
    recorded = 1;
  }
  //Sensor Setup//
    //FC-51 Setup
  pinMode(FC1,INPUT);
  pinMode(FC2,INPUT);
  pinMode(FC3,INPUT);
    //HC-SR04 Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  //Servo Setup//
  	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
	  // using default min/max of 1000us and 2000us
	  // different servos may require different min/max settings
	  // for an accurate 0 to 180 sweep
  myservo.write(90);


  //Motor Setup
  pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

//END OF VOID SETUP//
}

void loop() {
  //unsigned long currenttime=millis();
  
  if(millis()-A>=timeRecord && recorded==1){
    digitalWrite(playe,1);
    digitalWrite(playe,0);
    A = millis();
  }
  
    //SENSOR&SERVO SETUP//
  FR = digitalRead(FC1);
  FL = digitalRead(FC2);
  BW = digitalRead(FC3);
    //Create distance
  distance = getDistance();
  if (distance < BlockDistance) 
    {
     block = 1;
    }
  else
    {block = 0;}

  if(FR == 0 && FL == 0 && block == 0)
    {
      //Vehicle run straight forward
    }
  else if (FR == 0 && FL == 1 && block == 0)
    {
      //Vehicle move slightly to the Right
    }
  else if (FR == 1 && FL == 0 && block == 0)
     {
      //Vehicle move slightly to the Left
    }
   else
  {
    //Vehicle Stop
    //Vehicle backward (2.5 cm)
    myservo.write(0);
    if (block = 0 && FR == 0)
    {
      //Vehicle turn right,then run
    }
    else
      {
      myservo.write(180);
      if (block = 0 && FL == 0)
     {
        //Vehicle turn left,then run
      }
      else
      {
        //Vehicle stop,can't continue to run.
      }
    }
  }


  



 

}



    //CREATE FUNCTION//

// Calculating distance//
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0343; // (cm)
  
  if (distance >= MAX_DISTANCE) {
    return MAX_DISTANCE;
  } else {
    return distance;
  }
}

  
