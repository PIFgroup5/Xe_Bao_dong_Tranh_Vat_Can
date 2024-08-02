//Setup order:Sound->Sensors->Servo->Motors//
//PLEASE FOLLOW THE ORDER TO PREVENT BUG//

//Sound Setup//
const int playe=20;
const int rec=19;
int recorded=0;
#define timerec 5000
unsigned long A=0;


//Sensor Setup//
#define FC1 (GPIO) //IR Sensor Front Left - Replace (GPIO) with your GPIO Number
#define FC2 (GPIO) //IR Sensor Front Right - Replace (GPIO) with your GPIO Number
#define FC3 (GPIO) //IR Sensor Back - Replace (GPIO) with your GPIO Number
#define trigPin (GPIO) // Acvtivate HC-SR04 Pin - Replace (GPIO) with your GPIO Number
#define echoPin (GPIO) // HC-SR04 Receiver pin - Replace (GPIO) with your GPIO Number
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
#include <ESP32Servo.h>
Servo myservo;  // create servo object to control a serv
int pos = 0;    // variable to store the servo position
int servoPin = 34;


//Motors Setup//
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define ENA 33
#define ENB 32
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

  //Sound Setup//
  pinMode(rec,OUTPUT);
  pinMode(playe,OUTPUT);
  Serial.begin(9600);

  //Sensor Setup//
    //FC-51 Setup
  pinMode(FC1,INPUT);
  pinMode(FC2,INPUT);
  pinMode(FC3,INPUT);
  pinMode(FC4,INPUT);
  pinMode(FC5,INPUT);
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
  // put your main code here, to run repeatedly:

}
