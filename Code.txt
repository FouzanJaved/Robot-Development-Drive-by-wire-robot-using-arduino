/// Rever and ford robot code

int SteerMap=0;
int RaceMap=0;
int LeftPwm=0;
int RightPwm=0;

//MotorLeft
int enA = 3;
int in1 = 4;
int in2 = 7;

//MotorLeft
int enB = 9;
int in3 = 12;
int in4 = 13;

//For reverse
int btn=8;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
    // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //For reverse
  pinMode(btn,INPUT_PULLUP);
}

// the loop routine runs over and over again forever:
void loop() {
  
  //For forward
  int Val=digitalRead(btn);
     // read the input on analog pin 5 Race:
  int RaceVal = analogRead(A5);
  // print out the value you read:
 //Serial.println(RaceVal);
  RaceMap = map(RaceVal, 0, 1023, 11, 1);
  Serial.println("RaceVal");
  Serial.println(RaceVal);

  if(Val==0)
  {
  // read the input on analog pin 0 Brake:
  int BrakeVal = analogRead(A0);
  Serial.println(BrakeVal);

  if(BrakeVal >800)
  {
  
  // read the input on analog pin 2 Steering:
  int SteerVal = analogRead(A2);
  // print out the value you read:
  //Serial.println(SteerVal);
  SteerMap = map(SteerVal, 0, 1023, 1, 11);
  //Serial.println(SteerMap);
  //delay(500);        // delay in between reads for stability


   // read the input on analog pin 5 Race:
  int RaceVal = analogRead(A5);
  // print out the value you read:
 //Serial.println(RaceVal);
  RaceMap = map(RaceVal, 0, 1023, 11, 1);
  Serial.println("RaceMap");
  Serial.println(RaceVal);
 
  
  //Now the Equation part comes in:-
  // if sterr value is 2 then it means on left side decrease speed of left wheel to 2 and increase 
  //spedd of right wheel to 15-2=13
  // Now multiple 2 with speed pressed and 13 with speed pressed.
  if(SteerMap==1)
  {
    Serial.println("Steer 1 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap/5);
    }


  else if(SteerMap==2)
  {
    Serial.println("Steer 2 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap/4);
    }

  else if(SteerMap==3)
  {
    Serial.println("Steer 3 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap/3);
    }

  else if(SteerMap==4)
  {
    Serial.println("Steer 4 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap/2);
    }


  else if(SteerMap==5)
  {
    Serial.println("Steer 5 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap);
    }
  

  else if(SteerMap==6)
  {
    Serial.println("Steer 6 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap);
    }


  else if(SteerMap==7)
  {
    Serial.println("Steer 7 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap);
    }

  else if(SteerMap==8)
  {
    Serial.println("Steer 8 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap/2);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap);
    }

  else if(SteerMap==9)
  {
    Serial.println("Steer 9 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap/3);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap);
    }

  else if(SteerMap==10)
  {
    Serial.println("Steer 10 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap/4);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, RaceMap);
    }
  else if(SteerMap==11)
  {
    Serial.println("Steer 11 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, RaceMap/5);

    //Right Wheel Value
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB,RaceMap);
  }
      else 
  {
    Serial.println("Nothing detected");
    }

   delay(1);        // delay in between reads for stability
}

else{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
  }
  }
// If 
  else{

      // read the input on analog pin 0 Brake:
  int BrakeVal = analogRead(A0);
  Serial.println(BrakeVal);

  Serial.println("Reverse");
  
  if(BrakeVal >800)
  {
  
  // read the input on analog pin 2 Steering:
  int SteerVal = analogRead(A2);
  // print out the value you read:
  //Serial.println(SteerVal);
  SteerMap = map(SteerVal, 0, 1023, 1, 11);
  //Serial.println(SteerMap);
  //delay(500);        // delay in between reads for stability


   // read the input on analog pin 5 Race:
  int RaceVal = analogRead(A5);
  // print out the value you read:
 //Serial.println(RaceVal);
  RaceMap = map(RaceVal, 0, 1023, 11, 1);
  //Serial.println(RaceMap);
 
  
  //Now the Equation part comes in:-
  // if sterr value is 2 then it means on left side decrease speed of left wheel to 2 and increase 
  //spedd of right wheel to 15-2=13
  // Now multiple 2 with speed pressed and 13 with speed pressed.
  if(SteerMap==1)
  {
    Serial.println("Steer 1 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap/5);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }


  else if(SteerMap==2)
  {
    Serial.println("Steer 2 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap/4);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }

  else if(SteerMap==3)
  {
    Serial.println("Steer 3 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap/3);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }

  else if(SteerMap==4)
  {
    Serial.println("Steer 4 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap/2);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }


  else if(SteerMap==5)
  {
    Serial.println("Steer 5 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }
  

  else if(SteerMap==6)
  {
    Serial.println("Steer 6 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }


  else if(SteerMap==7)
  {
    Serial.println("Steer 7 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap);
    }

  else if(SteerMap==8)
  {
    Serial.println("Steer 8 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap/2);
    }

  else if(SteerMap==9)
  {
    Serial.println("Steer 9 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap/3);
    }

  else if(SteerMap==10)
  {
    Serial.println("Steer 10 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, RaceMap/4);
    }
  else if(SteerMap==11)
  {
    Serial.println("Steer 11 detected");
    RaceMap = map(RaceVal, 0, 1023, 240, 10);
    //Left Wheel Pwm
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, RaceMap);

    //Right Wheel Value
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB,RaceMap/5);
  }
      else 
  {
    Serial.println("Nothing detected");
    }

   delay(1);        // delay in between reads for stability
}

else{
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
  }
    
    }
}


//////Encoder + lcd code

/*  
  Optical Sensor Two Motor Demonstration
  DualMotorSpeedDemo.ino
  Demonstrates use of Hardware Interrupts
  to measure speed from two motors
  
  DroneBot Workshop 2017
  http://dronebotworkshop.com
*/

// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"

// Constants for Interrupt Pins
// Change values if not using Arduino Uno

const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1

// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;

// Float for number of slots in encoder disk
float diskslots = 254;  // Change to match value of encoder disk

//For average speed of robot
int  average;


// Interrupt Service Routines

// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 

// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 

// TimerOne ISR
void ISR_timerone()
{
  Timer1.detachInterrupt();  // Stop the timer
  Serial.print("Motor Speed 1: "); 
  float rotation1 = (counter1 / diskslots) * 60.00;  // calculate RPM for Motor 1
  Serial.print(rotation1);
  Serial.print(" RPM - "); 
  int v=0.03*rotation1*0.10472;
  Serial.println(v );  
  Serial.print(" Speed - "); 
  counter1 = 0;  //  reset counter to zero
  Serial.print("Motor Speed 2: "); 
  float rotation2 = (counter2 / diskslots) * 60.00;  // calculate RPM for Motor 2
  Serial.print(rotation2);  
  Serial.println(" RPM");
  int v2=0.03*rotation2*0.10472;//velocity=radius in meter 1.2 inch* rpm* 0.10472
  Serial.println(v2);  
  Serial.print(" Speed - ");  
  counter2 = 0;  //  reset counter to zero
  //Average rpm of both tyre speed
  average= (rotation1+rotation2)/2;
  Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
}

//LCD CODE
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);


void setup() 
{
  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  

  //Encoder Code
  Serial.begin(9600);
  
  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);  // Increase counter 2 when speed sensor pin goes High
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
  average=0;


} 

void loop()
{
      // Nothing in the loop!
  // You can place code here
  lcd.setCursor(0, 0); // top left
  lcd.print("RPM");
  lcd.setCursor(0, 1); // top left
  lcd.print(average);
  
 

}