/* Developed by Anas Ahmed
   Manager, Hackerspace Karachi

   +92-332-3265201
   anas.ahm5@gmail.com
   www.hackerspacekarachi.com
*/

/* PINS USED
   V0 = Digger
   V1 = Joystick - xAxis
   V2 = Joystick - yAxis
   V3 = Speed Control MotorLeft
   V4 = System Button
   V5 = Distance Guage
   V6 = Distance Setting Button
   V7 = Modes Switch
   V8 = Distance Clear Button
   V9 = Seeder
   V10= Speed Control MotorRight
   V11= Distance Left for Digging
   V12= Seeding LED
   V13=Digging LED

*/

void  Right_ISR(), bwd(), fwd(), pause() , left(), right();

#include <Servo.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>

Servo servodig;   // create servo object to control a servo
Servo servoSeed;  // create servo object to control a servo

char auth[] = "***********************";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "***************";
char pass[] = "***************";

//Digital Pins
#define diggerPin     D0
#define distancePin   D1
#define in3           D2
#define in4           D3
#define enB           D4
#define enA           D5
#define in1           D6
#define in2           D7
#define seederPin     D8

//Variables
int clrdis, seederValue, diggerValue, speedM, systemState, manualMode, autoMode, Mode, motorR;
int motorSpeedA = 600;
int motorSpeedB = 600;
int xAxis = 512;
int yAxis = 512;
int a = 1;
int prevDigger = 0;

//For distance
int right_intr = 0;
float radius_of_wheel = 0.033;  //Measure the radius of your wheel and enter it here in m
volatile byte rotation; // variale for interrupt fun must be volatile
float timetaken, rpm, dtime;
float v, dis, digDistance;
unsigned long pevtime;


BLYNK_WRITE(V0)
{
  diggerValue = param.asInt(); // assigning incoming value from pin V0 to a variable
}
BLYNK_WRITE(V1)
{
  xAxis = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V2)
{
  yAxis = param.asInt(); // assigning incoming value from pin V2 to a variable
}

BLYNK_WRITE(V3)
{
  speedM = param.asInt(); // assigning incoming value from pin V2 to a variable
}

BLYNK_WRITE(V4)
{
  systemState = param.asInt(); // assigning incoming value from pin V2 to a variable
}

BLYNK_WRITE(V6)
{
  digDistance = param.asFloat(); // assigning incoming value from pin V2 to a variable
}
BLYNK_WRITE(V7)
{
  Mode = param.asInt(); // assigning incoming value from pin V2 to a variable
}
BLYNK_WRITE(V8)
{
  clrdis = param.asInt(); // assigning incoming value from pin V2 to a variable
}
BLYNK_WRITE(V9)
{
  seederValue = param.asInt(); // assigning incoming value from pin V2 to a variable
}
BLYNK_WRITE(V10)
{
  motorR = param.asInt(); // assigning incoming value from pin V2 to a variable
}


void setup() {
  Blynk.begin(auth, ssid, pass);

  rotation = rpm = pevtime = 0; //Initialize all variable to zero

  xAxis = 512;
  yAxis = 512;


  servodig.attach(diggerPin);  // attaches the servo on pin 8 to the servo object
  servoSeed.attach(seederPin);  // attaches the servo on pin 8 to the servo object

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(enA, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(enB, LOW);


  Serial.begin(9600);
}

void loop() {
  Blynk.run();


  //MAIN SWITCH
  if (systemState == 255) {


    //MODES SWITCH
    if (Mode == 1) {                       //Manual Mode


      // Y-axis used for forward and backward control
      if (yAxis < 470) {
        bwd();
      }
      else if (yAxis > 550) {
        fwd();
      }
      // If joystick stays in middle the motors are not moving
      else if (xAxis == 512 && yAxis == 512) {
        pause();
      }

      // X-axis used for left and right control
      if (xAxis < 470) {
        left();
      }
      if (xAxis > 550) {
        right();
      }

      //DIGGING PART
      if (diggerValue == 255) {
        servodig.write(0);

      }
      else if (diggerValue == 0) {
        servodig.write(80);

      }

      //Seeder Part
      if (seederValue == 255) {
        servoSeed.write(300);              // tell servo to go to position in variable 'pos'

      }
      else if (seederValue == 0) {
        servoSeed.write(50);              // tell servo to go to position in variable 'pos'

      }
    }

    else if (Mode == 2) {
      //Auto Mode
      fwd();
      if (digitalRead(distancePin) == HIGH) {
        Right_ISR();
      }

      //To drop to zero if vehicle stopped
      if (millis() - dtime > 500) //no inetrrupt found for 500ms
      {
        rpm = v = 0; // make rpm and velocity as zero
        dtime = millis();
      }
      //v = radius_of_wheel * rpm * 0.104; //0.033 is the radius of the wheel in meter
      dis = (2 * 3.141 * radius_of_wheel) * (right_intr / 40) * (100); // in cm
      if (clrdis == 255) {
        a = 1; right_intr = dis = 0;
      }

      Blynk.virtualWrite(V5, dis);


      float disLeft =  (a * digDistance) - dis;
      Blynk.virtualWrite(V11, disLeft );

      if (disLeft <= 0) {
        a++;
        if (a > 1) {
          servodig.write(0); // digging start
          Blynk.virtualWrite(V13, 255);
          delay(1000);

          servodig.write(80);
          Blynk.virtualWrite(V13, 0);
          servoSeed.write(300);//seeding start
          Blynk.virtualWrite(V12, 255);
          delay(200);
          servoSeed.write(50);
          Blynk.virtualWrite(V12, 0);
        }
      }

    }


  }

  else if (systemState == 0) {
    pause();
  }
}


void fwd() {
  analogWrite(enA, speedM); // Send PWM signal to motor A
  digitalWrite(in1, LOW); // Send PWM signal to motor A
  digitalWrite(in2, HIGH); // Send PWM signal to motor A


  analogWrite(enB, motorR); // Send PWM signal to motor B
  digitalWrite(in3, HIGH); // Send PWM signal to motor B
  digitalWrite(in4, LOW); // Send PWM signal to motor B

}

void bwd() {
}

void right() {
  analogWrite(enA, speedM); // Send PWM signal to motor A
  digitalWrite(in1, LOW); // Send PWM signal to motor A
  digitalWrite(in2, HIGH); // Send PWM signal to motor A


  digitalWrite(enB, LOW); // Send PWM signal to motor B
}

void left() {
  digitalWrite(enA, LOW); // Send PWM signal to motor A

  analogWrite(enB, motorR); // Send PWM signal to motor B
  digitalWrite(in3, HIGH); // Send PWM signal to motor B
  digitalWrite(in4, LOW); // Send PWM signal to motor B
}


void pause() {
  analogWrite(enA, LOW); // Send PWM signal to motor A
  analogWrite(enB, LOW); // Send PWM signal to motor B
}


void Right_ISR() {
  right_intr++;


  rotation++;
  dtime = millis();
  if (rotation >= 40)
  {
    timetaken = millis() - pevtime; //timetaken in millisec
    rpm = (1000 / timetaken) * 60; //formulae to calculate rpm
    pevtime = millis();
    rotation = 0;
  }
}
