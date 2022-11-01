#include <SoftwareSerial.h>
#include <Servo.h>
#include <VoiceRecognitionV3.h>
#define RightMotorForward 6
#define RightMotorBackward 7

#define trigger 2
#define echo 3

#define LeftMotorForward 4
#define LeftMotorBackward 5

#define enable1 8
#define enable2 9

#define button 25
#define handPin 29

#define green (0)
#define red (1)
#define blue (2)

char flag = ' ';
int distancee = 0;
int dis = 0;
int distanceeB = 0;
int disB = 0;
long duration, durationB;
int counter = 0;
char letter = ' ';
String cmd;
int Basketdegree = 0;
int Degree360 = 360;
int buttonState = 0;

SoftwareSerial BTSerial(10, 11);
VR myVR(12, 13);  // 12:RX 13:TX
Servo hand;
uint8_t buf[64];
String color11 = "";
void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  myVR.begin(9600);
  if (myVR.clear() != 0) {
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while (1)
      ;
  }

  if (myVR.load((uint8_t)green) >= 0) {
  }

  if (myVR.load((uint8_t)red) >= 0) {
  }
  if (myVR.load((uint8_t)blue) >= 0) {
  }
  //hand
  hand.attach(handPin);
  //motors
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode (enable1, OUTPUT);
  pinMode (enable2, OUTPUT);
  //button
  pinMode(button, INPUT);
  digitalWrite(button,HIGH);
//leds
  pinMode(22, OUTPUT); 
  pinMode(23, OUTPUT); 
  pinMode(24, OUTPUT); 
  //ultra sonics
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);


}

void loop() {
//PickUpBall();
//dropBall();

  delay(1000);
  if (digitalRead(button) == LOW) {
    //    Serial.println("button pressed");
    buttonState += 1;
    delay(600);
    if (buttonState == 3) {
      buttonState = 0;
      delay(10);
    }
  }
/////////////////////// HAND GESTURE MODE
  if (buttonState == 1) {
    // Serial.println(b\uttonState);
      digitalWrite(23, HIGH);
    digitalWrite(24, LOW);
    digitalWrite(22, LOW);
    if (BTSerial.available()) {
      letter = BTSerial.read();
      if (letter == 'f') {
        Serial.println("moving forward");
        moveForward();
        

      }

      else if (letter == 'b') {
        Serial.println("moving backward");
        moveBackward();
       
      }

      else if (letter == 'r') {
        Serial.println("moving right");
        turnRight();
        
      }

      else if (letter == 'l') {
        Serial.println("moving left");
        turnLeft();
       
      }

      else if (letter == 's') {
         Serial.println("stoping");
        Stop();
      }

      else if (letter == 'g') {
         Serial.println("grapping");
        PickUpBall();
      }

      else if (letter == 'n') {
        Serial.println("releasing");
        dropBall();
      }
    }
  } 
    //////////////////RASPBERRY PI AUTOMATED MODE
  
  else if (buttonState == 0) {
    //Serial.println(buttonState);
      digitalWrite(22, HIGH);
    digitalWrite(23, LOW);
    digitalWrite(24, LOW);
    Serial.println("Search");

    if (Serial.available() > 0) {
     

    cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equals("SearchBalls")) {
      cmd = "";

      if (counter != 15) {
        turnRight();
        delay(200);
        counter++;
        Stop();
        delay(200);
        Serial.println("notFullRotation");
      } else {
        Stop();
        delay(5);
        counter = 0;
        Serial.println("fullRotation");
      }
    }
    if (cmd.equals("Forward05m")) {
      cmd = "";
      moveForward();
      delay(1000);
      Stop();
    }

    if (cmd.equals("ForwardToBall")) {
      cmd = "";
      counter = 0;
      hand.write(20);
      delay(1000);
      disB = readAltra();
      while (disB > 16) {
        moveForward();
        disB = readAltra();
      }
      Stop();
      delay(1000);
      
      disB = readAltra();
      while (disB > 3) {
        moveForward();
        delay(70);
        Stop();
        delay(70);
        disB = readAltra();
      }
      Stop();
      delay(1000);
      hand.write(120);
      delay(1000);
    }
    if (cmd.equals("RotateLtoBall")) {
      cmd = "";
      turnLeft();
      delay(50);
      Stop();
      delay(100);
    }

    if (cmd.equals("RotateRtoBall")) {
      cmd = "";
      turnRight();
      delay(50);
      Stop();
      delay(100);
    }
    if (cmd.equals("Hand")) {
      cmd = "";
      delay(1000);
      while (1) {
        if (digitalRead(button) == LOW)
        break;
    if (BTSerial.available()) {
      letter = BTSerial.read();
      if (letter == 'f') {
        //Serial.println("moving forward");
        moveForward();
        

      }

      else if (letter == 'b') {
        //Serial.println("moving backward");
        moveBackward();
       
      }

      else if (letter == 'r') {
        //Serial.println("moving right");
        turnRight();
        
      }

      else if (letter == 'l') {
        //Serial.println("moving left");
        turnLeft();
       
      }

      else if (letter == 's') {
        // Serial.println("stoping");
        Stop();
      }

      else if (letter == 'g') {
        // Serial.println("grapping");
        PickUpBall();
      }

      else if (letter == 'n') {
        //Serial.println("releasing");
        dropBall();
      }//if n
    }//end bt serial
      }//while
  }  //hand
  }// serial available}
  } else if (buttonState == 2) {
    
      digitalWrite(24, HIGH);
      digitalWrite(23, LOW);
      digitalWrite(22, LOW);
      int x = 0;
      while (1) {
        int ret;
        ret = myVR.recognize(buf, 50);
        if (ret > 0) {
            switch (buf[1]) {
              case green:
              color11 = "green";
              Serial.println(color11);
                break;
      case blue:
        color11 = "blue";
        Serial.println(color11);
        
        break;
      case red:
        /** turn off LED*/
        color11 = "red";
         Serial.println(color11);
        break;
      default:
        Serial.println("Record function undefined");
        break;
    }  // end switch
}  // end if
    if (Serial.available() > 0) {

    cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equals("SearchBalls")) {
      cmd = "";

      if (counter != 15) {
        turnRight();
        delay(200);
        counter++;
        Stop();
        delay(200);
        Serial.println("notFullRotation");
      } else {
        Stop();
        delay(5);
        counter = 0;
        Serial.println("fullRotation");
      }
    }
    if (cmd.equals("Forward05m")) {
      cmd = "";
      moveForward();
      delay(1000);
      Stop();
    }

    if (cmd.equals("ForwardToBall")) {
      cmd = "";
      counter = 0;
      hand.write(20);
      delay(1000);
      disB = readAltra();
      while (disB > 16) {
        moveForward();
        disB = readAltra();
      }
      Stop();
      delay(1000);
      
      disB = readAltra();
      while (disB > 3) {
        moveForward();
        delay(70);
        Stop();
        delay(70);
        disB = readAltra();
      }
      Stop();
      delay(1000);
      hand.write(120);
      delay(1000);
    }
    if (cmd.equals("RotateLtoBall")) {
      cmd = "";
      turnLeft();
      delay(50);
      Stop();
      delay(100);
    }

    if (cmd.equals("RotateRtoBall")) {
      cmd = "";
      turnRight();
      delay(50);
      Stop();
      delay(100);
    }
    if (cmd.equals("Hand")) {
      cmd = "";
      delay(1000);
      while (1) {
        if (digitalRead(button) == LOW)
        break;
    if (BTSerial.available()) {
      letter = BTSerial.read();
      if (letter == 'f') {
        //Serial.println("moving forward");
        moveForward();
        

      }

      else if (letter == 'b') {
        //Serial.println("moving backward");
        moveBackward();
       
      }

      else if (letter == 'r') {
        //Serial.println("moving right");
        turnRight();
        
      }

      else if (letter == 'l') {
        //Serial.println("moving left");
        turnLeft();
       
      }

      else if (letter == 's') {
        // Serial.println("stoping");
        Stop();
      }

      else if (letter == 'g') {
        // Serial.println("grapping");
        PickUpBall();
      }

      else if (letter == 'n') {
        //Serial.println("releasing");
        dropBall();
      }//if n
    }//end bt serial
      }//while
  }  //hand
  }// serial available}
 
  
      if (digitalRead(button) == LOW)
        break;
    }
  }








}






  void PickUpBall() {
    delay(100);
    hand.write(120);
  }

  void dropBall() {
    delay(50);
    hand.write(20);
  }

  void Stop() {
    digitalWrite(RightMotorForward, LOW);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    digitalWrite(LeftMotorBackward, LOW);
    analogWrite(enable1, 255);
    analogWrite(enable2, 255);
  }

  void moveForward() {
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
    analogWrite(enable1, 255);
    analogWrite(enable2, 255);
  }
  void moveBackward() {
    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(RightMotorForward, LOW);
    analogWrite(enable1, 255);
    analogWrite(enable2, 255);
  }

  void turnRight() {

    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW);
    analogWrite(enable1, 255);
    analogWrite(enable2, 255);
  }

  void turnLeft() {

    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(RightMotorBackward, LOW);
    analogWrite(enable1, 255);
    analogWrite(enable2, 255);
  }



  int readAltra() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  duration = pulseIn(echo, HIGH);
  distancee = duration * 0.034 / 2;

  // Serial.print("distance:");
  // Serial.println(distancee);
  return distancee;
}
