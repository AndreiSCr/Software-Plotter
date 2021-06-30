#include <ESP_FlexyStepper.h>
#include <ESP32Servo.h>

const int MOTOR_STEP_PIN_Y =22 ;
const int MOTOR_DIRECTION_PIN_Y = 21;
const int MOTOR_STEP_PIN_X =32 ;
const int MOTOR_DIRECTION_PIN_X = 33;
const int LED_PIN = 5;
const int SWITCH_PIN_X = 26;
const int SWITCH_PIN_Y = 27;
const int SERVO_PIN = 13;

const int UP=60;
const int DOWN=0;
float X=0;
float Y=0;
int P=0;

float plot[][3]={
  {2,2,0},
  {2.5,2.5,1},
  {3,2.5,1},
  {4,3.25,1},
  {5,3.25,1},
  {5,2.75,0},
  {4,2.75,1},
  {3.5,2.5,1},
  {3,1.5,1},
  {3.5,1.5,0},
  {4,2,1},
  {5.25,2,1},
  {5.25,2.5,1},
  {5.5,2.5,0},
  {5.5,2,1},
  {5,1.5,1},
  {4.5,1.5,1},
  {4,1,1},
  {5,0.5,0},
  {5,1,1},
  {5.75,1.5,1},
  {5.75,2.5,1},
  {6,2.5,0},
  {6,1,1},
  {5.5,0.5,1},
  {6.25,0.5,0},
  {6.25,2.5,1},
  {6.5,2.5,0},
  {6.5,1,1},
  {7,0.5,1},
  {7,1,0},
  {6.75,1.5,1},
  {6.75,2.5,1},
  {7,2.75,0},
  {7.5,2.75,1},
  {8,2,1},
  {8,1.5,1},
  {7.5,1,1},
  {8,1,0},
  {8.25,1.5,1},
  {8.25,2,1},
  {7.75,3,1},
  {7,3,1},
  {7,3.25,0},
  {8.5,3.25,1},
  {8.5,1,1},
  {9,0.5,1},
  {9.5,1.5,0},
  {9.5,3,1},
  {9,3.5,1},
  {7,3.5,1},
  {7,3.75,0},
  {9.5,3.75,1},
  {9.5,4.5,0},
  {9,4.5,1},
  {8.5,4,1},
  {7,4,1},
  {7,4.25,0},
  {8,4.25,1},
  {8.5,5,1},
  {9,5,1},
  {6.75,4.5,0},
  {6.75,5,1},
  {7.5,5.75,1},
  {10,5.75,1},
  {10,6,0},
  {7.5,6,1},
  {6.5,5,1},
  {6.5,4.5,1},
  {6.25,4.5,0},
  {6.25,5,1},
  {7.5,6.25,1},
  {10,6.25,1},
  {10,6.5,0},
  {8.5,6.5,1},
  {7.75,7.5,1},
  {7.75,8,1},
  {8,8,0},
  {8,7.5,1},
  {8.5,6.75,1},
  {10,6.75,1},
  {10.25,7,0},
  {10.25,8.5,1},
  {10.5,8.5,0},
  {10.5,7,1},
  {10.75,7,0},
  {10.75,8.5,1},
  {11,8.5,0},
  {11,7,1},
  {11.25,7,0},
  {11.25,8,1},
  {12,8.5,1},
  {12.5,8.5,1},
  {11.5,6.75,0},
  {13.5,6.75,1},
  {14,7.5,1},
  {14.5,7.5,1},
  {15,8,1},
  {16,7,0},
  {15,7,1},
  {14.5,6.5,1},
  {11.5,6.5,1},
  {11.5,6.25,0},
  {14,6.25,1},
  {14.5,5.5,1},
  {15.5,5.5,1},
  {16,5,1},
  {15.5,4.5,0},
  {15,4.5,1},
  {14.5,5,1},
  {13.5,5,1},
  {13,6,1},
  {11.5,6,1},
  {11.5,5.75,0},
  {12.5,5.75,1},
  {13.5,4.5,1},
  {14.5,4.5,1},
  {14.5,3.5,1},
  {13.5,3.5,0},
  {13,4,1},
  {11.5,4,1},
  {11.25,4.5,1},
  {11.25,5.5,1},
  {11,5.5,0},
  {11,4,1},
  {12,3,1},
  {13,3,1},
  {12,1.5,0},
  {11.5,2,0},
  {11.5,3,1},
  {10.75,3.5,1},
  {10.75,5.5,1},
  {10.5,5.5,0},
  {10.5,3,1},
  {11,2.5,1},
  {11,1.5,1},
  {11.5,1,1},
  {9.5,0.5,0},
  {10,1,1},
  {10,1.5,1},
  {10.25,2,1},
  {10.25,5.5,1},
  {6,4.5,0},
  {6,6,1},
  {6.5,6.5,1},
  {6.5,7,1},
  {5,7,0},
  {5.5,6.5,1},
  {5.5,6,1},
  {5.75,5.5,1},
  {5.75,4.5,1},
  {5.5,4.5,0},
  {5.5,5.5,1},
  {5,6,1},
  {4,6,1},
  {3.5,6.5,1},
  {4,5,0},
  {4.5,5.5,1},
  {5,5.5,1},
  {5.25,5,1},
  {5.25,4.5,1},
  {5,4.25,0},
  {4,4.25,1},
  {3.5,5,1},
  {2.5,5,1},
  {3,4.5,0},
  {3.5,4,1},
  {5,4,1},
  {5,3.75,0},
  {3.5,3.75,1},
  {3,3.5,1},
  {2.5,3.5,1},
  {2,2.5,0},
  {2.5,3,1},
  {3,3,1},
  {3.5,3.5,1},
  {5,3.5,1},
  {0,0,0}};
  
// create the stepper motor objects
ESP_FlexyStepper stepper_X;
ESP_FlexyStepper stepper_Y;

//create servo motor object
Servo servo_pen;

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo_pen.setPeriodHertz(50);  
  servo_pen.attach(SERVO_PIN, 500, 2400); 
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN_X, INPUT);
  pinMode(SWITCH_PIN_Y, INPUT);
  Serial.begin(115200);
  // connect and configure the stepper motors to IO pins
  stepper_X.connectToPins(MOTOR_STEP_PIN_X, MOTOR_DIRECTION_PIN_X);
  stepper_Y.connectToPins(MOTOR_STEP_PIN_Y, MOTOR_DIRECTION_PIN_Y);
  Serial.begin(9600);
  while (! Serial);
  delay(5000);
  servo_pen.write(UP);
  delay(1000);
  X_set();
  Y_set();
  
  //square_test();
  servo_pen.write(DOWN);
  delay(1000);
  servo_pen.write(UP);
  line_test();
}

void loop() 
{}

bool X_set()
{
  while(debounce(SWITCH_PIN_X)== HIGH)
  {
    stepper_X.setSpeedInStepsPerSecond(1600);
    stepper_X.setAccelerationInStepsPerSecondPerSecond(1600);
    stepper_X.moveRelativeInSteps(-80);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
  stepper_X.moveRelativeInSteps(160);
  X=0;
  return true;
  
}
bool Y_set()
{
  while(debounce(SWITCH_PIN_Y)== HIGH)
  {
    stepper_Y.setSpeedInStepsPerSecond(1600);
    stepper_Y.setAccelerationInStepsPerSecondPerSecond(1600);
    stepper_Y.moveRelativeInSteps(-80);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
  stepper_Y.moveRelativeInSteps(160);
  Y=0;
  return true;
}

void line_test()
{
  Serial.begin(9600);
  delay(1000);
  servo_pen.write(UP);
  P=0;
  float Xset,Yset;
  int Pset;
  int Xtravel,Ytravel;
  for(int i=0;i<178;i++)
  {
    /*
	//Manual input
	
    Serial.print("x=");
    while (Serial.available() == 0) {}  
    Xset=Serial.parseFloat();
    byte discard = Serial.read();
    Serial.println(Xset);
    
    Serial.write("y=");
    while (Serial.available() == 0) {}  
    Yset=Serial.parseFloat();
    discard = Serial.read();
    Serial.println(Yset);
    
    Serial.write("p=");
    while (Serial.available() == 0) {}  
    Pset=Serial.parseFloat();
    discard = Serial.read();
    Serial.println(Pset);
    */
    Xset=plot[i][0];
    Yset=plot[i][1];
    Pset=plot[i][2];
    
    if(P!=Pset)
    {
      delay(1000);
      if(Pset==1)
        servo_pen.write(DOWN);
      else
        servo_pen.write(UP);
      delay(1000);
      P=Pset;
    }
    
    Xtravel=(Xset-X)*400;
    Ytravel=(Yset-Y)*400;
    Serial.print("X:");
    Serial.println(Xset);
    Serial.print("Y:");
    Serial.println(Yset);
    
    if(Xtravel!=0 || Ytravel!=0)
      if(Xtravel==0)
      {
        if(Ytravel>0)
          digitalWrite(MOTOR_DIRECTION_PIN_Y,LOW);
        else
          digitalWrite(MOTOR_DIRECTION_PIN_Y,HIGH);
        for(int i = 0; i < abs(Ytravel); i++) 
        {
          digitalWrite(MOTOR_STEP_PIN_Y,HIGH);
          delayMicroseconds(500); 
          digitalWrite(MOTOR_STEP_PIN_Y,LOW);
          delayMicroseconds(500); 
        }   
        X=Xset;
        Y=Yset;        
      }
      else if(Ytravel==0)
      {
        if(Xtravel>0)
          digitalWrite(MOTOR_DIRECTION_PIN_X,LOW);
        else
          digitalWrite(MOTOR_DIRECTION_PIN_X,HIGH);
        for(int i = 0; i < abs(Xtravel); i++) 
        {
          digitalWrite(MOTOR_STEP_PIN_X,HIGH);
          delayMicroseconds(500); 
          digitalWrite(MOTOR_STEP_PIN_X,LOW);
          delayMicroseconds(500); 
        }   
        X=Xset;
        Y=Yset; 
      }
      else
      {
        if(Xtravel>0)
          digitalWrite(MOTOR_DIRECTION_PIN_X,LOW);
        else
          digitalWrite(MOTOR_DIRECTION_PIN_X,HIGH);
        if(Ytravel>0)
          digitalWrite(MOTOR_DIRECTION_PIN_Y,LOW);
        else
          digitalWrite(MOTOR_DIRECTION_PIN_Y,HIGH);
        if(abs(Xtravel)>abs(Ytravel))
        { 
          int dif=abs(Xtravel)%abs(Ytravel);
          for(int i = 0; i < abs(Ytravel); i++) 
          {
            digitalWrite(MOTOR_STEP_PIN_Y,HIGH);
            delayMicroseconds(500); 
            digitalWrite(MOTOR_STEP_PIN_Y,LOW);
            delayMicroseconds(500); 
            for(int j = 0; j < abs(Xtravel)/abs(Ytravel); j++)
            {
              digitalWrite(MOTOR_STEP_PIN_X,HIGH);
              delayMicroseconds(500); 
              digitalWrite(MOTOR_STEP_PIN_X,LOW);
              delayMicroseconds(500); 
            }
            
            if(abs(Xtravel)%abs(Ytravel)!=0 && dif>0)
              if(i%( abs(Ytravel)/(abs(Xtravel)%abs(Ytravel)))==0)
              {
                digitalWrite(MOTOR_STEP_PIN_X,HIGH);
                delayMicroseconds(500); 
                digitalWrite(MOTOR_STEP_PIN_X,LOW);
                delayMicroseconds(500);
                dif--;
              }
          }
          X=Xset;
          Y=Yset;
        }
        else
        {
          int dif=abs(Ytravel)%abs(Xtravel);
          for(int i = 0; i < abs(Xtravel); i++) 
          {
            digitalWrite(MOTOR_STEP_PIN_X,HIGH);
            delayMicroseconds(500); 
            digitalWrite(MOTOR_STEP_PIN_X,LOW);
            delayMicroseconds(500); 
            for(int j = 0; j < abs(Ytravel)/abs(Xtravel); j++)
            {
              digitalWrite(MOTOR_STEP_PIN_Y,HIGH);
              delayMicroseconds(500); 
              digitalWrite(MOTOR_STEP_PIN_Y,LOW);
              delayMicroseconds(500); 
            }
            if(abs(Ytravel)%abs(Xtravel)!=0&& dif>0)
              if(i%( abs(Xtravel)/(abs(Ytravel)%abs(Xtravel)))==0)
              {
                digitalWrite(MOTOR_STEP_PIN_Y,HIGH);
                delayMicroseconds(500); 
                digitalWrite(MOTOR_STEP_PIN_Y,LOW);
                delayMicroseconds(500);
                dif--;
              }
          }
          X=Xset;
          Y=Yset;
        }
        
      }
  }
}

void square_test()
{int i=0;
int j=0;
  delay(1000);
  servo_pen.write(DOWN);
  delay(1000);
  stepper_Y.setSpeedInStepsPerSecond(1600);
  stepper_Y.setAccelerationInStepsPerSecondPerSecond(1600);
  
  stepper_Y.moveRelativeInSteps(7500);
  
  delay(1000);
  servo_pen.write(UP);
  delay(1000);
  servo_pen.write(DOWN);
  delay(1000);
  stepper_X.setSpeedInStepsPerSecond(1600);
  stepper_X.setAccelerationInStepsPerSecondPerSecond(1600);
  
  stepper_X.moveRelativeInSteps(5700);
  
  delay(1000);
  servo_pen.write(UP);
  delay(1000);
  servo_pen.write(DOWN);
  delay(1000);
   
  stepper_Y.moveRelativeInSteps(-7500);
 
  delay(1000);
  servo_pen.write(UP);
  delay(1000);
  servo_pen.write(DOWN);
  delay(1000);
  
  stepper_X.moveRelativeInSteps(-5700);
 
  delay(1000);
  servo_pen.write(UP);
  delay(1000);
}

int debounce(int pin)
{
  int state, previous_state;
  previous_state = digitalRead(pin); 
  for(int i = 0; i < 25; i++)
  {
    delay(1);                       
    state = digitalRead(pin);       
    if( state != previous_state)
    {
      i = 0;                  
      previous_state = state;       
    }
  }
  return state;
}
