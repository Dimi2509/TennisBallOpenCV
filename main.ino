#include <NewPing.h>

#define TRIGGER_PIN 10
#define ECHO_PIN 11
#define MAX_DIST 60

int distance;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DIST);

const int L1 = 7;
const int L2 = 6;
const int R1 = 5;
const int R2 = 4;
const int ENA = 9; //ENA and ENB are connected where the jumper is, the jumper makes the motors go max speed
const int ENB = 3;

const int FW = 8;

int x;
int y;
String sx, sy;
String data;

//Function prototypes
String getValue(String data, char separator, int index);
void moveForward(void);
void turnLeft(void);
void turnRight(void);
void stopCar(void);
void searchBall(void);

unsigned int Speed = 60;

int x_boundary_1 = 110;
int x_boundary_2 = 210;
int y_boundary = 130;


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(2); // 2 ms timeout response
  
  
  pinMode(L1,OUTPUT);
  pinMode(L2,OUTPUT);
  pinMode(R1,OUTPUT);
  pinMode(R2,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(FW,OUTPUT);
  
  
}

void loop() {
  distance = sonar.ping_cm();
  
  while(distance < MAX_DIST && distance > 1){
        stopCar();
        distance = sonar.ping_cm();
    }
  

  if (Serial.available() > 0){
      data = Serial.readString(); // Read string  
      sx = getValue(data,'.',0);
      sy = getValue(data,'.',1);
      x = sx.toInt();
      y = sy.toInt();   
      
      if(x > x_boundary_2){
        speedControl(35,15);
        if(y > y_boundary){
          digitalWrite(FW, HIGH);        
        }
        else{
          digitalWrite(FW, LOW);
        }
      }
      
      if(x < x_boundary_1){
        speedControl(15,35);
        if(y > y_boundary){
          digitalWrite(FW, HIGH);        
        }
        else{
          digitalWrite(FW, LOW);
        }
      }
    
      else{
        speedControl(30,30);
        if(y > y_boundary){
          digitalWrite(FW, HIGH);        
        }
        else{
          digitalWrite(FW, LOW);
        }
      }
   }
  else{
    
    stopCar();
  }
  




     
  
}


void moveForward(void){
  digitalWrite(L1,HIGH);
  digitalWrite(L2,LOW);
  digitalWrite(R1,HIGH);
  digitalWrite(R2,LOW);
  
  analogWrite(ENA,Speed);
  analogWrite(ENB,Speed);
  delay(1000);
}

void turnLeft(void){
  digitalWrite(L1,HIGH);
  digitalWrite(L2,LOW);
  digitalWrite(R1,HIGH);
  digitalWrite(R2,LOW);
  
  analogWrite(ENA,Speed-15);
  analogWrite(ENB,Speed+30);
}

void turnRight(void){
  stopCar();
  digitalWrite(L1,HIGH);
  digitalWrite(L2,LOW);
  digitalWrite(R1,HIGH);
  digitalWrite(R2,LOW);
  
  analogWrite(ENA,Speed+30);
  analogWrite(ENB,Speed-15);

  
}

void stopCar(void){
  digitalWrite(L1,LOW);
  digitalWrite(L2,LOW);
  digitalWrite(R1,LOW);
  digitalWrite(R2,LOW);
  digitalWrite(FW, LOW);  
  
  analogWrite(ENA,0);
  analogWrite(ENB,0);

}


void searchBall(void){
  digitalWrite(L1,HIGH);
  digitalWrite(L2,LOW);
  digitalWrite(R1,LOW);
  digitalWrite(R2,HIGH);
  
  analogWrite(ENA,Speed);
  analogWrite(ENB,Speed);
}



void speedControl(int A, int B){
  
  digitalWrite(L1,HIGH);
  digitalWrite(L2,LOW);
  digitalWrite(R1,HIGH);
  digitalWrite(R2,LOW);

  analogWrite(ENA,A = A*2.55);
  analogWrite(ENB,B = B*2.55);
  delay(150);
}


String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
