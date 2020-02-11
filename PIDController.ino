#include <Servo.h>
#include <inttypes.h>
#include <stdlib.h>


#define r3 1.73205 // square root of 3

#define servopin1 6
#define servopin2 5
#define servopin3 3

#define outMax 2100-1500
#define outMin -1500+1250

char controlByte = '0';

Servo servo1; //servo on y axis
Servo servo2; // x right servo
Servo servo3; // x left servo

double xCor, yCor; //Coordinates from touchscreen
double xO, yO;

double error[3];//holds current errror
double lastError[3] = {0,0,0};
double lastLastError[3] = {0,0,0};

double output[3] = {0,0,0}; //Hold actual output

double ITerm[3] = {0,0,0};
double lastDVal[3] = {0,0,0};
double lastLastDVal[3] = {0,0,0};
unsigned long now, deltaT, lastT;

double setkp[3] = {0.3,0.5,0.5};
double setki[3] = {0.15,0.2,0.2};
double setkd[3] = {0.1,0.14,0.14};
double setka[3] = {1,1,1};

double kp[3], ki[3], kd[3], ka[3]; // working tunings
int compT; //computation time in ms

bool compute_bool = true, computeDone = true, setPointChange = false, isBall = false;

int auxTimeCount = 100, methodCount = 0;

int squareCoords[4][2] = {{380,360}, {360,185}, {205, 135}, {180, 310}};
int lineCoords[2][2] = {{259,146}, {285,377}};
int triangleCoords[3][2] = {{272,246},{408-30,328-30},{372-30,165+30}};

void setup() {
  // put middleYur setup code here, to run once:
  Serial.begin(115200);

  servo1.attach(servopin1);
  servo2.attach(servopin2);
  servo3.attach(servopin3);
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  compT = 10;
  xO = 250; 
  yO = 229;
  setPIDTunings(true);
}

void loop() {
  // put middleYur main code here, to run repeatedly:
  readTouchData();
  if(isBall){
    if(compute_bool){
      if(controlPID()){
        writeServos();
        computeDone = true;
      }
    }
    if(computeDone){
      receiveSerial();
      switch(controlByte){
        case('1'):
          auxTimeCount++;
          Serial.println(auxTimeCount);
          if(auxTimeCount > 100){
            square();
            auxTimeCount = 0;
            setPointChange = true;
          }else{
            break;
          } 
        case('2'):
          auxTimeCount++;
          if(auxTimeCount > 25){
            line();
            auxTimeCount = 0;
            setPointChange = true;
          }else{
            break;
          }
        case('3'):
          auxTimeCount++;
          if(auxTimeCount > 25){
            triangle();
            auxTimeCount = 0;
            setPointChange = true;
          }else{
            break;
          }
      }
    }
  }
}

void readTouchData(){
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(A2, LOW);
  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);
  pinMode(A0, OUTPUT);
  digitalWrite(A0, LOW);
  xCor = analogRead(A3);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  digitalWrite(A0, LOW);
  pinMode(A3, OUTPUT);
  digitalWrite(A3, HIGH);
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
  yCor = analogRead(A1);
  // printCoords();
  if(xCor > 95 && yCor > 40){
    isBall = true;
  }else{
    isBall = false;
    readTouchData();
  }
}

void limitOutput(int i){
 
  if(output[i] > outMax){
    output[i] = outMax;
  }
  else if(output[i] < outMin){
    output[i] = outMin;
  }
  if(ITerm[i] > outMax){
    ITerm[i] = outMax;
  }
  else if(ITerm[i] < outMin ){
    ITerm[i] = outMin;
  }
}

bool controlPID(){
  now = millis();
  deltaT = now - lastT;
  if(isBall){
    for(int i = 0; i<3; i++){
      readTouchData();
      computeError(i);
      computePID(i);
      limitOutput(i);
    }
    lastT = now;
    return true;
  }else{
    return false;
  }
}

void computeError(int i){
  if (i == 0){
    error[0] = 1.1*(yCor - yO);
  }
  if (i == 1){
    error[1] = (r3*(xCor - xO) + (yO - yCor))/2.0;
  }
  if (i == 2){
    error[2] = (r3*(xO - xCor) + (yO - yCor))/2.0;
  }
  printError(i);
}

void computePID(int i){
  Serial.println("/////COMPUTE//////");
  
  // calculate each PID term individually:
  double PTerm = (error[i] * kp[i]);
  ITerm[i] += (ki[i] * error[i]);
  double DVal = (error[i] - lastError[i]);
  double DTerm = kd[i] * (((5*DVal) + (3*lastDVal[i]) + (2*lastLastDVal[i]))/48.0);
  double ATerm = ka[i] * (((DVal - lastDVal[i]) + (lastDVal[i] - lastLastDVal[i]))/2.0);
  Serial.print("ERROR:");
  Serial.println(error[i]);
  Serial.print("LASTERROR:");
  Serial.println(lastError[i]);
  Serial.print("DVAL:");
  Serial.println(DVal);
  Serial.print("LASTDVAL:");
  Serial.println(lastDVal[i]); 
  Serial.print("LASTLASTDVAL:");
  Serial.println(lastLastDVal[i]);  
  Serial.print("PTERM:");
  Serial.println(PTerm);
  Serial.print("DTERM:");
  Serial.println(DTerm);  
  Serial.print("ATERM:");
  Serial.println(ATerm);  
  Serial.print("ITERM:");
  Serial.println(ITerm[i]);  
       
  // Calculate Output:
  output[i] = PTerm + ITerm[i] + DTerm + ATerm;
  Serial.print("OUTPUT:");
  Serial.println(output[i]); 
  // save some calculations for later:
  lastError[i] = error[i];
  lastLastDVal[i] = lastDVal[i];
  lastDVal[i] = DVal;
}

void writeServos(){
  Serial.println("WRITE//////////////////////////////");
  Serial.println(output[0]);
  Serial.println(output[1]);
  Serial.println(output[2]);
  Serial.println("END WRITE///////////////////////");
  servo1.writeMicroseconds(1500 + output[0]); //1500 cambiar outmax
  servo2.writeMicroseconds(1500 + output[1]);
  servo3.writeMicroseconds(1500 + output[2]);
}

void setPIDTunings(bool aggressive){
  if(!aggressive){
    for(int i = 0; i < 3; i++)
    {
      kp[i] = setkp[i];
      kd[i] = setkd[i] / (compT/1000.0);
      ki[i] = setki[i] * (compT/1000.0);
      ka[i] = setka[i] * (compT/1000.0);
    }
  }
  else{
    for(int i = 0; i < 3; i++)
    {
      kp[i] = setkp[i];
      kd[i] = setkd[i] / (compT/1000.0);
      ki[i] = setki[i] * (compT/1000.0);
      ka[i] = setka[i] * (compT/1000.0);
    }
  }
}
/* Changing setpoint to square */

void square(){
  Serial.println("SQUare has been summoned!");
  xO = squareCoords[methodCount][0];
  yO = squareCoords[methodCount][1];
  Serial.println("XO");
  Serial.println(xO);
  Serial.println("YO");
  Serial.println(yO);
  methodCount++;
  if(methodCount > 3){
    methodCount = 0;
  }
}

void line(){
  Serial.println("Line has been summoned!");
  xO = lineCoords[methodCount][0];
  yO = lineCoords[methodCount][1];
  methodCount++;
  if(methodCount > 1){
    methodCount = 0;
  }
}

/* Changing setpoint to triangle */

void triangle(){
  Serial.println("Triangle has been summoned!");
  xO = triangleCoords[methodCount][0];
  yO = triangleCoords[methodCount][1];
  methodCount++;
  if(methodCount > 2){
    methodCount = 0;
  }
}

/* Methods for changing state */
bool receiveSerial()
{
  if(Serial.available() > 0)
  {
    Serial.print("Aqui hay algo");
    int incomingByte = Serial.read();
    controlByte = (char)incomingByte;
    Serial.println(controlByte);
    return true;
    
  }
  else return false;
}

void printCoords(){
  Serial.print("x=");
  Serial.print(xCor);
  Serial.print("       y=");
  Serial.print(yCor);
  Serial.println();
}

void printError(int i){
  Serial.println("ERROR ///////////////////////////");
  Serial.print(i);
  Serial.println(error[i]);
  Serial.println("YCor//////////////////////////");
  Serial.println(yCor);
  Serial.println("YO///////////////////////////");
  Serial.println(yO);
  Serial.println("XCor//////////////////////////");
  Serial.println(xCor);
  Serial.println("xO///////////////////////////");
  Serial.println(xO);
}
