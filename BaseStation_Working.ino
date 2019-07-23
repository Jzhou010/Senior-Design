//This code has autonomous mode
// Change altitude
//Change Lift distance (513) to target from 3 to 2 (263)
#include <SPI.h>
#include <Keypad.h>
#include <LiquidCrystal.h>
#include <TinyGPS++.h>
#include <nRF24L01.h>
#include "RF24.h"
#define Pi 3.14159

float MagMinX = 0, MagMaxX = 0;//Compass global variabls
float MagMinY = 0, MagMaxY = 0;
float XOffset = 0, YOffset = 0;
double des_LAT = 0;
double des_LNG = 0;
double home_LAT = 0;
double home_LNG = 0;
RF24 radio(15, 16); //CE, CSN  //Antena connection
byte addresses[][6] = {"0"};

struct package
{
  float latt = 0.0;
  float lonn = 0.0;
  float magneticX = 0.0;
  float magneticY = 0.0;
  long sonarDistance = 0;  //Can We call this sonarDistance
  bool GPS_status = false;
  bool compass_status = false;
  bool Sonar_status = false;
};

typedef struct package Package;
Package data;

int RadioCounter = 0;
const byte ROWS = 4;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] =
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {A3, A2, A1, A0};  //connect to the row pinouts of the keypad
byte colPins[COLS] = {A7, A6, A5, A4}; //connect to the column pinouts of the keypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
LiquidCrystal lcd(38, 36, 28, 30, 32, 34);

static char keyPress = 0;      //Used on KeyPad Functionh
int x = 0;                    //Used on Timer Function
unsigned long long times = 0;//Used on Timer Function
int counter = 0;            //Used on Timer Function
int rfCounter = 0;
const int gpsLED =  22;          //LED
const int compassLED =  23;     //LED
const int rfLED =  24;         //LED
const int autonomousLED =  25;//LED
const int manualLED =  26;   //LED
const int userCommandLED = 27;
int LeftJoyStickVertical = 2;     // This pin will control Vertical motions
int LeftJoyStickSpin = 3;        // This pin will control the Spin motions
int RightJoyStickBackForth = 4; // This pin will control Back and Forth motions
int RightJoyStickLeftRight = 5;// This pin will control Left and Right motions
//===1Spin===2Vertical===3Forward/Backward===4Left/Right

short distanceToDestination = 0;//Action Valriables
short distanceToHome = 0;
short lastDistanceToDestination = 0;
short lastDistanceToHome = 0;
short distanceDifference = 0;
short courseToDestination = 0;
short courseToHome = 0;
short compassdegree = 0;
double degreeDifference = 0;

bool peripheralCheck = false;
bool rfConnection = false;
bool point = false;
bool negative = false;
bool degreeSet = true;
bool goingHome = false;
bool getHomeCoorCounter = 0;
bool distanceIs9Away = false;
bool proceed = false;
int numberOfTargets = 0;
float LatCoordinate[3] = {0};
float LonCoordinate[3] = {0};
int arrayCounter = 0;

float large = 0;//Coordinate Input Variables
float small = 0;
float smallLarge = 0;
float smallTemp = 0;
float numba = 0;
float lastNumba = 0;
float temp1 = 0;
float base = 10;
float exponent = 0;
float latt = 0;
float lonn = 0;
unsigned long power = 0;

void setup()
{
  lcd.clear();
  setUpLoop();
  delay(5000);
  stationaryAction();
  delay(1000);
  flashLED();
}
void loop()
{
  State_machine();
}
//**************************************************************************State_Machine******************************************************************************
enum State{Init,ChooseMode,ManualMode,CheckPeripherals,ChooseNumOfTargets,EnterCoordinates,Lift,Calibrate,SetOrientation,OnCourse,Hover,AutoMode,Descend}state; 
void State_machine() 
{
  switch (state) //***********************************************************Transitions******************************************************************************
  {
    case Init:
      lcd.clear();
      state = ChooseMode;
      break;

    case ChooseMode:
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (keyPress == 'D')
      {
        state = CheckPeripherals;
        lcd.clear();
      }
      else
      {
        state = ChooseMode;
      }
      //lcd.clear();
      break;

    case CheckPeripherals:
      GetChar();
      peripheralCheckFunction();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (peripheralCheck)
      {
        state = ChooseNumOfTargets;
      }
      else if (!peripheralCheck)
      {
        state = CheckPeripherals;
      }
      lcd.clear();
      break;

    case ChooseNumOfTargets:
          GetChar();
          if(keyPress == 'C') 
          {
             state = ManualMode;
             lcd.clear();
          }
          else if(keyPress == '#')
          {
            state = EnterCoordinates;
          }
          else if(keyPress == 'D')
          {
            state = ChooseNumOfTargets;
          }
          lcd.clear();
    break;  

    case EnterCoordinates:
          GetChar();
          if(keyPress == 'C') 
          {
             state = ManualMode;
             lcd.clear();
          }
          else if(proceed)
          {
            state = Lift;
          }
          else if(!proceed)
          {
            state = EnterCoordinates;
          }
          lcd.clear();
    break;

    case Lift:
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (peripheralCheck)
      {
        state = Calibrate;
      }
      lcd.clear();
      break;

    case Calibrate:
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (peripheralCheck)
      {
        state = SetOrientation;
        des_LAT = LatCoordinate[arrayCounter];
        des_LNG = LonCoordinate[arrayCounter];
      }
      lcd.clear();
      break;

    case SetOrientation:
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (degreeSet)
      {
        state = OnCourse;
        if (!goingHome)
        {
          lastDistanceToDestination = getDistanceToDes();
        }
        if (goingHome)
        {
          lastDistanceToHome = getDistanceToHome();
        }
      }
      else if (!degreeSet)
      {
        state = SetOrientation;
      }
      lcd.clear();
      break;

    case OnCourse:
      if (goingHome)
      {
        if (distanceToHome < 3)
        {
          state = Hover;
          lcd.clear();
        }
        else if (distanceToHome >= 3)
        {
          lcd.setCursor(0, 0);
          lcd.print("Going Home");
          lcd.setCursor(0, 1);
          lcd.print("Distance:");
          lcd.setCursor(9, 1);
          lcd.print(distanceToHome);
          if(distanceToHome == 10)
          {
            lcd.clear();
          }
          getData();
          compassdegree = ComCourse();
          courseToHome = getCourseToHome();
          degreeDifference = abs(compassdegree - courseToHome);
          state = OnCourse;
        }
      }

      if (!goingHome)
      {
        if (distanceToDestination < 3)
        {
          state = Hover;
          lcd.clear();
          //goingHome = true;
        }
        else if (distanceToDestination >= 3)
        {
          lcd.setCursor(0, 0);
          lcd.print("To point:");
          lcd.setCursor(9, 0);
          lcd.print(arrayCounter + 1);
          lcd.setCursor(0, 1);
          lcd.print("Distance:");
          lcd.setCursor(9, 1);
          lcd.print(distanceToDestination);
          if(distanceToDestination == 10)
          {
            lcd.clear();
          }
          //delay(10);
          
          getData();
          compassdegree = ComCourse();
          courseToDestination = getCourseToDes();
          degreeDifference = abs(compassdegree - courseToDestination);
          state = OnCourse;
        }
      }
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
  
      break;

    case Hover:
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (goingHome)
      {
        state = Descend;
      }
      else if (!goingHome)
      {
        state = Calibrate;
        if((arrayCounter + 1) == numberOfTargets)
        {
          goingHome = true;
        }
        arrayCounter = arrayCounter + 1;
      }
      else
      {
        state = Hover;
      }
      break;

    case ManualMode:
      GetChar();
      peripheralCheckFunction();
      if (keyPress == 'D' && peripheralCheck)
      {
        if(data.sonarDistance < 10)
        {
         state = CheckPeripherals;
        }
        else 
        {
          state = Calibrate;
        }
        lcd.clear();
      }
      else
      {
        state = ManualMode;
      }
      break;

    case Descend:
      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        lcd.clear();
      }
      else if (keyPress == 'D')
      {
        state = CheckPeripherals;
        lcd.clear();
      }
      break;

    default:
      state = ChooseMode;
      break;
  }
  switch (state)//********************************************************************Actions************************************************************************
  {
    case Init:
      break;

    case ChooseMode:
      peripheralCheckFunction();
      lcd.setCursor(0, 0);
      lcd.print("  Choose  Mode");
      lcd.setCursor(0, 1);
      lcd.print("C:Manual  D:Auto");
      break;

    case CheckPeripherals:
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case ChooseNumOfTargets:
          chooseNumOfTargetsFunction();
          digitalWrite(manualLED,LOW);
          digitalWrite(userCommandLED,LOW);
          digitalWrite(autonomousLED,HIGH);
    break;

    case EnterCoordinates:
          enterCoordinateFunction();
          digitalWrite(manualLED,LOW);
          digitalWrite(userCommandLED,LOW);
          digitalWrite(autonomousLED,HIGH);
    break;  

    case Lift:
      home_LAT = data.latt;
      home_LNG = data.lonn;
      liftFunction();
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case Calibrate:
      calibrateFunction();
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case SetOrientation:
      
      setOrientationFunction();
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case OnCourse:
      onCourseFunction();
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case Hover:
      hoverFunction();
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case Descend:
      descendFunction();
      peripheralCheckFunction();
      digitalWrite(manualLED, LOW);
      digitalWrite(userCommandLED, LOW);
      digitalWrite(autonomousLED, HIGH);
      break;

    case ManualMode:
      Manual();
      //peripheralCheckFunction();
      digitalWrite(manualLED, HIGH);
      digitalWrite(autonomousLED, LOW);
      break;
  }
}

//**********************************************************************Drone Motions******************************************************************************************
void liftFunction()
{
  keyPress = 0;
  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Turning on motor");
  while (counter < 5 && keyPress != 'C')
  {
    counter = TimeCounter(counter);
    turnOnAction();
    GetChar();
    if (keyPress == 'C')
    {
      state = ManualMode;
    }
  }

  counter = 0;
  while (counter < 3 && keyPress != 'C')
  {
    counter = TimeCounter(counter);
    stationaryAction();
    GetChar();
    if (keyPress == 'C')
    {
      state = ManualMode;
    }
  }

  counter = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Taking off");
  while (counter < 5 && keyPress != 'C')
  {
    counter = TimeCounter(counter);
    upAction();

    GetChar();
    if (keyPress == 'C')
    {
      state = ManualMode;
    }
  }

  counter = 0;
  lcd.clear();
  while (counter < 3 && keyPress != 'C')
  {
    counter = TimeCounter(counter);
    stationaryAction();

    GetChar();
    if (keyPress == 'C')
    {
      state = ManualMode;
    }
  }
  counter = 0;
}

void calibrateFunction()
{
  MagMinX = 0.0;
  MagMaxX = 0.0;
  MagMinY = 0.0;
  MagMaxY = 0.0;
  counter = 0;

  while (counter < 7)
  {
    compassCalibration();
    peripheralCheckFunction();
    lcd.setCursor(0, 0);
    lcd.print("Calibrating");
    counter = TimeCounter(counter);
    spinLeftAction();
    Serial.println(counter);

    GetChar();
    if (keyPress == 'C')
    {
      state = ManualMode;
      counter = 13;
    }
  }

  counter = 0;
}

void setOrientationFunction()
{
  degreeSet = false;
  if (!goingHome)
  {
    getData();
    compassdegree = ComCourse();
    courseToDestination = getCourseToDes();
    degreeDifference = compassdegree - courseToDestination;
    while (abs(degreeDifference) > 2)
    {
      peripheralCheckFunction();
      getData();
      compassdegree = ComCourse();
      courseToDestination = getCourseToDes();
      degreeDifference = compassdegree - courseToDestination;
      lcd.setCursor(0, 0); lcd.print("Setting");
      lcd.setCursor(0, 1); lcd.print("Orientation");
      lcd.setCursor(12, 1); lcd.print(abs(degreeDifference));

      if (degreeDifference < 0 && abs(degreeDifference) < 180) {
        spinSlowRightAction();
        Serial.println("Spinning Right");
      }
      else if (degreeDifference < 0 && abs(degreeDifference) > 180) {
        spinSlowLeftAction();
        Serial.println("Spinning Left");
      }
      else if (degreeDifference > 0 && abs(degreeDifference) < 180) {
        spinSlowLeftAction();
        Serial.println("Spinning Left");
      }
      else if (degreeDifference > 0 && abs(degreeDifference) > 180) {
        spinSlowRightAction();
        Serial.println("Spinning Right");
      }

      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        degreeDifference = 1;
      }
    }
  }

  if (goingHome)
  {
    getData();
    compassdegree = ComCourse();
    courseToHome = getCourseToHome();
    degreeDifference = compassdegree - courseToHome;
    while (abs(degreeDifference) > 2)
    {
      getData();
      compassdegree = ComCourse();
      courseToHome = getCourseToHome();
      degreeDifference = compassdegree - courseToHome;
      lcd.setCursor(0, 0); lcd.print("Setting");
      lcd.setCursor(0, 1); lcd.print("Orientation");
      lcd.setCursor(12, 1); lcd.print(abs(degreeDifference));

      if (degreeDifference < 0 && abs(degreeDifference) < 180) {
        spinSlowRightAction();
      }
      else if (degreeDifference < 0 && abs(degreeDifference) > 180) {
        spinSlowLeftAction();
      }
      else if (degreeDifference > 0 && abs(degreeDifference) < 180) {
        spinSlowLeftAction();
      }
      else if (degreeDifference > 0 && abs(degreeDifference) > 180) {
        spinSlowRightAction();
      }

      GetChar();
      if (keyPress == 'C')
      {
        state = ManualMode;
        degreeDifference = 1;
      }
    }
  }
  if (abs(degreeDifference) < 2)
  {
    degreeSet = true;
  }

  GetChar();
  if (keyPress == 'C')
  {
    state = ManualMode;
    degreeDifference = 1;
  }
}

void onCourseFunction()
{
  if (!goingHome)
  {
    distanceToDestination = getDistanceToDes();
    counter = TimeCounter(counter);

    if (counter > 2)
    {
      counter = 0;
      lastDistanceToDestination = getDistanceToDes();
    }

    if ((!distanceIs9Away) && (distanceToDestination > 9) && (distanceToDestination < 10) && (lastDistanceToDestination >= distanceToDestination))
    {
      state = SetOrientation;
      distanceIs9Away = true;
    }

    if (lastDistanceToDestination >= distanceToDestination)
    {
      forwardAction();
    }
    else if ((lastDistanceToDestination + 1) < distanceToDestination)
    {
      state = Calibrate;
      distanceToDestination = 0;
    }
  }

  if (goingHome)
  {
    distanceToHome = getDistanceToHome();
    counter = TimeCounter(counter);
    if (counter > 2)
    {
      counter = 0;
      lastDistanceToHome = getDistanceToHome();
    }
    if ((!distanceIs9Away) && (distanceToHome > 9) && (distanceToHome < 10) && (lastDistanceToHome >= distanceToHome))
    {
      state = SetOrientation;
      distanceIs9Away = true;
    }
    if (lastDistanceToHome >= distanceToHome)
    {
      forwardAction();
    }
    else if ((lastDistanceToHome + 1) < distanceToHome)
    {
      state = Calibrate;
      distanceToHome = 0;
    }
  }
}

void descendFunction()
{
  getData();
  if (data.sonarDistance > 18)
  {
    downAction();
    lcd.setCursor(0,0);
    lcd.print("Landing");
  }
  else if (data.sonarDistance <= 18)
  {
    stationaryAction();
    delay(3000);
    turnOffAction();
    delay(4000);
    state = ManualMode;
  }
}
void hoverFunction()
{
  counter = 0;
  lcd.clear();
  while (counter < 13)
  {
    counter = TimeCounter(counter);
    stationaryAction();
    GetChar();
    if (keyPress == 'C')
    {
      state = ManualMode;
      counter = 8;
    }
    lcd.setCursor(0,0);
    lcd.print("Hovering");
  }
  degreeDifference = 0;
}

//***********************************************************************Manual********************************************************************************************
void Manual()
{
  lcd.clear();
  GetChar();
  if (keyPress == '1' && customKeypad.getState() == HOLD)
  {
    spinLeftAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Spin Left");
    delay(100);
  }
  else if (keyPress == '2' && customKeypad.getState() == HOLD)
  {
    forwardAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Forward");
    delay(100);
  }
  else if (keyPress == '3' && customKeypad.getState() == HOLD)
  {
    spinRightAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Spin Right");
    delay(100);
  }
  else if (keyPress == '4' && customKeypad.getState() == HOLD)
  {
    leftAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Left");
    delay(100);
  }
  else if (keyPress == '5' && customKeypad.getState() == HOLD)
  {
    backwardAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Back");
    delay(100);
  }
  else if (keyPress == '6' && customKeypad.getState() == HOLD)
  {
    rightAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Right");
    delay(100);
  }
  else if (keyPress == '7' && customKeypad.getState() == HOLD)
  {
    downAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Down");
    delay(100);
  }
  else if (keyPress == '8' && customKeypad.getState() == HOLD)
  {
    stationaryAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Stationary");
    delay(100);
  }
  else if (keyPress == '9' && customKeypad.getState() == HOLD)
  {
    upAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Up");
    delay(100);
  }
  else if (keyPress == 'A' && customKeypad.getState() == HOLD)
  {
    turnOnAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Turn ON");
    delay(100);
  }
  else if (keyPress == 'B' && customKeypad.getState() == HOLD)
  {
    turnOffAction();
    digitalWrite(userCommandLED, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("Turn OFF");
    delay(100);
  }
  else
  {
    stationaryAction();
    digitalWrite(userCommandLED, LOW);
    lcd.setCursor(0, 1);
    lcd.print("Stationary");
    delay(100);
  }
}
//****************************************************************************DRONE ACTIONS********************************************************************************
void turnOnAction()  
{
  analogWrite(LeftJoyStickVertical, 26);
  analogWrite(LeftJoyStickSpin, 138);
  analogWrite(RightJoyStickBackForth, 138);
  analogWrite(RightJoyStickLeftRight, 138);
}
void turnOffAction()
{
  analogWrite(LeftJoyStickVertical, 27);
  analogWrite(LeftJoyStickSpin, 83);
  analogWrite(RightJoyStickBackForth, 78);
  analogWrite(RightJoyStickLeftRight, 83);
}
void stationaryAction()
{
  analogWrite(LeftJoyStickVertical, 79);
  analogWrite(LeftJoyStickSpin, 85);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void upAction()
{
  analogWrite(LeftJoyStickVertical, 102);//Changed at the park. Used to be 115
  analogWrite(LeftJoyStickSpin, 85);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void downAction()
{
  analogWrite(LeftJoyStickVertical, 70);//Changed at the park. Used to be 51
  analogWrite(LeftJoyStickSpin, 85);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void spinLeftAction()
{
  analogWrite(LeftJoyStickVertical, 79);
  analogWrite(LeftJoyStickSpin, 51);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void spinSlowLeftAction()
{
  analogWrite(LeftJoyStickVertical, 79);
  analogWrite(LeftJoyStickSpin, 75);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void spinRightAction()
{
  analogWrite(LeftJoyStickVertical, 79);
  analogWrite(LeftJoyStickSpin, 115);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void spinSlowRightAction()
{
  analogWrite(LeftJoyStickVertical, 79);
  analogWrite(LeftJoyStickSpin, 95);
  analogWrite(RightJoyStickBackForth, 79);
  analogWrite(RightJoyStickLeftRight, 85);
}
void forwardAction()
{
  analogWrite(LeftJoyStickVertical, 79);
  analogWrite(LeftJoyStickSpin, 85);
  analogWrite(RightJoyStickBackForth, 60);
  analogWrite(RightJoyStickLeftRight, 85);
}
void backwardAction()
{
  analogWrite(LeftJoyStickVertical, 77);
  analogWrite(LeftJoyStickSpin, 84);
  analogWrite(RightJoyStickBackForth, 102);//Changed at the park. Used to be 115
  analogWrite(RightJoyStickLeftRight, 84);
}
void leftAction()
{
  analogWrite(LeftJoyStickVertical, 77);
  analogWrite(LeftJoyStickSpin, 84);
  analogWrite(RightJoyStickBackForth, 77);
  analogWrite(RightJoyStickLeftRight, 107);//Changed at the park. Used to be 115
}
void rightAction()
{
  analogWrite(LeftJoyStickVertical, 77);
  analogWrite(LeftJoyStickSpin, 84);
  analogWrite(RightJoyStickBackForth, 77);
  analogWrite(RightJoyStickLeftRight, 61);//Changed at the park. Used to be 51
}
//**********************************************************************Get Data******************************************************************************************
int getDistanceToDes()
{
  int destinationReturn = 0;
  destinationReturn = TinyGPSPlus::distanceBetween(data.latt, data.lonn, des_LAT, des_LNG);
  return destinationReturn;
}
int getDistanceToHome()
{
  int homeReturn = 0;
  homeReturn = TinyGPSPlus::distanceBetween(data.latt, data.lonn, home_LAT, home_LNG);
  return homeReturn;
}
int getCourseToDes()
{
  int courseDesReturn = 0;
  courseDesReturn = TinyGPSPlus::courseTo(data.latt, data.lonn, des_LAT, des_LNG);
  return courseDesReturn;
}
int getCourseToHome()
{
  int courseHomeReturn = 0;
  courseHomeReturn = TinyGPSPlus::courseTo(data.latt, data.lonn, home_LAT, home_LNG);
  return courseHomeReturn;
}
void getData()
{
  rfCounter = TimeCounter(rfCounter);
  if (radio.available())
  {
    while (radio.available() > 0 && radio.isChipConnected())
    {
      radio.read(&data, sizeof(data));
      rfConnection = true;
    }
    rfCounter = 0;
    Serial.println("it's runnning.");
  }
  else if (rfCounter > 3)
  {
    rfConnection = false;
  }
}

void peripheralCheckFunction()
{
  getData();
  if (data.GPS_status)
  {
    digitalWrite(gpsLED, HIGH);
  }
  if (data.compass_status)
  {
    digitalWrite(compassLED, HIGH);
  }
  if (rfConnection)
  {
    digitalWrite(rfLED, HIGH);
  }
  if (rfConnection && data.compass_status && data.GPS_status && data.Sonar_status)
  {
    peripheralCheck = true;
  }

  if (!data.GPS_status)
  {
    digitalWrite(gpsLED, LOW);
  }
  if (!data.compass_status)
  {
    digitalWrite(compassLED, LOW);
  }
  if (!rfConnection)
  {
    digitalWrite(rfLED, LOW);
    digitalWrite(compassLED, LOW);
    digitalWrite(gpsLED, LOW);
  }
  if (!rfConnection || !data.compass_status || !data.GPS_status)
  {
    peripheralCheck = false;
  }


  if (!getHomeCoorCounter)
  {
    // getData();
    home_LAT = data.latt;
    home_LNG = data.lonn;
    getHomeCoorCounter = true;
  }
}

float ComCourse()
{
  float heading = 0.0;
  // Calculate the angle of the vector y,x
  heading = (atan2(data.magneticY - YOffset, data.magneticX - XOffset) * 180) / Pi;
  // Normalize to 0-360
  if (heading > 0)
  {
    heading = heading + 90;
  }
  else if (heading < 0)
  {
    heading = 450 + heading;
    if (heading > 360)
    {
      heading = heading - 360;
    }
  }

  return heading;
}

void compassCalibration() 
{
  if (data.magneticX < MagMinX) MagMinX = data.magneticX;
  if (data.magneticX > MagMaxX) MagMaxX = data.magneticX;

  if (data.magneticY < MagMinY) MagMinY = data.magneticY;
  if (data.magneticY > MagMaxY) MagMaxY = data.magneticY;

  if (MagMinX + MagMaxX > 1.00 || MagMinX + MagMaxX < -1.00)
    {
      XOffset = (MagMinX + MagMaxX) / 2;
    }

  if (MagMinY + MagMaxY > 1.00 || MagMinY + MagMaxY < -1.00)
    {
      YOffset = (MagMinY + MagMaxY) / 2;
    }
}
//********************************************************************Choose # of Targets/Enter Coordinate***************************************************************************************
void chooseNumOfTargetsFunction()
{
  delay(2000);
  
  GetChar();
  while(keyPress < '0'  || keyPress > '4')
  {
   GetChar();
   lcd.setCursor(0,0);
   lcd.print("How many targets");
   lcd.setCursor(0,1);
   lcd.print("are we visiting?");
  }
  if(keyPress > '0'  && keyPress <= '4')
  {
   numberOfTargets = keyPress - 48; 
   state = EnterCoordinates;
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("You selected");
   lcd.setCursor(13,0);
   lcd.print(numberOfTargets);
   lcd.setCursor(0,1);
   lcd.print("targets");
   delay(2000);
  }
}
void enterCoordinateFunction()
{
  arrayCounter = 0;
    lcd.setCursor(0,0);
    lcd.print("* for .(point)");
    lcd.setCursor(0,1);
    lcd.print("# to enter");
    delay(2000);
  while((arrayCounter < numberOfTargets)  && (keyPress != 'D'))
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Enter Lat:");
    lcd.setCursor(11,0);
    lcd.print(arrayCounter + 1);
    delay(1500);
    getLat();
    LatCoordinate[arrayCounter] = latt;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Enter Lon:");
    lcd.setCursor(11,0);
    lcd.print(arrayCounter + 1);
    delay(1500);
    lcd.clear();
    getLon();
    LonCoordinate[arrayCounter] = lonn;
    arrayCounter = arrayCounter + 1;
  }
  lcd.clear();
  GetChar();
  while(!(keyPress == 'D' || keyPress == 'C') )
  {
    GetChar();
    lcd.setCursor(0,0);
    lcd.print("Cancel: C");
    lcd.setCursor(0,1);
    lcd.print("Continue: D");
  }
    if(keyPress == 'C')
    {
      proceed = false;
    }
    if(keyPress == 'D')
    {
      proceed = true;
    } 
  arrayCounter = 0;
}

void getLat()
{
  lcd.clear();
  point = false;
  negative = false;
  large = 0;
  small = 0;
  smallLarge = 0;
  smallTemp = 0;
  numba = 0;
  lastNumba = 0;
  temp1 = 0;
  base = 10;
  exponent = 0;
  latt = 0;
  lonn = 0;
  power = 0;
  keyPress = 0;

  while(keyPress != '#')
  {
   GetChar();                          
   if(keyPress == '*')//Use to check if . was entered
     {
      point = true;
     } 
   if(keyPress == 'D')//Use to check if . was entered
    {
     negative = true;
     lcd.print('-');
     keyPress = 0;
    } 
    if(keyPress >= '0' && keyPress <= '9') 
    {                                       
      if(!point)  //Add to the digits to the left of .
      {
        keyPress = keyPress - 48;      
        large = (large * 10) + keyPress;     
      }
      if(point)//Add to the digits to the right of .
      {
        keyPress = keyPress - 48;      
        exponent = exponent + 1;
        smallLarge = (smallLarge * 10) + keyPress;
        power = pow(base,exponent);
        small = (smallLarge/power);
      }
    }   
    numba = large + small;  
    if(negative)
    {
      numba = numba*(-1);
    }
    if((numba>0) || (numba<0))
    {
      lcd.clear();
      lcd.print(numba, exponent);
      delay(100);
    }
  }
  latt = numba;
}
void getLon()
{
  lcd.clear();
  point = false;
  negative = false;
  large = 0;
  small = 0;
  smallLarge = 0;
  smallTemp = 0;
  numba = 0;
  lastNumba = 0;
  temp1 = 0;
  base = 10;
  exponent = 0;
  latt = 0;
  lonn = 0;
  power = 0;
  keyPress = 0;
  
  while(keyPress != '#')
  {
    GetChar();                          
    if(keyPress == '*')//Use to check if . was entered
    {
      point = true;
    } 
    if(keyPress == 'D')//Use to check if . was entered
    {
      negative = true;
      lcd.print('-');
      keyPress = 0;
    } 
    if(keyPress >= '0' && keyPress <= '9') 
    {                                       
      if(!point)  //Add to the digits to the left of .
      {
        keyPress = keyPress - 48;      
        large = (large * 10) + keyPress;     
      }
      if(point)//Add to the digits to the right of .
      {
        keyPress = keyPress - 48;      
        exponent = exponent + 1;
        smallLarge = (smallLarge * 10) + keyPress;
        power = pow(base,exponent);
        small = (smallLarge/power);
      }
    }   
    
    numba = large + small;  
    if(negative)
    {
      numba = numba*(-1);
    }
    if((numba>0) || (numba<0))
    {
      lcd.clear();
      lcd.print(numba, exponent);
      delay(100);
    }
  }
  lonn = numba;
}
//********************************************************************Start UP***************************************************************************************
void flashLED()
{
  digitalWrite(gpsLED, HIGH);
  digitalWrite(compassLED, HIGH);
  digitalWrite(rfLED, HIGH);
  digitalWrite(autonomousLED, HIGH);
  digitalWrite(manualLED, HIGH);
  digitalWrite(userCommandLED, HIGH);
  delay(200);
  digitalWrite(gpsLED, LOW);
  digitalWrite(compassLED, LOW);
  digitalWrite(rfLED, LOW);
  digitalWrite(autonomousLED, LOW);
  digitalWrite(manualLED, LOW);
  digitalWrite(userCommandLED, LOW);
  delay(200);
  digitalWrite(gpsLED, HIGH);
  digitalWrite(compassLED, HIGH);
  digitalWrite(rfLED, HIGH);
  digitalWrite(autonomousLED, HIGH);
  digitalWrite(manualLED, HIGH);
  digitalWrite(userCommandLED, HIGH);
  delay(200);
  digitalWrite(gpsLED, LOW);
  digitalWrite(compassLED, LOW);
  digitalWrite(rfLED, LOW);
  digitalWrite(autonomousLED, LOW);
  digitalWrite(manualLED, LOW);
  digitalWrite(userCommandLED, LOW);
  delay(200);
  digitalWrite(gpsLED, HIGH);
  digitalWrite(compassLED, HIGH);
  digitalWrite(rfLED, HIGH);
  digitalWrite(autonomousLED, HIGH);
  digitalWrite(manualLED, HIGH);
  digitalWrite(userCommandLED, HIGH);
  delay(200);
  digitalWrite(gpsLED, LOW);
  digitalWrite(compassLED, LOW);
  digitalWrite(rfLED, LOW);
  digitalWrite(autonomousLED, LOW);
  digitalWrite(manualLED, LOW);
  digitalWrite(userCommandLED, LOW);
  delay(200);
}
void setUpLoop()
{
  Serial.begin(9600);
  customKeypad.setDebounceTime(50);
  customKeypad.setHoldTime(100);

  lcd.begin(16, 2);
  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, addresses[0]);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(gpsLED, OUTPUT);
  pinMode(compassLED, OUTPUT);
  pinMode(rfLED, OUTPUT);
  pinMode(autonomousLED, OUTPUT);
  pinMode(manualLED, OUTPUT);
  pinMode(userCommandLED, OUTPUT);
}
//*******************************************************************TimeCounter***************************************************************************************
int TimeCounter(int x)
{
  if (millis() - times > 1000)
  {
    x++;
    times = millis();
  }
  return x;
}
//**********************************************************************Key Pad*****************************************************************************************
void GetChar()
{
  char key = 0;
  key = customKeypad.getKey();
  if (key != NO_KEY)
  {
    keyPress = key;
  }
}
