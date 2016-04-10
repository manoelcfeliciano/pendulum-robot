//------------
//-----------------------
//---------------------------------
// Initializations and Definitions
//---------------------------------
//-----------------------
//------------

// Motor pins
const int AIN1 = 3;
const int AIN2 = 9;
const int BIN1 = 10;
const int BIN2 = 11;

//Speed variable
int speed;


//------------
//-----------------------
//------------------------------
// General Functions
//------------------------------
//-----------------------
//------------

//Intialize Motors - Set pins as OUTPUT
void InitMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);  
}

//Drive forward and backward depeding on the speed variable from Pid function
void Motors(){
  if (speed > 0)
  { 
    //forward 
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, speed);
    analogWrite(BIN2, 0);
  }
  else
  { 
    // backward
    speed = map(speed,0,-255,0,255);
    analogWrite(AIN1, 0);
    analogWrite(AIN2, speed);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, speed);
  }
}

//Stop the motors
void stop()
{
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}
