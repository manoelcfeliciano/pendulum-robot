// Motor controller pins
const int AIN1 = 3;
const int AIN2 = 9;
const int BIN1 = 10;
const int BIN2 = 11;

//Speed variable
int speed;

void InitMotors() {
  pinMode(AIN1, OUTPUT); // set pins to output
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);  
}

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

void stop()
{
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}
