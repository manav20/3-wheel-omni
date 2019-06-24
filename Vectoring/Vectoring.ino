/*  AETR
     RC override
    On the serial monitor write : 111,111*111 where  <magnitude,angle*rotation>
*/

//encoder initializations
#define encoder1PinA  2    //motor 1
#define encoder1PinB  3
long unsigned int encoder1Pos = 1000;
#define encoder2PinA  18    //motor 2
#define encoder2PinB  19
long unsigned int encoder2Pos = 1000;
#define encoder3PinA  20    //motor 3
#define encoder3PinB  21
long unsigned int encoder3Pos = 1000;

volatile int cps1 = 0;
volatile int cps2 = 0;
volatile int cps3 = 0;

//motor intializations
const int inAPin1 = A5;
const int inBPin1 = A4;
const int PWMPin1 = 8;

const int inAPin2 = A2;
const int inBPin2 = A3;
const int PWMPin2 = 9;

const int inAPin3 = A0;
const int inBPin3 = A1;
const int PWMPin3 = 10;

int i = 0;                            //used by timing ISR

int pwm1 = 0;
int cps1_des = 0;
double output1 = 0;

int pwm2 = 0;
int cps2_des = 0;
double output2 = 0;

int pwm3 = 0;
int cps3_des = 0;
double output3 = 0;

int temp1 = 1000;
int temp3 = 1000;
int temp2 = 1000;

//**************************************
//*********SETUP FUNCTION***************
//**************************************
void setup() {
  // motor setup
  pinMode(inAPin1, OUTPUT);
  pinMode(inBPin1, OUTPUT);
  pinMode(PWMPin1, OUTPUT);

  pinMode(inAPin2, OUTPUT);
  pinMode(inBPin2, OUTPUT);
  pinMode(PWMPin2, OUTPUT);

  pinMode(inAPin3, OUTPUT);
  pinMode(inBPin3, OUTPUT);
  pinMode(PWMPin3, OUTPUT);


  //encoder setup
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), doEncoder1, CHANGE);  // encoder pin on interrupt 2

  pinMode(encoder2PinA, INPUT);
  digitalWrite(encoder2PinA, HIGH);
  pinMode(encoder2PinB, INPUT);
  digitalWrite(encoder2PinB, HIGH);
  attachInterrupt(digitalPinToInterrupt(18), doEncoder2, CHANGE);  // encoder pin on interrupt 18

  pinMode(encoder3PinA, INPUT);
  digitalWrite(encoder3PinA, HIGH);
  pinMode(encoder3PinB, INPUT);
  digitalWrite(encoder3PinB, HIGH);
  attachInterrupt(digitalPinToInterrupt(20), doEncoder3, CHANGE);  // encoder pin on interrupt 18


  //Serial setup
  Serial.begin (115200);


  // TIMER 1 for interrupt frequency 5 Hz:
  cli(); // stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 5 Hz increments
  OCR1A = 49999; // = 16000000 / (64 * 5) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // allow interrupts

  //RC override
  pinMode(4, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(13, OUTPUT);
  pinMode(30, INPUT);

}
//**************END OF SETUP FUNCTION****************************

int err_new1 = 0;
int err_old1 = 0;
int err_d11 = 0;
int err_d21 = 0;

int err_new2 = 0;
int err_old2 = 0;
int err_d12 = 0;
int err_d22 = 0;

int err_new3 = 0;
int err_old3 = 0;
int err_d13 = 0;
int err_d23 = 0;

int v1;
int v2;
int v3;
float vel_x;
float vel_y;
float theta;
int magnitude;

// for PID
float adj = 1;
float Kp = 0.3;

int d1;                           // for direction of rotation of motors
int d2;
int d3;

//for serial comm
char buf[12];
String mov;


//RC override
int flag = 0;
int a;
int e;
int t;
float Speed = 0;
float Theta = 0;
float Rot = 0;

void loop()
{
  char rxBuf[12];
  if (digitalRead(30) == 0)
  {
    if (Serial.available() >= 4)         //Each data packet is 7 characters, 3 - throttle, 3 - steering
    {
      char readChar = Serial.read();
      if (readChar == 'A')
      {
        while (Serial.available() < 12);    //Wait for 12 characters to be read
        for (int i = 0; i < 12; i++)        // Read 6 characters iteratively
          rxBuf[i] = Serial.read();
        //for (int j = 0; j < 12; j++)        // Loop to display the array read
        //Serial.print(rxBuf[j]);
        //Serial.println("*****");

        Speed = 100 * ((float)rxBuf[1] - 48) + 10 * ((float)rxBuf[2] - 48) + 1 * ((float)rxBuf[3] - 48);
        Theta = 100 * ((float)rxBuf[5] - 48) + 10 * ((float)rxBuf[6] - 48) + 1 * ((float)rxBuf[7] - 48);
        Rot = 100 * ((float)rxBuf[9] - 48) + 10 * ((float)rxBuf[10] - 48) + 1 * ((float)rxBuf[11] - 48);
        //Serial.println(Speed);
        //Serial.println(Theta);
        //Serial.println(Rot);
        if (rxBuf[8] == 45)
        { flag = 1;
        }
        else flag = 0;
      }
    }
    if (flag == 1)
      base11(Speed, Theta, Rot);
    else if (flag == 0)
      base1(Speed, Theta, Rot);
  }
  else if (digitalRead(30) == 1)
  {
    digitalWrite(13, HIGH);
    a = pulseIn(7, HIGH);    //channel 1 (A)
    e = pulseIn(6, HIGH);    //channel 2 (E)
    t = pulseIn(4, HIGH);    //channel 3 (T)
    base2(a, e, t);
  }
}


//=============== Serial read=============
int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;
  if (readch > 0)
  {
    switch (readch)
    {
      case '\r': //ignore carriage return
        break;
      case '\n': // return on new line
        rpos = pos;
        pos = 0;
        return rpos;
      default:
        if (pos < len - 1)
        {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  return 0;
}

//===================Serial parse and Base command  ========================
void parseCommand(String com)
{
  String part1;
  String part2;
  String part3;
  part1 = com.substring(0, com.indexOf(','));                           // vector magnitude
  part2 = com.substring(com.indexOf(',') + 1, com.indexOf(',') + 4 );   // vector angle
  part3 = com.substring(com.indexOf('*') + 1, com.indexOf('*') + 4);    // rotation speed
  int mag = part1.toInt();
  mag = abs(mag);
  int ang = part2.toInt();
  ang = abs(ang);
  int rot = part3.toInt();
  base1(mag, ang, rot);
}


//============================== Vectoring via serial =======================
void base1(float magnitude, float theta, int rot )
{
  theta = theta * 0.0174533;              //convert from deg angle to radian
  vel_x = magnitude * cos(theta);
  vel_y = magnitude * sin(theta);

  const float sqrt3o2 = 1.0 * sqrt(3) / 2;
  float v1 = - vel_x + rot;
  float v2 = 0.5 * vel_x - sqrt3o2 * vel_y + rot;
  float v3 = 0.5 * vel_x + sqrt3o2 * vel_y + rot;

  d1 = 0;
  d2 = 0;
  d3 = 0;
  d1 = v1 < 0 ? -1 : 1;
  d2 = v2 < 0 ? -1 : 1;
  d3 = v3 < 0 ? -1 : 1;

  v1 = map(abs(v1), 0, 100, 0, 550);
  v2 = map(abs(v2), 0, 100, 0, 550);
  v3 = map(abs(v3), 0, 100, 0, 550);

  M1(v1, d1);
  M2(v2, d2);
  M3(v3, d3);
}

void base11(float magnitude, float theta, int rot )
{
  theta = theta * 0.0174533;              //convert from deg angle to radian
  vel_x = magnitude * cos(theta);
  vel_y = magnitude * sin(theta);

  const float sqrt3o2 = 1.0 * sqrt(3) / 2;
  float v1 = - vel_x - rot;
  float v2 = 0.5 * vel_x - sqrt3o2 * vel_y - rot;
  float v3 = 0.5 * vel_x + sqrt3o2 * vel_y - rot;

  d1 = 0;
  d2 = 0;
  d3 = 0;
  d1 = v1 < 0 ? -1 : 1;
  d2 = v2 < 0 ? -1 : 1;
  d3 = v3 < 0 ? -1 : 1;

  v1 = map(abs(v1), 0, 100, 0, 550);
  v2 = map(abs(v2), 0, 100, 0, 550);
  v3 = map(abs(v3), 0, 100, 0, 550);

  M1(v1, d1);
  M2(v2, d2);
  M3(v3, d3);
}

//====================== Vectoring via RC ===================
void base2(int vel_x, int vel_y, int rot)
{
  //  theta = theta * 0.0174533;              //convert from deg angle to radian
  //  vel_x = magnitude * cos(theta);
  //  vel_y = magnitude * sin(theta);



  vel_x = map(vel_x, 990 , 2000  , -100, 100);
  vel_y = map(vel_y, 980 , 2000 , -100, 100);
  rot = map(rot, 990 ,  2000 , -50, +50);

  if (rot > -10 && rot < 10)
  {
    rot = 0;
  }

  const float sqrt3o2 = 1.0 * sqrt(3) / 2;
  float v1 = - vel_x + rot;
  float v2 = 0.5 * vel_x - sqrt3o2 * vel_y + rot;
  float v3 = 0.5 * vel_x + sqrt3o2 * vel_y + rot;

  d1 = 0;
  d2 = 0;
  d3 = 0;
  d1 = v1 < 0 ? -1 : 1;
  d2 = v2 < 0 ? -1 : 1;
  d3 = v3 < 0 ? -1 : 1;

  v1 = map(abs(v1), 0, 100, 0, 550);
  v2 = map(abs(v2), 0, 100, 0, 550);
  v3 = map(abs(v3), 0, 100, 0, 550);

  M1(v1, d1);
  M2(v2, d2);
  M3(v3, d3);
}

//============== Motor Functions ================================
void M1( int cps1_des, int dir1)
{
  if (dir1 == 1)
  {
    digitalWrite(inAPin1, HIGH);
    digitalWrite(inBPin1, LOW);
  }
  else if (dir1 == -1)
  {
    digitalWrite(inAPin1, LOW);
    digitalWrite(inBPin1, HIGH);
  }
  else if (dir1 == 0)
  {
    digitalWrite(inAPin1, LOW);
    digitalWrite(inBPin1, LOW);
  }

  err_new1 = (cps1_des - cps1);                 // calc error in counts
  err_d21 = err_new1 - err_d11;
  err_new1 = (Kp * err_new1) + (0 * err_old1) + (0 * err_d21) ;

  output1 = adj * (1.4919 * (err_new1));      //converting counts to PWM

  if (output1 > 250)
    output1 = 250;
  else if (output1 < 10)
    output1 = 0;

  analogWrite(PWMPin1, output1);
  err_old1 = err_new1;
  err_d11 = err_old1;
}

void M2( int cps2_des, int dir2)
{
  if (dir2 == 1)
  {
    digitalWrite(inAPin2, LOW);
    digitalWrite(inBPin2, HIGH);
  }
  else if (dir2 == -1)
  {
    digitalWrite(inAPin2, HIGH);
    digitalWrite(inBPin2, LOW);
  }
  else if (dir2 == 0)
  {
    digitalWrite(inAPin2, LOW);
    digitalWrite(inBPin2, LOW);
  }

  err_new2 = (cps2_des - cps2);               // calc error in counts
  err_d22 = err_new2 - err_d12;
  err_new2 = (Kp * err_new2) + (0 * err_old2) + (0 * err_d22);

  output2 = adj * (1.4919 * (err_new2));    //converting counts to PWM

  if (output2 > 250)
    output2 = 250;
  else if (output2 < 10)
    output2 = 0;

  analogWrite(PWMPin2, output2);
  err_old2 = err_new2;
  err_d12 = err_old2;
}

void M3( int cps3_des, int dir3)
{
  if (dir3 == 1)
  {
    digitalWrite(inAPin3, LOW);
    digitalWrite(inBPin3, HIGH);
  }
  else if (dir3 == -1)
  {
    digitalWrite(inAPin3, HIGH);
    digitalWrite(inBPin3, LOW);
  }
  else if (dir3 == 0)
  {
    digitalWrite(inAPin3, LOW);
    digitalWrite(inBPin3, LOW);
  }

  err_new3 = (cps3_des - cps3);            // calc error in counts
  err_d23 = err_new3 - err_d13;
  err_new3 = (Kp * err_new3) + (0 * err_old3) + (0 * err_d23) ;

  output3 = adj * (1.4919 * (err_new3)); //converting counts to PWM

  if (output3 > 250)
    output3 = 250;
  else if (output3 < 10)
    output3 = 0;

  analogWrite(PWMPin3, output3);
  err_old3 = err_new3;
  err_d13 = err_old3;
}


//=============== Timer ISR ======================================
ISR(TIMER1_COMPA_vect)                    // Speed calcullation
{
  if ((encoder3Pos != temp3) && (i == 0))
  {
    i++;
    temp1 = encoder1Pos;
    temp2 = encoder2Pos;
    temp3 = encoder3Pos;
  }
  if (i != 0)
  {
    cps1 = encoder1Pos - temp1;
    temp1 = encoder1Pos;
    cps2 = encoder2Pos - temp2;
    temp2 = encoder2Pos;
    cps3 = encoder3Pos - temp3;
    temp3 = encoder3Pos;
  }
}

//=============================== Encoder ISR ====================
void doEncoder1()
{
  encoder1Pos++;
}
void doEncoder2()
{
  encoder2Pos++;
}
void doEncoder3()
{
  encoder3Pos++;
}
