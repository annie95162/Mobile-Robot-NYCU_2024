#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <PID_v1.h>

#define EN_DEBUG (0)
#define TIME_DELAY (5)

/*-----Define pin(s)-----*/
// Left = motorA
const int motorA1 = 8;
const int motorA2 = 9;
const int motorAPWM = 5;
const int encoderA_A = 12;
const int encoderA_B = 3;
// Right = motorB
const int motorB1 = 10;
const int motorB2 = 11;
const int motorBPWM = 6;
const int encoderB_A = 4;
const int encoderB_B = 2;


/*-----Define variables-----*/
volatile long encoderAPos = 0;
volatile long encoderBPos = 0;
bool need_correction = false;
double encoderA_accumulation = 0.0;
double encoderB_accumulation = 0.0;
double distanceDifference = 0.0;
double last_distanceDifference = 0.0;
double speed_correction = 0.0;
double kc = 0.00003;



/*-----Define PID parameters-----*/
double Kpleft=28.0, Kileft=0.10, Kdleft=0.05;
double Kpright=32.9, Kiright=0.30, Kdright=0.70;
double Setleft=0.0, Setright=0.0, Inputleft=0.0, Inputright=0.0, Outputleft=0.0, Outputright=0.0;
PID PIDleft(&Inputleft, &Outputleft, &Setleft, Kpleft, Kileft, Kdleft, DIRECT);
PID PIDright(&Inputright, &Outputright, &Setright, Kpright, Kiright, Kdright, DIRECT);


/*-----Rosserial Define-----*/
ros::NodeHandle nh;

// control subscriber
void controlCallback(const std_msgs::Int16MultiArray& msg) {
  // Set left control
  Setleft = abs(msg.data[0]);
  // Set right control
  Setright = abs(msg.data[1]);

  // Set left motor direction
  if (msg.data[0] >= 0) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  } else {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }
  // Set right motor direction
  if (msg.data[1] >= 0) {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  } else {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }

  // If same speed need straight line correction
  if (msg.data[0] == msg.data[1] && msg.data[0] != 0) {
    need_correction = true;
  } else {
    need_correction = false;
  }

  // Reset cumulative distance
  encoderA_accumulation =0;
  encoderB_accumulation =0;
}
ros::Subscriber<std_msgs::Int16MultiArray> control_sub("control", controlCallback);

#if EN_DEBUG
// output publisher
std_msgs::Int16MultiArray output_msg;
ros::Publisher output_pub("output", &output_msg);
#endif


void setup() {
  // Set motor as output
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Set encoder pin as input
  pinMode(encoderA_A, INPUT);
  pinMode(encoderA_B, INPUT);
  pinMode(encoderB_A, INPUT);
  pinMode(encoderB_B, INPUT);

  // Set ISR
  attachInterrupt(1, encoderAInt, CHANGE);
  attachInterrupt(0, encoderBInt, CHANGE);

  // Set PID mode
  PIDleft.SetMode(AUTOMATIC);
  PIDright.SetMode(AUTOMATIC);
  
  // Set PID limit
  PIDleft.SetOutputLimits(0, 255);
  PIDright.SetOutputLimits(0, 255);

  // Set rosserial
  nh.initNode();
  nh.subscribe(control_sub);
#if EN_DEBUG
  output_msg.data_length = 6;
  output_msg.data = new int16_t[6];
  nh.advertise(output_pub);
#endif
}


void loop() {
  // Read from encoder
  Inputleft = encoderAPos;
  encoderAPos = 0;
  Inputright = encoderBPos;
  encoderBPos = 0;

  // Calculate moving distance
  encoderA_accumulation += Inputleft;
  encoderB_accumulation += Inputright;

  // Straight line correction
  if (need_correction) {
    distanceDifference = abs(encoderA_accumulation - encoderB_accumulation);
    speed_correction = distanceDifference * kc;
    if (encoderA_accumulation > encoderB_accumulation) {
      if (distanceDifference > last_distanceDifference) {
        if (speed_correction > Setleft) speed_correction = Setleft;
        Setleft -= speed_correction;
        Setright += speed_correction;
      }
    } else if (encoderA_accumulation < encoderB_accumulation) {
      if (distanceDifference > last_distanceDifference) {
        if (speed_correction > Setright) speed_correction = Setright;
        Setleft += speed_correction;
        Setright -= speed_correction;
      }
    }
    last_distanceDifference = distanceDifference;
  }

  // Compute PID
  PIDleft.Compute();
  PIDright.Compute();

  // Motor PWM control
  analogWrite(motorAPWM, round(Outputleft));
  analogWrite(motorBPWM, round(Outputright));
  
#if EN_DEBUG
  if((Setleft != 0) && (Setright != 0)) {
    output_msg.data[0] = Inputleft;
    output_msg.data[1] = Inputright;
    output_msg.data[2] = (Outputleft + 0.5);
    output_msg.data[3] = (Outputright + 0.5);
    output_msg.data[4] = encoderA_accumulation;
    output_msg.data[5] = encoderB_accumulation;
    output_pub.publish(&output_msg);
  }
#endif

  nh.spinOnce();
  delay(TIME_DELAY);
}

void encoderAInt()
{
  encoderAPos++;
}

void encoderBInt()
{
  encoderBPos++;
}
