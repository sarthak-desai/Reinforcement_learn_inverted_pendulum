/***********************************************************
File name:  AdeeptSelfBalancingRobotCode.ino
Description:  
Three working modes after the car is power on: 
Mode No.1: Remote control via Bluetooth
You can control the car to go forward and backward and turn left and right by commands via 
Bluetooth. At the same time, you can switch between the modes and control the buzzer to beep.
Mode No.2: Obstacle avoidance by ultrasonic
Under this mode, the car can detect and bypass the obstacles in front automatically. 
Mode No.3: Following 
Under this mode, the car will follow the object ahead straight. When it's 30-50cm to the 
object, the light onside will light up and the car will follow the object to move forward; 
when it's 5-20cm, the light turns into green, and the car will move backward. 
Website: www.adeept.com
E-mail: support@adeept.com
Author: Tom
Date: 2017/10/12 
***********************************************************/
#include "PinChangeInt.h"
#include "MsTimer2.h"
#include "Adeept_Balance2WD.h"
#include "Adeept_KalmanFilter.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu; //Instantiate an MPU6050 object with the object name mpu
Adeept_Balance2WD balancecar;//Instantiate an balance object with the object name balancecar
Adeept_KalmanFilter kalmanfilter;//Instantiate an KalmanFilter object with the object name kalmanfilter
//Adeept_Distance Dist;//Instantiate an distance object with the object name Dist

int motorRun = 0;//0:Stop;  1:Go ahead;  2;Backwards;  3:Turn left;  4:Turn right;

int16_t ax, ay, az, gx, gy, gz;
//TB6612FNG Drive module control signal
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8
//Speed PID control is realized by using speed code counting
#define PinA_left 2  //Interrupt 0
#define PinA_right 4 //Interrupt 1

#define PPR_const 0.2656923077//pulse per meter
#define vel_const 0.53138461//m/s per pulse

/****************************Declare a custom variable*****************/
int time;
byte inByte; //The serial port receives the byte
int num;
double Setpoint;                                         //Angle DIP setpoint, input, and output
double Setpoints, Outputs = 0;                           //Speed DIP setpoint, input, and output
double kp = 30, ki = 0.1, kd = 0.58;//kp = 38, ki = 0, kd = 0.58;//Need you to modify the parameters kp = 30, ki = 0.1, kd = 0.58;
double kp_speed = 3.6, ki_speed = 0.1058, kd_speed = 0.0; // Need you to modify the parameters kp_speed = 3.6, ki_speed = 0.1058, kd_speed = 0.0;
double kp_turn = 28, ki_turn = 0, kd_turn = 0.29;        //Rotate PID setting
//Steering PID parameters
double setp0 = 0, dpwm = 0, dl = 0; //Angle balance point, PWM difference, dead zone, PWM1, PWM2
float value;

/***************************new parameters****************************************************************************************************************************/
float theta = 0.0;
float theta_dot = 0.0;
float theta_old = 0.0;

float x_ = 0.0;
float x_dot = 0.0;

//long long int total_pulse_L_right = 0;
//long long int total_pulse_R_right = 0;
//long long int total_pulse_L_left = 0;
//long long int total_pulse_R_left = 0;

//float cent_dist = 0;
//float net_pulse_right = 0.0;
//float net_pulse_left = 0.0;
float net_pulse = 0.0;

char move_dir = '0'; // 0 = left, 1 = right

uint8_t mscount_20 = 0;
bool send_flag = 0;
bool start_flag = 0;
char start_temp = 'a';

float multi = 1;
/********************angle data*********************/
float Q;
float Angle_ax; //The angle of inclination calculated from the acceleration
float Angle_ay;
float K1 = 0.05; // The weight of the accelerometer
float angle0 = 0.00; //Mechanical balance angle
int slong;

/***************Kalman_Filter*********************/
float Q_angle = 0.001, Q_gyro = 0.005; //Angle data confidence, angular velocity data confidence
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //Filter method sampling time interval milliseconds
float dt = timeChange * 0.001; //Note: The value of dt is the filter sampling time

/******************* speed count ************/
volatile long count_right = 0;//Use the volatile long type to ensure that the value is valid for external interrupt pulse count values used in other functions
volatile long count_left = 0;//Use the volatile long type to ensure that the value is valid for external interrupt pulse count values used in other functions
int speedcc = 0;

/*******************************Pulse calculation*****************************/
float lz = 0;
float rz = 0;
float rpluse = 0;
float lpluse = 0;

/********************Turn the parameters of rotation**********************/
int turncount = 0;
float turnoutput = 0;

/****************Bluetooth control volume*******************/
int front = 0;//Forward variable
int back = 0;//Backward variables
int turnl = 0;//Turn left mark
int turnr = 0;//Turn right
int spinl = 0;//Left rotation mark
int spinr = 0;//Right turn mark

/***************Ultrasonic velocity******************/
int distance;
int detTime=0; 

//const int buzzerPin = 11;  // define pin for buzzer

/*Pulse calculation*/
void countpluse(){
  lz = count_left;
  rz = count_right;
  
  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if(move_dir == '1'){
    //total_pulse_R_right += rpluse;
    //total_pulse_L_right += lpluse;
    multi = 1;
    }
  else{
    //total_pulse_R_left += rpluse;
    //total_pulse_L_left += lpluse;
    multi = -1;
    }

  //net_pulse_right = (total_pulse_R_right + total_pulse_L_right)/2;
  //net_pulse_left = (total_pulse_R_left + total_pulse_L_left)/2;
  net_pulse = (lpluse+rpluse)/2;
  x_ += multi*net_pulse;//multi*PPR_const*net_pulse;
  x_dot = multi*net_pulse;//multi*vel_const*net_pulse;
  
  /*
  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0)){//Car movement direction to determine: back when (PWM is the motor voltage is negative) pulse number is negative
    rpluse = -rpluse;
    lpluse = -lpluse;
  }else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0)){//Car movement direction to determine: forward (PWM is the motor voltage is positive) pulse number is negative
    rpluse = rpluse;
    lpluse = lpluse;
  }else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0)){////Car movement direction to determine: right rotation, the right pulse number is positive, the number of left pulse is negative.
    rpluse = rpluse;
    lpluse = -lpluse;
  }else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0)){//Car movement direction to determine: left rotation, the right pulse number is negative, the number of left pulse is positive.
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  //To judge
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;
  //Every 5ms into the interruption, the number of pulses superimposed
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  */
}
/*Angle PD*/
void angleout(){
  theta = kalmanfilter.angle;
  theta_dot = (theta - theta_old)/0.005;
  theta_old = theta;
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD angle loop control
}
/*Interrupt timing 5ms timer interrupt*/
void inter(){
  sei();                                           
  countpluse();                                     //Pulse superposition of sub - functions
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC gets MPU6050 six axis data ax ay az gx gy gz
  kalmanfilter.angleTest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro,R_angle,C_0,K1);  //Get angle and Kaman filter
  angleout();                                       //Angle loop PD control
  mscount_20++;
  if(mscount_20 == 4){
    mscount_20 = 0;
  }
}

void setup() {
  // TB6612FNGN drive module control signal initialization
  pinMode(IN1M, OUTPUT);//Control the direction of the motor 1, 01 for the forward rotation, 10 for the reverse
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);//Control the direction of the motor 2, 01 for the forward rotation, 10 for the reverse
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);//Left motor PWM
  pinMode(PWMB, OUTPUT);//Right motor PWM
  pinMode(STBY, OUTPUT);//TB6612FNG enabled

  //Initialize the motor drive module
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);  //Speed code A input
  pinMode(PinA_right, INPUT); //Speed code B input


  //Initialize the I2C bus
  Wire.begin();  
  //Turn on the serial port and set the baud rate to 9600
  //Communicate with the Bluetooth module
  Serial.begin(57600); 
  delay(150);
  //Initialize the MPU6050
  mpu.initialize();    
  delay(2);
 //5ms timer interrupt setting. Use timer2. Note: Using timer2 will affect the PWM output of pin3 and pin11.
 //Because the PWM is used to control the duty cycle timer, so when using the timer should pay attention to 
 //see the corresponding timer pin port.
  MsTimer2::set(5, inter);
  MsTimer2::start();
  attachInterrupt(0, Code_left, CHANGE);//pin 2 is interrupt 0
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
}

void loop() {
  //The main function of the cycle of detection and superposition of pulse, the determination of car speed.
  //Use the level change both into the pulse superposition, increase the number of motor pulses to ensure 
  //the accuracy of the car.
  /*
  if(!start_flag){
    if(Serial.available() > 0){ 
         start_temp = Serial.read();
         if(start_temp == 's'){
          //start_flag = 1;
          Serial.println('S');
          Serial.println(x_);
          Serial.println(x_dot);
          Serial.println(theta);
          Serial.println(theta_dot);
          start_temp = Serial.read();
          if(start_temp == 'a'){
          start_flag = 1;
          }
         }
      }
  }
  */
  //else{

    if(Serial.available() > 0){
      
    start_temp = Serial.read();
    
    if(start_temp == 'e'){
    Serial.println('S');
    Serial.println(x_);
    Serial.println(x_dot);
    Serial.println(theta);
    Serial.println(theta_dot);
    //if(Serial.available() > 0){ 
       move_dir = Serial.read();
      //}
    }

    else if(start_temp == 'i'){
    Serial.println('S');
    Serial.println(x_);
    Serial.println(x_dot);
    Serial.println(theta);
    Serial.println(theta_dot);

    
    }
    
    }

  
    if(move_dir == '0')
    {
      motor_left();
    }
    else if(move_dir == '1')
    {
      motor_right();
    }
    else if(move_dir == 'x')
    {
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
    }
    
  //}
}
/*Left speed chart*/
void Code_left() {
  count_left ++;
  //total_pulse_L++;
} 
/*Right speed chart count*/
void Code_right() {
  count_right ++;
  //total_pulse_R++;
} 

void motor_left(){

  digitalWrite(IN1M, HIGH);
  digitalWrite(IN2M, LOW);
  digitalWrite(IN3M, HIGH);
  digitalWrite(IN4M, LOW);
  analogWrite(PWMA, 130);
  analogWrite(PWMB, 130);
  //digitalWrite(PWMA, HIGH);
  //digitalWrite(PWMB, HIGH);
  digitalWrite(STBY, HIGH);

}

void motor_right(){

  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, HIGH);
  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, HIGH);
  analogWrite(PWMA, 130);
  analogWrite(PWMB, 130);
  //digitalWrite(PWMA, HIGH);
  //digitalWrite(PWMB, HIGH);
  digitalWrite(STBY, HIGH);

}
