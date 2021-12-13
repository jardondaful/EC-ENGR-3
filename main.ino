


// PD Controller

#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

// Pin Numbers
const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

// Base & Incremental Speeds
const int lSpd = 60;    //60 - 80
const int rSpd = 60;    //60 - 80
const int spdInc = 10;  //10 - 15
 
// PD Constants
const float k_p = 0.009; //0.007 -  -
const float k_d = 0.18
;  //0.18 -  

// Error Tracker
float old_error = 0;
int end_counter = 0;
int final_counter = 0;

const int LED_RF = 41;

void setup() 
{
  // Initialize Pins
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Set Wheel Directions
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  // Start LED
  pinMode(LED_RF, OUTPUT);
  
  ECE3_Init();

  // Start Serial Monitor
  Serial.begin(9600); 

  // Wait 2 seconds before starting
  delay(2000);
}

void loop() 
{
  // Set Initial State
  int leftSpd = lSpd;
  int rightSpd = rSpd;
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(left_dir_pin, LOW);

  // Sensor Fusion
  ECE3_read_IR(sensorValues);
  sensorValues[0] = (sensorValues[0]-640.2)/1.8598;
  sensorValues[1] = (sensorValues[1]-496.8)/2.0031;
  sensorValues[2] = (sensorValues[2]-687.8)/1.8122;
  sensorValues[3] = (sensorValues[3]-552)/1.2306;
  sensorValues[4] = (sensorValues[4]-528)/1.256;
  sensorValues[5] = (sensorValues[5]-551)/1.2692;
  sensorValues[6] = (sensorValues[6]-597)/1.424;
  sensorValues[7] = (sensorValues[7]-631.2)/1.6288;

  // Error Function
  float error = -8*sensorValues[0]-6*sensorValues[1]-4*sensorValues[2]-2*sensorValues[3]+2*sensorValues[4]+4*sensorValues[5]+6*sensorValues[6]+8*sensorValues[7];
  
  // Derivative Controller
  float d_spd = 0;
  
  if(old_error != 0)
  {
    float dif_error = error - old_error;
    error = error + dif_error * k_d;
  }
  old_error = error;

  // Proportional Controller
  float p_spd = abs(error)*k_p;
  if(error > 0)
  {
    rightSpd = rightSpd + 0.5*p_spd;
    leftSpd = leftSpd - 0.5*p_spd;
  } 
  else if(error < 0)
  {
    leftSpd = leftSpd + 0.5*p_spd;
    rightSpd = rightSpd - 0.5*p_spd;
    digitalWrite(right_dir_pin, HIGH);
  }

  // U-Turn Condition
  if(sensorValues[0] > 900 && sensorValues[1] > 900 && sensorValues[2] > 900 && sensorValues[3] > 900)
  {
    end_counter = end_counter + 1;
  }
  else
  {
    end_counter = 0; 
   }
   
  if(end_counter == 2)
  {
    final_counter = final_counter + 1;
    if(final_counter == 2){
      digitalWrite(right_nslp_pin,LOW);
      digitalWrite(left_nslp_pin,LOW);
   }
    digitalWrite(right_dir_pin,HIGH);
    analogWrite(left_pwm_pin,leftSpd);
    analogWrite(right_pwm_pin,rightSpd);
    delay(700);      // 900 - 700
  }
  
  // Update State
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);
  }
