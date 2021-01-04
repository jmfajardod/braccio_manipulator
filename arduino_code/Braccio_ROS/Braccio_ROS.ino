// Control of Braccio manipulator
// Developer: Jose Fajardo

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle  nh;

#define SOFT_START_CONTROL_PIN  12

// Declare Servos
Servo base1;
Servo shoulder2;
Servo elbow3;
Servo wrist_rot4;
Servo wrist_ver5;
Servo gripper6;

// Variables for control and sensing of the motors
float angle1, angle2, angle3, angle4, angle5, angle6; 
float val_m_1, val_m_2, val_m_3, val_m_4, val_m_5, val_m_6; 
float goal_m_1, goal_m_2, goal_m_3, goal_m_4, goal_m_5, goal_m_6;
float vel_m_1, vel_m_2, vel_m_3, vel_m_4, vel_m_5, vel_m_6;   
float step_aux;
float aux_pos;
bool reached_Goal;

// Frequency of update of the motor values
float Frequency = 30; // 30 Hz
float Period = 1/Frequency;

// Variable to enable or disable the motors
bool Motors_ON = false;


// Declare subscribers


void servo_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  
  goal_m_1   = cmd_msg.data[0];
  goal_m_2   = cmd_msg.data[1];
  goal_m_3   = cmd_msg.data[2];
  goal_m_4   = cmd_msg.data[3];
  goal_m_5   = cmd_msg.data[4];
  goal_m_6   = cmd_msg.data[5];
  checkAnglesGoal();
  
  if( TIMSK1 ==0){
    if((goal_m_1 != val_m_1) || (goal_m_2 != val_m_2) || (goal_m_3 != val_m_3) || (goal_m_4 != val_m_4) || (goal_m_5 != val_m_5) || (goal_m_6 != val_m_6)){
      TIMSK1 |= (1 << OCIE1C); // Set interrupt for channel C
    }
  }
  
}
ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo_cmd", servo_cb);

void setup(){
  
  // Select pin 12 as output    
  pinMode(12,OUTPUT);
  change_State_Motors();
  
  // Initialization pin Servo motors
  base1.attach(11);
  shoulder2.attach(10);
  elbow3.attach(9);
  wrist_rot4.attach(6);
  wrist_ver5.attach(5,500,1888); // Define different PWM limits 500 low and 1888 high
  gripper6.attach(3,500,1888);

  // Initialize Sensor values
  angle1 = 0;
  angle2 = 0;
  angle3 = 0;
  angle4 = 0;
  angle5 = 0;
  angle6 = 0;

  // Init Motors
  init_Motors();

  // Check that motor values and goals are valid
  checkAnglesValues();
  checkAnglesGoal();

  // INIT interrupts after servo attachment
  
  TCCR1A |= (1 << WGM10); // Set 10 bit counter
  TCCR1A |= (1 << WGM11);
  
  TCCR1B = 0x00;
  TCCR1B |= (1 << CS12); // 1024 divider
  TCCR1B |= (1 << CS12);  

  // Clean Interrupts
  TIMSK1 = 0x00;

  // Set 30 Hz as desired rate
  // (16MHz/1024)/30Hz = 521
  OCR1C = int(((16000000.0 / 1024.0)/Frequency)+1);

  go_Home();

  // Set velocities of joints
  vel_m_1    = 30; // deg/s
  vel_m_2    = 30;
  vel_m_3    = 30;
  vel_m_4    = 30;
  vel_m_5    = 30;
  vel_m_6    = 10;

  // Init node and subscribers
  nh.initNode();
  nh.subscribe(sub);
  
}
/***************************************************************************/
/***************************************************************************/
void change_State_Motors(){
  // Enable or disable motors
  if( Motors_ON){
    digitalWrite(12,HIGH);
  }
  else{
    digitalWrite(12,LOW);
  }
}
/***************************************************************************/
/***************************************************************************/
void init_Motors(){

  // Send dummy values to stabilize voltage
  base1.write(90);
  shoulder2.write(90);
  elbow3.write(90);
  wrist_rot4.write(90);
  wrist_ver5.write(90);
  gripper6.write(90);
  
  // Initialize Servo position with current position
  sensingPosition(); // Sense for the first time attaching the pin to analogRead
  delay(500);       // Wait for the Analog Value to stabilize
  sensingPosition(); // Read again
  delay(500);       // Wait for the Analog Value to stabilize
  sensingPosition(); // Read again
  
  // Init the motor values with the sensed angle values
  val_m_1 = angle1;
  val_m_2 = angle2;
  val_m_3 = angle3;
  val_m_4 = angle4;
  val_m_5 = angle5;
  val_m_6 = angle6;
  
  // Check that values are valid
  checkAnglesValues();
  sendCmd();
  
  // Enable motors
  Motors_ON = true;
  change_State_Motors();
  
  // Send to motors init values
  sendCmd();
  delay(500);
 
}
/***************************************************************************/
/***************************************************************************/
// Channel A Timer 1 Interrupt
ISR(TIMER1_COMPC_vect){          // timer compare interrupt service routine

  reached_Goal = true;
  
  // For motor1
  if(val_m_1 != goal_m_1){
    reached_Goal = false;
    step_aux = Period * vel_m_1; // Delta_x = Vel*delta_t;
    // Positive delta
    if(val_m_1 < goal_m_1){
      aux_pos = val_m_1 + step_aux;
      if  (aux_pos>goal_m_1) val_m_1 = goal_m_1;
      else val_m_1 = aux_pos;
    }
    // Negative delta
    else{
      aux_pos = val_m_1 - step_aux;
      if  (aux_pos<goal_m_1) val_m_1 = goal_m_1;
      else val_m_1 = aux_pos;
    }
  }
  /*************************************************************/
  // For motor 2
  if(val_m_2 != goal_m_2){
    reached_Goal = false;
    step_aux = Period * vel_m_2; // Delta_x = Vel*delta_t;
    // Positive delta
    if(val_m_2 < goal_m_2){
      aux_pos = val_m_2 + step_aux;
      if  (aux_pos>goal_m_2) val_m_2 = goal_m_2;
      else val_m_2 = aux_pos;
    }
    // Negative delta
    else{
      aux_pos = val_m_2 - step_aux;
      if  (aux_pos<goal_m_2) val_m_2 = goal_m_2;
      else val_m_2 = aux_pos;
    }
  }
  /*************************************************************/
  // For motor 3
  if(val_m_3 != goal_m_3){
    reached_Goal = false;
    step_aux = Period * vel_m_3; // Delta_x = Vel*delta_t;
    // Positive delta
    if(val_m_3 < goal_m_3){
      aux_pos = val_m_3 + step_aux;
      if  (aux_pos>goal_m_3) val_m_3 = goal_m_3;
      else val_m_3 = aux_pos;
    }
    // Negative delta
    else{
      aux_pos = val_m_3 - step_aux;
      if  (aux_pos<goal_m_3) val_m_3 = goal_m_3;
      else val_m_3 = aux_pos;
    }
  }
  /*************************************************************/
  // For motor 4
  if(val_m_4 != goal_m_4){
    reached_Goal = false;
    step_aux = Period * vel_m_4; // Delta_x = Vel*delta_t;
    // Positive delta
    if(val_m_4 < goal_m_4){
      aux_pos = val_m_4 + step_aux;
      if  (aux_pos>goal_m_4) val_m_4 = goal_m_4;
      else val_m_4 = aux_pos;
    }
    // Negative delta
    else{
      aux_pos = val_m_4 - step_aux;
      if  (aux_pos<goal_m_4) val_m_4 = goal_m_4;
      else val_m_4 = aux_pos;
    }
  }
  /*************************************************************/
  // For motor 5
  if(val_m_5 != goal_m_5){
    reached_Goal = false;
    step_aux = Period * vel_m_5; // Delta_x = Vel*delta_t;
    // Positive delta
    if(val_m_5 < goal_m_5){
      aux_pos = val_m_5 + step_aux;
      if  (aux_pos>goal_m_5) val_m_5 = goal_m_5;
      else val_m_5 = aux_pos;
    }
    // Negative delta
    else{
      aux_pos = val_m_5 - step_aux;
      if  (aux_pos<goal_m_5) val_m_5 = goal_m_5;
      else val_m_5 = aux_pos;
    }
  }
  /*************************************************************/
  // For motor 6
  if(val_m_6 != goal_m_6){
    reached_Goal = false;
    step_aux = Period * vel_m_6; // Delta_x = Vel*delta_t;
    // Positive delta
    if(val_m_6 < goal_m_6){
      aux_pos = val_m_6 + step_aux;
      if  (aux_pos>goal_m_6) val_m_6 = goal_m_6;
      else val_m_6 = aux_pos;
    }
    // Negative delta
    else{
      aux_pos = val_m_6 - step_aux;
      if  (aux_pos<goal_m_6) val_m_6 = goal_m_6;
      else val_m_6 = aux_pos;
    }
  }

  if(reached_Goal == true){
    TIMSK1 &= (0 << OCIE1C); // Disable interrupt for channel C
  }
  else{
    sendCmd();
    //sensingPosition();
  }
}
/***************************************************************************/
/***************************************************************************/
void go_Home(){
    
  // Initialize Servo velocity
  vel_m_1 = 5;
  vel_m_2 = 5;
  vel_m_3 = 5;
  vel_m_4 = 5;
  vel_m_5 = 5;
  vel_m_6 = 5;

  // Initialize Goal Position
  goal_m_1 = 90.0;
  goal_m_2 = 90.0;
  goal_m_3 = 90.0;
  goal_m_4 = 90.0;
  goal_m_5 = 90.0;
  goal_m_6 = 45.0;

  TIMSK1 |= (1 << OCIE1C); // Set interrupt for channel C
  
  while( TIMSK1 !=0){
    continue;
  }
  
  delay(5000);
}
/***************************************************************************/
/***************************************************************************/
void checkAnglesValues(){
  // Check that the values are within range
  if(val_m_1>180.0) val_m_1 = 180.0;
  if(val_m_1<0.0)   val_m_1 =  0.0;

  if(val_m_2>165.0) val_m_2 =  165.0;
  if(val_m_2<15.0)  val_m_2 =  15.0;

  if(val_m_3>180.0) val_m_3 = 180.0;
  if(val_m_3<0.0)   val_m_3 =  0.0;

  if(val_m_4>180.0) val_m_4 = 180.0;
  if(val_m_4<0.0)   val_m_4 =  0.0;
  
  if(val_m_5>125.0) val_m_5 = 125.0;
  if(val_m_5<0.0)   val_m_5 =  0.0;

  if(val_m_6>90.0)  val_m_6 = 90.0;
  if(val_m_6<0.0)   val_m_6 =  0.0;
}
/***************************************************************************/
/***************************************************************************/
void checkAnglesGoal(){
  // Check that the values are within range
  if(goal_m_1>180.0) goal_m_1 = 180.0;
  if(goal_m_1<0.0)   goal_m_1 =  0.0;

  if(goal_m_2>165.0) goal_m_2 =  165.0;
  if(goal_m_2<15.0)  goal_m_2 =  15.0;

  if(goal_m_3>180.0) goal_m_3 = 180.0;
  if(goal_m_3<0.0)   goal_m_3 =  0.0;

  if(goal_m_4>180.0) goal_m_4 = 180.0;
  if(goal_m_4<0.0)   goal_m_4 =  0.0;
  
  if(goal_m_5>125.0) goal_m_5 = 125.0;
  if(goal_m_5<0.0)   goal_m_5 =  0.0;

  if(goal_m_6>90.0)  goal_m_6 = 90.0;
  if(goal_m_6<0.0)   goal_m_6 =  0.0;
}
/***************************************************************************/
/***************************************************************************/
void sendCmd(){

  float cmd_1, cmd_2, cmd_3, cmd_4, cmd_5, cmd_6;
  
  // Map commands to motors
  cmd_1 = val_m_1;
  cmd_2 = val_m_2-7;
  cmd_3 = val_m_3-3;
  cmd_4 = val_m_4+3;
  cmd_5 = 50  + val_m_5*( (150.0-50.0) /125.0);
  cmd_6 = 100 + val_m_6*( (140.0-100.0) /90.0);
  
  // Write to Motors
  base1.write(int(cmd_1));
  shoulder2.write(int(cmd_2));
  elbow3.write(int(cmd_3));
  wrist_rot4.write(int(cmd_4));
  wrist_ver5.write(int(cmd_5));
  gripper6.write(int(cmd_6));
}
/***************************************************************************/
/***************************************************************************/
void sensingPosition(){

  int sensor_1=0, sensor_2=0, sensor_3=0, sensor_4=0, sensor_5=0,sensor_6=0;
  
  // Read Sensors
  sensor_1 = analogRead(A0);
  sensor_2 = analogRead(A1);
  sensor_3 = analogRead(A2);
  sensor_4 = analogRead(A3);
  sensor_5 = analogRead(A4);
  sensor_6 = analogRead(A5);

  // Equations when motors are ON
  if( Motors_ON){
    // Map sensors value to degrees
    angle1 = ( float(sensor_1) - 40.0) * (180.0/(659.0-40.0)); 
    angle2 = 15 + ( float(sensor_2) - 100.0) * (150.0/(630.0-100.0)); 
    angle3 = ( float(sensor_3) - 48.0) * (180.0/(655.0-48.0)); 
    angle4 = ( float(sensor_4) - 43.0) * (180.0/(652.0-43.0)); 
    angle5 = ( float(sensor_5) - 300.0) * (125.0/(104.0-300.0)); 
    angle6 = ( float(sensor_6) - 201) * (90.0/(121.0-201.0)); 

    // Estimate the angle using the sensor and the command
    angle1 = 0.35*angle1 + 0.65*float(val_m_1);
    angle2 = 0.25*angle2 + 0.75*float(val_m_2);
    angle3 = 0.35*angle3 + 0.65*float(val_m_3);
    angle4 = 0.35*angle4 + 0.65*float(val_m_4);
    angle5 = 0.35*angle5 + 0.65*float(val_m_5);
    angle6 = 0.35*angle6 + 0.65*float(val_m_6);
  }
  else{
    
    // Map sensors value to degrees
    angle1 = ( float(sensor_1) - 4.0) * (180.0/(75.0-4.0)); 
    angle2 = 15 + ( float(sensor_2) - 6.0) * (150.0/(65.0-6.0)); 
    angle3 = ( float(sensor_3) - 4.0) * (180.0/(70.0-4.0)); 
    angle4 = ( float(sensor_4) - 4.0) * (180.0/(70-4.0)); 
    angle5 = ( float(sensor_5) - 264.7) * (125.0/(109.2-264.7)); 
    angle6 = ( float(sensor_6) - 199.5) * (90.0/(124.0-199.5)); 
  }
  
}
/***************************************************************************/
/***************************************************************************/
void loop() {

  nh.spinOnce();
  delay(1);
  
}
/***************************************************************************/
/***************************************************************************/
