#define trigPin 11
#define echoPin 12
#define GreenLED 10
#define AmberLED 9

#define LFS A1 //line follower left sensor
#define RFS A0 // line follower right sensor
#define servopin 7
#define HallEffect 13

#define LeftIR A3


//#include <turning_left.ino> //the file that holds turning function
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

Servo myservo;
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1 and M2
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);

void searching_for_mines(int line_follower_right,int line_follower_left,int ultrasound_distance);
void mine_found(int line_follower_right,int line_follower_left,int ultrasound_distance);
void driving_forward(int driving_speed, int driving_time);
void turning_about_its_centre(int turning_speed, int driving_time, int turn_direction); //declaring function
void turning_to_the_next_line(int turning_speed, int driving_time, int turn_direction);
void driving_backward(int driving_speed, int driving_time);
void line_follower(int right_or_left);
void blinking(int ledPin);

int line_follower_threshold = 700; //line follower readings are >900 when white line and  <300 when black surface (DEPENDS ON WEATHER)
int turning_time_right_using_one_wheel = 530; //calibrate after each change made to the prototype
int turning_time_left_using_one_wheel = 650; 
//int turning_time_right_centre = 130; 
//int turning_time_left_centre = 130;
//int turning_time_around_centre = 280;

int threshold_IR = 1.8; // threshold for IR sensor

unsigned long currentmillis, previousmillis = 0, turnmillis = 0, IRmillis = 0, interval = 1000, ledState = 0;

long duration, ultrasound_distance;
long line_follower_left,line_follower_right;

int detecting_mine();
int dummy_or_live();

int mine = 0; //is 1 if found, is 0 if not
int turns = 1; //a variable that is couting how many turns has the robot made
int right_angle = 1; //used for the last part for guiding the robot back to it's starting position
int stop_program = 0; //will run everything until it is not 1
int active = 0; //0 for dummy and 1 for active
int pos = 20, lever_flag = 0; //initial position and direction of servo
int line_follower_starting_variable =0 ; //used to avoid turning at the beginning when robot moves from starting position
int line_follower_final_variable = 0; //used to avoid fluctuation in the line follower sensor readings when it rotates

int variable_1 = 0; //for avoiding ultrasound fluctuation
int variable_2 = 0;
int variable_3 = 0;

void setup() 
{
  Serial.begin (9600);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(AmberLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  pinMode(LFS, INPUT);
  pinMode(RFS, INPUT);
  myservo.attach(servopin);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  //time to take out the usb cable
  Serial.println("waiting");
  int i;
  for (i=0; i<300; i++) 
  { 
    delay(10);
  }
}

void loop() 
{
  if (stop_program == 0)
  {
    
    //light sensor for line following readings
    line_follower_left = analogRead(LFS); 
    line_follower_right = analogRead(RFS); 
    
    //ultrasound readings
    digitalWrite(trigPin, LOW); // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    ultrasound_distance = (duration/2) / 29.1;
    
    currentmillis = millis();
    if (mine == 0)
      blinking(AmberLED);
    
    //if mine is not found use function searching_for_mines
    if (mine == 0)
    {
      line_follower_starting_variable = line_follower_starting_variable + 1 ;
      searching_for_mines(line_follower_right,line_follower_left, ultrasound_distance);
    }
    
    //if mine is found use function mine_found
    if (mine == 1) 
    {
      mine_found(line_follower_right,line_follower_left,ultrasound_distance);
    }
  }
}


//    searching for mine, will make 6 turns and then come back to the starting position if nothing is found, 
//    if found will jump to mine_found function
void searching_for_mines(int line_follower_right,int line_follower_left, int ultrasound_distance)
{
  Serial.println("Searching for mine");
  if (turns < 6) //this amount of turns was chosen so it would fulfill the task in 5 mins
  {
    
    //turning using one wheel when it sees the white line, should turn to the next line 
    if (line_follower_right > line_follower_threshold && line_follower_left > line_follower_threshold &&line_follower_starting_variable >100 )
    { 
      int turning_speed = 150;
      int driving_time = turning_time_right_using_one_wheel;
      driving_backward(turning_speed, driving_time/2); //so while rotating would not touch the box for mines
      turning_to_the_next_line(turning_speed, driving_time, 2); //turn direction = 2, because to the right
      turns = turns + 1; //to estimate how many turns the robot has done
      
    }
    
    //just in case it does not sense anything just go forward
    if (ultrasound_distance >= 300 || ultrasound_distance <= 0)
    { 
      Serial.println("Out of range");
      Serial.println("Driving forward");
      int driving_speed = 150;
      int driving_time = 5;
      driving_forward(driving_speed, driving_time);
      delay(10);
    }
      
    else 
    {
      if (ultrasound_distance <= 35) //avoid robot turning at the wrong time
      {
        variable_3 = variable_2;
        variable_2 = variable_1;
        variable_1 = 1;
      }
      if (ultrasound_distance > 35) //variable were added, because the ultrasound sensor was fluctuating
      {
        variable_3 = variable_2;
        variable_2 = variable_1;
        variable_1 = 0;
      }
      
      //This is turning using one wheel when it sees the wall 
      if (ultrasound_distance<=35 && variable_1 == 1 && variable_2 == 1 && variable_3 == 1) //turns if all 3 variables are 1 (3 in a row readings are correct)
      {
        Serial.println("rotate");
        int turning_speed = 150; 
        int driving_time = turning_time_left_using_one_wheel;//for the compiler
        int turn_direction = 1; //turn_direction = 1 if left, = 2 if right
        turning_to_the_next_line(turning_speed, driving_time, turn_direction);
        
        turns = turns + 1; //to estimate how many turns it will do
        variable_1 =variable_2 = variable_3 = 0;
        
        int driving_speed=150;
        driving_time=3000; 
        driving_backward(driving_speed, driving_time); //driving backwards to adjust to the walls
        
        turnmillis = millis();
      }
      
      //function for detecting mine 
      mine = detecting_mine();
      if (mine == 1) //mine detected
      {
        Serial.println("mine is detected in this round");
        digitalWrite(AmberLED, HIGH);
        leftMotor -> run(RELEASE);
        rightMotor -> run(RELEASE);
        delay(1000);
        int driving_speed = 150;
        int driving_time = 5;
        driving_forward(driving_speed, driving_time);
        delay(500);
        rightMotor -> run(RELEASE);
        leftMotor -> run(RELEASE);
        active = dummy_or_live(); //drive forward to utilize the hall effect
      }
      
      if (ultrasound_distance > 35 && mine == 0)   
      {
        Serial.println("Driving forward");  
        int driving_speed = 150;
        int driving_time = 5; 
        driving_forward(driving_speed, driving_time); 
        delay(10);
      }
    }
  }
  if (turns == 6) // will start driving back to the starting position
  {

    if (ultrasound_distance <= 35)
    {
      variable_3 = variable_2;
      variable_2 = variable_1;
      variable_1 = 1;
    }
    if (ultrasound_distance > 35)
    {
      variable_3 = variable_2;
      variable_2 = variable_1;
      variable_1 = 0;
    }
    
    line_follower_final_variable = line_follower_final_variable + 1;

    // right_angle variable makes sure that the turns are made one after another, prevents incorrect movement of the robot
    if (ultrasound_distance <= 35 && ultrasound_distance >= 20 && right_angle == 2 && variable_1 == 1 && variable_2 == 1 && variable_3 == 1) 
    {
      //the robot will be turning right to place itself in the starting position
      int turning_speed = 150;
      int driving_time = 250; 
      turning_to_the_next_line(turning_speed, driving_time, right_angle);
      rightMotor->run(RELEASE);
      leftMotor->run(RELEASE);
      delay(1000);
      
      stop_program = 1; //will stop the robot from moving

      //in case stop_program does not work, use the loop so it would be enough time to unplug the battery
      Serial.print("stop running");
      int i;
      for (i=0; i<1000; i++) 
      { 
        delay(10);
      }
    }

    //turning left once the white line is found
    if ((line_follower_right > line_follower_threshold && line_follower_left > line_follower_threshold && line_follower_final_variable >= 50) && right_angle == 1)
    {
      int turning_speed = 150;
      int driving_time = 230;  //need to be calibrated
      driving_backward(turning_speed, driving_time/2); //so while rotating would not touch the box for mines
      turning_to_the_next_line(turning_speed, driving_time, right_angle); // turn to the left 90 degrees
      right_angle = 2; 
    }

    //following the line
    if (line_follower_left > line_follower_threshold && right_angle == 2)
    {
      line_follower(1);
    }
    if (line_follower_right > line_follower_threshold && right_angle == 2)
    { 
      line_follower(2);
    }
    
    else 
    {
      Serial.println("Driving forward"); 
      int driving_speed = 150;
      int driving_time = 10;
      driving_forward(driving_speed, driving_time); //might worth checking what would change if put release at the end
      delay(10);
    }
  }
}

//function that is running when the mine = 1 (it is found) 
void mine_found(int line_follower_right,int line_follower_left,int ultrasound_distance)
{
  Serial.println("mine is found");   
  leftMotor -> run(RELEASE);
  rightMotor -> run(RELEASE); //stops driving
  //controlling LED
  if(active == 0)
    digitalWrite(GreenLED, HIGH);
  delay(5000);
  digitalWrite(AmberLED, LOW);
  digitalWrite(GreenLED, LOW);
  mine = 0;
  //turning to avoid mine sticking to robot or detecting it twice
  int turning_speed = 150;
  int driving_time = 100;  //not calibrated yet
  turning_about_its_centre(turning_speed, driving_time, 1);
  driving_backward(turning_speed, driving_time);
  delay(500);
  turning_about_its_centre(turning_speed, driving_time, 2);
  driving_forward(turning_speed, driving_time);
  delay(500);
  leftMotor -> run(RELEASE);
  rightMotor -> run(RELEASE);
  Serial.println("wiggle successful");
  delay(1000);
}

//function that commands to drive forward
void driving_forward(int driving_speed, int driving_time)
{
  rightMotor->run(BACKWARD); 
  leftMotor->run(FORWARD);
  int i;
  for (i=0; i < driving_time; i++) 
  { 
    leftMotor->setSpeed(driving_speed); //sets speed for both motors
    rightMotor->setSpeed(driving_speed);
  }
}

//function that commands to drive backwards
void driving_backward(int driving_speed, int driving_time)
{
  rightMotor->run(FORWARD); 
  leftMotor->run(BACKWARD);
  int i;
  for (i=0; i < driving_time; i++) 
  { 
    leftMotor->setSpeed(driving_speed); //sets speed for both motors
    rightMotor->setSpeed(driving_speed);
  }
}

//function that commands to turn to the next line (turn 180, but using one wheel only
void turning_to_the_next_line(int turning_speed, int driving_time, int turn_direction)
{
  AFMS.begin();  // create with the default frequency 1.6KHz // not sure if this is needed
  int i;
  
  if (turn_direction == 1)
  { //turning left
    rightMotor->run(BACKWARD);
    for (i=0; i < driving_time; i++) 
    { 
      rightMotor->setSpeed(turning_speed);
      delay(10);
    }
    rightMotor->run(RELEASE);
    delay(1000);
  }
  
  if (turn_direction == 2)
  {  //turning right
    leftMotor->run(FORWARD);
    int i;
    for (i=0; i < driving_time; i++) 
    { 
      leftMotor->setSpeed(turning_speed);
      delay(10);
    }
    leftMotor->run(RELEASE);
    delay(1000);
  }  
}


//function that commands to turn around, keeping the mass centre in the same place(turning both wheels)
void turning_about_its_centre(int turning_speed, int driving_time, int turn_direction)
{ 
  //turn_direction = 1 if left = 2 if right

  AFMS.begin();  // create with the default frequency 1.6KHz //not sure if this is needed
  int i;
  
  if (turn_direction == 1)
  { //turning left
    rightMotor->run(BACKWARD);
    leftMotor->run(BACKWARD);
    for (i=0; i < driving_time; i++) 
    { 
      rightMotor->setSpeed(turning_speed); //sets speed for both motors
      leftMotor->setSpeed(turning_speed);
      delay(10);
    }
    rightMotor->run(RELEASE);
    leftMotor->run(RELEASE);
    delay(1000);
  }

  if (turn_direction == 2)
  {  //turning right
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    int i;
    for (i=0; i < driving_time; i++) 
    { 
      leftMotor->setSpeed(turning_speed); //sets speed for both motors
      rightMotor->setSpeed(turning_speed);
      delay(10);
    }
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    delay(1000);
  }  
}


//function for following the line
void line_follower(int right_or_left) //if right = 1, if left = 2
{
  AFMS.begin();  
  int speedmotor = 150;
  int driving_time = 20;
  int i;
  rightMotor->run(BACKWARD); 
  leftMotor->run(FORWARD);
  
  if (right_or_left == 1)
  { //left sensor, means needs to rotate to left
    for (i=0; i<driving_time ; i++)
    {
      rightMotor->setSpeed(speedmotor);
      delay(10);
    }
    rightMotor->run(RELEASE);
  }
  
  if (right_or_left == 2){ //right sensor, means needs to rotate to right
    for (i=0; i<driving_time ; i++){
      leftMotor->setSpeed(speedmotor);
      delay(10);
    }
    leftMotor->run(RELEASE);
  }
}

//function for utilizing the IR sensor to detect the mine
int detecting_mine()
{
  float VIR1 = 0.00, VIR2 = 0.00, cm = 0.00;
  //digitalWrite(IR_input, HIGH);
  VIR1 = analogRead(LeftIR)*5.0/1024.0;  //sensor on the lever, which is A3
  //VIR2 = analogRead(RightIR)*5.0/1024.0;  //sensor on the stationary bit, which is A2
  Serial.println(VIR1);
  //Serial.print("  ");
  //Serial.println(VIR2);
  IRmillis = millis();
  if (ultrasound_distance >= 60 && IRmillis - turnmillis >= 5000)
  {
    if(VIR1 <= threshold_IR && VIR2 <= threshold_IR)
      return 0;
    else
      return 1;
  }
  else
    return 0;
}

//function for detecting whether the mine is live or not using the hall effect and comparator
int dummy_or_live()
{
  int hall;
  hall = digitalRead(HallEffect);
  //hall effect 1 = no mine, 0 = mine
  if (hall == HIGH)
  {
    Serial.println("dummy mine");
    return 0; // high means dummy mine
  }
  else
  {
    Serial.println("live mine");
    return 1; // low means active mine
  }
}

//function for controlling LED blinking as required
void blinking(int ledPin)
{
  int ledState;
  currentmillis = millis();
  if (currentmillis - previousmillis >= interval) 
  {
    previousmillis = currentmillis;
    if (ledState == 0) 
    {
      ledState = 1;
    } 
    else 
    {
      ledState = 0;
    }
    digitalWrite(ledPin, ledState);
  }    
}
