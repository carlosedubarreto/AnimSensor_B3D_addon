#include <VarSpeedServo.h>
#define LEDPIN    13
#define msgSize   30      // Sample message will be of the form P:0.401;R:2.932;Y:3.653@ or P:-0.40;R:-8.12;Y:-6.59
#define pi        3.142

const int pitch_pin   = 9;
const int roll_pin    = 6;
 
int pitch_err_deg;      // variable to read the value from the analog pin 
int pitch_speed;
int roll_err_deg;       // variable to read the value from the analog pin 
int roll_speed;
const int initialServoAngle = 90;
int currentServoAngle_roll = initialServoAngle;
int currentServoAngle_pitch = initialServoAngle;

float pitch_err;   // Error variables from the raspberry
float Roll_err;
float roll_err;
byte Size;
char container[msgSize]; 

/* Function prototypes */
void parseReading(char* container);
void signalAvailable();
void serialWaiting();
int inline mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
float extractFloat(char* ptr);
 
VarSpeedServo pitchservo;   // create servo object to control a servo 
VarSpeedServo rollservo;    // create servo object to control a servo 

void setup() 
{ 
  Serial.begin(115200);
  pitchservo.attach(pitch_pin);                             // attaches the servo on pin 9 to the servo object 
  rollservo.attach(roll_pin);                              // attaches the servo on pin 9 to the servo object 
  pitchservo.write(initialServoAngle, 20, false);   //Initialize both servo positions
  rollservo.write(initialServoAngle, 20, false);
} 
 
void loop() {
  // Reads the serial monitor for the inputs from raspberry pi
  // If no input yet, wait till something is available
  if(Serial.available() > 0){
    String str="";
    signalAvailable();            // If signal received -> Light up LED
    memset(container, 0, sizeof(container));

    char c = Serial.read();
    if (c =='#'){
      c = Serial.read();
      while(c != '&'){
        str += c;
        c = Serial.read();}
    str.toCharArray(container, sizeof(container));
    parseReading(container);

    /// COMPENSATE FOR PITCH ERROR
    pitch_err_deg = mapfloat(pitch_err, -pi/2, pi/2, -89, 89);        // scale it to use it with the servo (value between 0 and 180) 
    pitch_speed   = map(abs(pitch_err_deg), 0, 30, 100, 130);        // Controls the speed of the turn: 1 - slowest, 255 - fastest
    currentServoAngle_pitch = currentServoAngle_pitch - pitch_err_deg;    
    pitchservo.write(currentServoAngle_pitch, pitch_speed, false);        // sets the servo position according to the scaled value 
    
    delay(10);                           // waits for the servo to get there 
    
    /// COMPENSATE FOR ROLL ERROR
    roll_err_deg = mapfloat(roll_err, -pi/2, pi/2, -89, 89);        // scale it to use it with the servo (value between 0 and 180) 
    roll_speed   = map(abs(roll_err_deg), 0, 30, 100, 130);       // Controls the speed of the turn: 1 - slowest, 255 - fastest
    currentServoAngle_roll = currentServoAngle_roll - roll_err_deg; //MIGHT BE MINUS NOT PLUS
    rollservo.write(currentServoAngle_roll, roll_speed, false);                  // sets the servo position according to the scaled value 
  
//    Serial.print(pitch_speed); Serial.print(" "); Serial.print(roll_speed); Serial.println(" ");
    delay(10);                           // waits for the servo to get there 
    }
  }
  else{
    serialWaiting();
    }   
  }

void parseReading(char* container){
// Expect sample data from raspberry to be: Pitch=<pitch>;Roll=<roll>;Yaw=<yaw>;@
// Parses the serial input of tilt angles from the raspberry and assigns the values to pitch / roll / yaw errors
char* command = strtok(container, ";");
char result;

while(command != NULL){
  char* separator = strchr(command, ':');  // Splits each reading to individual components 
  if(separator != 0){
    *separator = 0;
    float value = extractFloat(++separator);
    result = *command;

    switch(result){
      case 'P':
        pitch_err = value;         
      case 'R':
        roll_err = value;     
        break;
      }
    }
  command = strtok(NULL, ";");
  }
}

void signalAvailable(){
  // Turns on when processing  inputs
  digitalWrite(LEDPIN, HIGH);
 // delay(25);
}

void serialWaiting(){
  // Turns off when processing serial inputs
  digitalWrite(LEDPIN, LOW);
  delay(250);
}

float extractFloat(char* ptr){
  // Gets the float data from the separator
  String tmp = "";

  while(*ptr != NULL){
    tmp += *ptr;        // Float digit terminates at NULL
    ptr++;
    }
// Serial.println(tmp);
 return tmp.toFloat();
}

int inline mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  // Map function for handling floating point numbers
 int val = (float)out_min + (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min);
 return max(min(val, out_max), out_min);
}
