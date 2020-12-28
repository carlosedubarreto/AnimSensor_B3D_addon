#define LEDPIN 13
#define msgSize 30    // Sample message will be of the form P:0.401;R:2.932;Y:3.653@ or P:-0.40;R:-8.12;Y:-6.59
#define pi  3142      // In thousandths
  
int   pitch_err;   // Error variables from the raspberry
int   roll_err;
int   increment;
long  err_sum;    // Accumulated error for PID
char  container[msgSize]; 

const word pitchMotorPin1   = 3;         // Pitch motor control pins
const word pitchMotorPin2   = 5;
const word pitchMotorPin3   = 6;
const word pwmPitchPin      = 7;         // pwm pitch reading Pin
const word rollMotorPin1    = 9;         // Roll motor control pins
const word rollMotorPin2    = 10;
const word rollMotorPin3    = 11;
const word pwmRollPin       = 12;        // pwm roll reading Pin

const int gimbalMaxReading  = 919;
const int angleResolution   = 50;      // Angle resolution in X * 10^-3 

int pwmSin[]= {0,1,2,4,6,8,12,15,19,24,29,34,40,
              46,52,59,66,73,80,88,95,103,111,119,
              127,135,143,151,159,166,174,181,188,
              195,202,208,214,220,225,230,235,239,
              242,246,248,250,252,253,254,255,254,
              253,252,250,248,246,242,239,235,230,
              225,220,214,208,202,195,188,181,174,
              166,159,151,143,135,127,119,111,103,
              95,88,80,73,66,59,52,46,40,34,29,24,
              19,15,12,8,6,4,2,1,0};              // array of PWM duty values - sine function
              
int arraySize = (sizeof(pwmSin)/sizeof(int)) -1;                // Goes from index 0 to arraySize
int pitchStepA = 0; int pitchStepB = (int) (arraySize /3); int pitchStepC = (int) (arraySize*2 /3);   // Stepping sequence for pitch motor
int rollStepA = 0; int rollStepB = (int) (arraySize /3); int rollStepC = (int) (arraySize*2 /3);   // Stepping sequence for pitch motor

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  }

void loop() {
  // Reads the serial monitor for the inputs from raspberry pi
  // If no input yet, wait till something is available
  // problem is that serial data is being sent twice even though we don't input in anything. Even though the serial buffer is empty, it still prints
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
//    Serial.print(pitch_err); Serial.print(" "); Serial.println(roll_err);
//    Serial.println(str);
    moveRollMotor();
    movePitchMotor();         // Motor correction
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
      int value = extractInt(++separator);
      result = *command;

      switch(result){
        case 'P':
          pitch_err = value;
          break;

        case 'R':
          roll_err = value;
          break;
        }
      }
    command = strtok(NULL, ";");
  }
}

void signalAvailable(){
  // Turns on when processing serial inputs
  digitalWrite(LEDPIN, HIGH);
  delay(100);
  }

void serialWaiting(){
  // Turns off when processing serial inputs
  digitalWrite(LEDPIN, LOW);
  delay(100);
  }
  
int extractFloat(char* ptr){
  // Gets the int data from the separator
  String tmp = "";

  while(*ptr != NULL){
    tmp += *ptr;        // Int digit terminates at NULL
    ptr++;
    }
// Serial.println(tmp);
 return tmp.toInt();
}

void movePitchMotor(){
  // Function to move the motor to correct for pitch errors
  unsigned long last_time = 0.0;
  int currentAngleNow = readCurrentAngle(pwmPitchPin);
  float desiredAngle = currentAngleNow - pitch_err;
  float in_err = desiredAngle - currentAngleNow;
  float last_err = in_err;
  float out_err; 
  if (desiredAngle < 0){desiredAngle = 2*pi + desiredAngle;}
  else if (desiredAngle >= 2*pi){desiredAngle = desiredAngle - 2*pi;}
  
  while(abs(in_err) > angleResolution){
    // Apply PID control

    out_err = PID_controller(in_err, last_err, last_time, 1.0, 0.1, 0.0);
    last_time = millis();
      
    increment = (int) mapfloat(out_err, -pi/4, pi/4, -15.0,15.0);

    pitchStepA = checkLimits(pitchStepA + increment);   
    pitchStepB = checkLimits(pitchStepB + increment);   
    pitchStepC = checkLimits(pitchStepC + increment);
    
    analogWrite(pitchMotorPin1, pwmSin[pitchStepA]);    // Move the pitch motor
    analogWrite(pitchMotorPin2, pwmSin[pitchStepB]);
    analogWrite(pitchMotorPin3, pwmSin[pitchStepC]);
    delay(10);

//    Serial.print(desiredAngle); Serial.print(" ");
//    Serial.println(currentAngleNow);
//    Serial.println("");
    currentAngleNow = readCurrentAngle(pwmPitchPin);                    // Resample reading
    last_err = in_err;
    
    if(abs(desiredAngle - currentAngleNow) <= 2*pi - abs(desiredAngle - currentAngleNow)){in_err = desiredAngle - currentAngleNow;} 
    else if(desiredAngle < currentAngleNow){in_err = 2*pi - abs(desiredAngle - currentAngleNow);}
    else in_err = -2*pi + abs(desiredAngle - currentAngleNow);
    }
//    Serial.print(desiredAngle); Serial.print(" ");
//    Serial.println(currentAngleNow);
//    Serial.println("");
  }


void moveRollMotor(){
  // Function to move the motor to correct for roll errors
  unsigned long   last_time = 0.0;
  unsigned int    currentAngleNow = readCurrentAngle(pwmRollPin);
  int             desiredAngle = currentAngleNow + roll_err;
  int             in_err = desiredAngle - currentAngleNow;
  int             last_err = in_err;
  int             out_err; 
  err_sum = 0;
  
  if (desiredAngle < 0){desiredAngle = 2*pi + desiredAngle;}
  else if (desiredAngle >= 2*pi){desiredAngle = desiredAngle - 2*pi;}
  
  while(abs(in_err) > angleResolution){
    // Apply PID control

    out_err = PID_controller(in_err, last_err, last_time, 0.5, 0.1, 0.0);     // PID with gains 
    last_time = millis();
    
    increment = (int) mapfloat(out_err, -pi/4, pi/4, -15.0,15.0);
//    Serial.print(roll_err); Serial.print(" ");
    
    rollStepA = checkLimits(rollStepA - increment);   
    rollStepB = checkLimits(rollStepB - increment);   
    rollStepC = checkLimits(rollStepC - increment);
    
    analogWrite(rollMotorPin1, pwmSin[rollStepA]);    // Move the roll motor
    analogWrite(rollMotorPin2, pwmSin[rollStepB]);
    analogWrite(rollMotorPin3, pwmSin[rollStepC]);
//    Serial.println(desiredAngle); Serial.println(currentAngleNow);
    delay(10);
      
    currentAngleNow = readCurrentAngle(pwmRollPin);                    // Resample reading
    last_err = in_err;

    if(abs(desiredAngle - currentAngleNow) <= 2*pi - abs(desiredAngle - currentAngleNow)){in_err = desiredAngle - currentAngleNow;} 
    else if(desiredAngle < currentAngleNow){in_err = 2*pi - abs(desiredAngle - currentAngleNow);}
    else in_err = -2*pi + abs(desiredAngle - currentAngleNow);
    }
//    Serial.print(desiredAngle); Serial.print(" ");
//    Serial.print(increment); Serial.print(" ");
//    Serial.println(currentAngleNow);
//    Serial.println("");
  }
  
  
int inline readCurrentAngle(int pwmPin){
  // Reads the current angle from the connection 
  // pwmPin1 var connected to PWM pin of the motor
  int val = pulseIn(pwmPin, LOW);
  return mapInt(val, 0, gimbalMaxReading, 0, 2*pi);
  }

int inline mapInt(int x, int in_min, int in_max, int out_min, int out_max){
  // Map function for handling floating point numbers
 int val = out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
 return max(min(val, out_max), out_min);
}

int inline checkLimits(int currentStep){
  // Function checks whether the current step goes beyond the limit and pegs it to the limit
  if(currentStep > arraySize){
    return currentStep - arraySize -1;
    }
  else if(currentStep < 0){
    return arraySize + currentStep +1;
    }
  else{
    return currentStep;
    }
  }

int inline PID_controller(int in_err, int last_err, unsigned long last_time, int Kp, int Kd, int Ki){
  // Implementation of PID controller with gains 
  unsigned long now = millis();
  float timeChange = (float)(now - last_time);
  /*Compute all the working error variables*/

  float dErr = (in_err - last_err) / timeChange;

  err_sum += timeChange * in_err;
//  Serial.print(in_err); Serial.print(" "); Serial.print(dErr); Serial.print(" "); Serial.println(err_sum);
  /*Compute PID Output*/
  return Kp * in_err + Kd * dErr + Ki * err_sum;
  }
