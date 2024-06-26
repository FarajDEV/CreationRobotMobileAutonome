#include <Arduino.h>
#include <limits.h>
#include <math.h>
#include <ld06.h>
#include <AccelStepper.h>




//PARAMETRES:
float current_x=0; 
float current_y=0;
int target_x=10;
float target_y=10;
float current_theta=0;
float target_theta=2;







//VARIABLES ET CONSTANTES:
const int RED_LED_PIN = 23;
const int GREEN_LED_PIN = 52;
const int DATA_BYTE = 36;
const int TOTAL_DATA_BYTE = 48;
char start_byte;
char data_length;
float speed;
float FSA;
float LSA;
int time_stamp;
int CS;
float angle;
float angle_step;
char tmpChars[TOTAL_DATA_BYTE];
const int MAX_SIZE = 48;
unsigned int angles[MAX_SIZE];
float distances[MAX_SIZE];
int confidences[MAX_SIZE];
int pinTrig = 47;
int pinEcho = 46;
float distance2;
const int STEPS_PER_REVOLUTION = 200; 
const float RPM_TO_DELAY = 60000.0; 
double  current_theta_radians;
const int num_waypoints = 100; 
const int lignes= 10;
const int colonnes = 10;
float waypoints[num_waypoints][2];
int grid[lignes][colonnes] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
const int bufferSize = 32; 
uint8_t buffer[bufferSize];
int bytesRead = 0;
unsigned long previousMillis = 0;  
const long interval = 100000;       
float x;
float y;
float waypoint_x = target_x;
float waypoint_y = target_y;
unsigned long lastStepTime = 0; 
unsigned long lastCalledTime = 0; 
int i=0;
int j=0;
typedef void (*FunctionPointer)();
#define DIR_PIN1 2
#define STEP_PIN1 3
#define SLEEP_PIN1 4
#define RESET_PIN1 4
#define MS1_PIN1 5
#define MS2_PIN1 6
#define MS3_PIN1 7
#define DIR_PIN2 8
#define STEP_PIN2 9
#define SLEEP_PIN2 10
#define RESET_PIN2 10
#define MS1_PIN2 13
#define MS2_PIN2 12
#define MS3_PIN2 11
volatile long left_steps = 0;
volatile long right_steps = 0;
enum Operation {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};
Operation lastOperation = FORWARD;
const int DISTANCE_THRESHOLD = 50; 
const float stepsPerRevolution = 200.0; 
const float wheelDiameter = 65.0; 
const float wheelBase = 245.0; 






//Partie Setup
void setup() {
   read_lidar_data();
  Serial1.begin(230400);
  Serial.begin(9600); 
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  digitalWrite(pinTrig, LOW);  
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(SLEEP_PIN1, OUTPUT);
  pinMode(RESET_PIN1, OUTPUT);
  pinMode(MS1_PIN1, OUTPUT);
  pinMode(MS2_PIN1, OUTPUT);
  pinMode(MS3_PIN1, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(SLEEP_PIN2, OUTPUT);
  pinMode(RESET_PIN2, OUTPUT);
  pinMode(MS1_PIN2, OUTPUT);
  pinMode(MS2_PIN2, OUTPUT);
  pinMode(MS3_PIN2, OUTPUT);
  digitalWrite(MS1_PIN1, LOW);
  digitalWrite(MS2_PIN1, LOW);
  digitalWrite(MS3_PIN1, LOW);
  digitalWrite(MS1_PIN2, LOW);
  digitalWrite(MS2_PIN2, LOW);
  digitalWrite(MS3_PIN2, LOW);
  digitalWrite(SLEEP_PIN1, HIGH);
  digitalWrite(RESET_PIN1, HIGH);
  digitalWrite(SLEEP_PIN2, HIGH);
  digitalWrite(RESET_PIN2, HIGH);
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW); 
}






//Partie loop:
void loop() {
executePath(target_x, target_y);
}





// La fonction principale qui appelle toutes les autres fonctions pour réaliser toutes les taches de notre robot:
void executePath(float target_x, float target_y) {
   calculateWaypoints(current_x, current_y, target_x, target_y, num_waypoints, waypoints);

while( !reachedTarget(current_x, current_y, current_theta, target_x, target_y,  target_theta)){
    
    for (int i = 0; i < num_waypoints; i++) {
     
        float waypoint_x = waypoints[i][0];
        float waypoint_y = waypoints[i][1];
        float angle_to_waypoint = atan2(waypoint_y - current_y, waypoint_x - current_x);
        float distance_to_waypoint = sqrt(pow(waypoint_x - current_x, 2) + pow(waypoint_y - current_y, 2));

       
         
        planPath(current_x, current_y, waypoint_x, waypoint_y);
     updateOdometry();
        rotateToAngle(angle_to_waypoint);
   Serial.print(  "The value of Z:"); 
         angle = get_angle(Serial1.read(), TOTAL_DATA_BYTE);
        if(!obstacleDetected( distance2)) {
         
           customDelay(200, avant );
              
            updateOdometry();
         
            distance_to_waypoint = sqrt(pow(waypoint_x - current_x, 2) + pow(waypoint_y - current_y, 2));
            Serial.print(  "The value of SAWSN:");
        }
        else{
                 
            
            calculateWaypoints(current_x, current_y, target_x, target_y, num_waypoints - i, waypoints + i); 
             float angle = get_angle(Serial1.read(), TOTAL_DATA_BYTE);
         avoidObstacle(distance2, angle); 
         updateOdometry();
            
        }
       
        
    }

}
}











//Toutes les fonctions utilisées:
//fonction direction à droite du robot:
void droite() {
  int rpm = 150; 
  int stepDelay = calculateStepDelay(rpm);
  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  delayMicroseconds(stepDelay);
}

//fonction direction à gauche du robot:
void gauche() {
  int rpm = 150;

  int stepDelay = calculateStepDelay(rpm); 
  digitalWrite(DIR_PIN1, HIGH);
  digitalWrite(DIR_PIN2, HIGH); 
  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  delayMicroseconds(stepDelay);
    left_steps++;
      right_steps++;
}


//fonction aller de l'avant du robot:
void avant() {
  int rpm = 150; 

  int stepDelay = calculateStepDelay(rpm); 
  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  digitalWrite(DIR_PIN1, LOW); 
  digitalWrite(DIR_PIN2, HIGH); 
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  delayMicroseconds(stepDelay);
      left_steps++;
      right_steps++;
}


//fonction marche arrière du robot:
void arriere() {
  int rpm = 150;

  int stepDelay = calculateStepDelay(rpm); 

  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  digitalWrite(DIR_PIN1, HIGH); 
  digitalWrite(DIR_PIN2, LOW);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  delayMicroseconds(stepDelay);
        left_steps++;
      right_steps++;
}

//fonction s'arreter du robot:
void stop() {
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);
  digitalWrite(DIR_PIN1, LOW);
  digitalWrite(DIR_PIN2, LOW);
}



//fonction calcul delai:
int calculateStepDelay(int rpm) {
  int stepsPerMinute = stepsPerRevolution * rpm;
  int stepsPerSecond = stepsPerMinute / 60;
  int stepDelay = 1000000 / stepsPerSecond;
  return stepDelay;
}



//fonction calcul delai pers:
void customDelay(unsigned long duration, FunctionPointer func) {
  unsigned long startTime = millis(); 
  while (millis() - startTime < duration) {
    func();
  }
  stop(); 
}
 

// La fonction pour éviter les obstacles:
void avoidObstacle(int distance, int angle) {
    angle = get_angle(Serial1.read(), TOTAL_DATA_BYTE);
    float distance_F = Serial1.read(); 

    if (distance_F < 100) {
    
        digitalWrite(RED_LED_PIN, HIGH);  
        digitalWrite(GREEN_LED_PIN, LOW); 

        if (angle < 90) {
            // Rotate left
            customDelay(50, stop);
            customDelay(500, arriere);
            customDelay(100, gauche);
        } else {
            customDelay(50, stop);
            customDelay(500, arriere);
            customDelay(100, droite);
        }
    } else {
 
        digitalWrite(RED_LED_PIN, LOW);  
        digitalWrite(GREEN_LED_PIN, HIGH); 

        continueOperation();
    }
}





//fonction continuer opération après une étape:
void continueOperation() {
  switch (lastOperation) {
    case FORWARD:
       customDelay(200, avant );
      break;
    case BACKWARD:
       customDelay(40, arriere );
      break;
    case LEFT:
       customDelay(40,gauche  );
      break;
    case RIGHT:
       customDelay(40,droite  );
      break;
  }
}







//fonction qui retourne true or false si on arrive à un point spécifié de la map:
bool reachedTarget(float current_x, float current_y, float current_theta, float target_x, float target_y, float target_theta) {
  float distance_threshold = 50.0; 
  float angle_threshold = 1; 
  return (abs(current_x - target_x) < distance_threshold &&
          abs(current_y - target_y) < distance_threshold &&
          abs(current_theta - target_theta) < angle_threshold);
}







//fonction qui fait que le robot pivote d'un angle pour arriver au point voulu avec l'angle voulu:
void rotateToAngle(float target_angle) {
   
    float angle_diff = target_angle - current_theta;
    if (angle_diff > PI) {
        angle_diff -= 2 * PI;
    } else if (angle_diff < -PI) {
        angle_diff += 2 * PI;
    }
    int speed1, speed2;
    unsigned long rotationDuration;

    if (angle_diff > 0) {
        speed1 = 500;
        speed2 = -500;
    } else {
        speed1 = -500;
        speed2 = 500;
        angle_diff = -angle_diff; 
    }

    if (speed1 > 0) {
        customDelay(200, avant);
    } else {
        customDelay(40, arriere);
    }

    if (speed2 > 0) {
        customDelay(40, gauche);
    } else {
        customDelay(40, droite);
    }

   
    current_theta = target_angle;

    stop();
}
 



//fonction pour détecter si il y a un obstacle:
float distance_F=Serial1.read();
 bool obstacleDetected(int distance) {
 if(distance_F < 12) {
  return true;
 }
 else{
  return false;
 }
}



//fonction pour renouveler l'odométrie à chaque instant
void updateOdometry() {  
  float left_distance = left_steps * (PI * wheelDiameter / stepsPerRevolution);
  float right_distance = right_steps * (PI * wheelDiameter / stepsPerRevolution);
  float distance = (left_distance + right_distance) / 2.0;
  float angle_diff = target_theta - current_theta;
   current_theta+= angle_diff;
 current_theta_radians = current_theta * M_PI / 180.0;
  current_x += distance * cos(current_theta_radians);
  current_y += distance * sin(current_theta_radians);
 Serial.print(  "The value of x:");
 Serial.print(  current_x);
  Serial.print(  "The value of y:");
 Serial.print( current_y );

  left_steps = 0;
  right_steps = 0;

}


// Fonction qui permet de mettre un plan à la meilleure trajectoire à chaque instant
void planPath(int current_x, int current_y, int target_x, int target_y) {

    int dx = abs(target_x - current_x);
    int dy = abs(target_y - current_y);


    if (dx > dy) {
        if (current_x < target_x) {
              customDelay(40, droite);
        } else if (current_x > target_x) {
             customDelay(40,gauche );
        }
    }
    else {
        if (current_y < target_y) {
            // Move forward
              customDelay(200, avant);
        } else if (current_y > target_y) {
            // Move backward
             customDelay(40, arriere );
        }
    }
}







// Fonction qui permet de calculer les points de passage du robot:
void calculateWaypoints(float start_x, float start_y, float goal_x, float goal_y, int num_waypoints, float waypoints[][2]) {
    for (int i = 0; i < num_waypoints; i++) {
        float ratio = float(i + 1) / float(num_waypoints + 1);
        waypoints[i][0] = start_x + (goal_x - start_x) * ratio;
        waypoints[i][1] = start_y + (goal_y - start_y) * ratio;
    }
}





// Fonction qui permet d'avoir l'angle avec lequel le robot doit tourner:
float get_angle(char values[], int length) {
  start_byte = values[0];
  data_length = 0x1F & values[1];
  speed = float(values[3] << 8 | values[2]) / 100;
  FSA = float(values[5] << 8 | values[4]) / 100;
  LSA = float(values[length - 4] << 8 | values[length - 5]) / 100;
  time_stamp = int(values[length - 2] << 8 | values[length - 3]);
  CS = int(values[length - 1]);
  if (LSA - FSA > 0) {
    angle_step = (LSA - FSA) / (data_length - 1);
  } else {
    angle_step = (LSA + (360 - FSA)) / (data_length - 1);
  }
  if (angle_step > 20) {
    return -1;
  }
  unsigned int raw_deg = FSA;
  float angle = (raw_deg <= 360 ? raw_deg : raw_deg - 360);
  return angle;
}






// Fonction qui permet de lire les données du lidar:
void read_lidar_data() {
 
  if (!Serial1.available()) {
    return;
  }

  bool loopFlag = true;
  char tmpChars[TOTAL_DATA_BYTE] = {'\0'};
  int dataIndex = 0;

  while (loopFlag) {
    char tmpInt = Serial1.read(); 

    // Check for start of data frame
    if (tmpInt == 0x54 && dataIndex == 0) {
      tmpChars[dataIndex++] = tmpInt; 
      continue;
    }
    if (dataIndex > 0 && dataIndex < TOTAL_DATA_BYTE) {
      tmpChars[dataIndex++] = tmpInt;
      if (dataIndex == TOTAL_DATA_BYTE) {
        loopFlag = false; 
        float angle = get_angle(tmpChars, TOTAL_DATA_BYTE);
        Serial.print("Calculated Angle: ");
        Serial.println(angle);
        memset(tmpChars, 0, sizeof(tmpChars)); 
        dataIndex = 0; 
      }
    }
  }
}




