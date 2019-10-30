// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
// #define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
// #include <FreqCount.h>
// #include <FreqMeasure.h>

#define ENC_A_PIN1 29
#define ENC_A_PIN2 30
#define M_PWM 6
#define M_DIR1 8
#define M_DIR2 9

#define ENC_B_PIN1 31
#define ENC_B_PIN2 32
#define M2_PWM 7
#define M2_DIR1 11
#define M2_DIR2 12

#define HALL_ENGINE 13

Encoder enc_a(ENC_A_PIN1, ENC_A_PIN2);
Encoder enc_b(ENC_B_PIN1, ENC_B_PIN2);

int e, esum;
int setpoint;
double Ki = .5;
double Kp = 1;
double push = 0;
static int error_sum1 = 0;
static int error_sum2 = 0;
int incomingByte = 0;
double gear_ratio = 2.0;

double ratio [8] = {.75, 1.0, 1.5, 2.0, 2.5, 3, 3.5, 4.0};
double primary_pos[8] = {15402.0, 12534.0, 8391.0, 5523.0, 3505.0, 744.0, 0.0}; 
double secondary_pos[8] = {16347.0, 12523.0,7724.0, 4818.0, 2944.0, 1568.0, 593.0, 0.0}; 

double sat(int push){
    if (push > 255){
        return 255;
    }
    else if (push < -255){
        return -255;
    }
    return push;
}

void spinMotor(int DIR1_PIN, int DIR2_PIN, int PWM_PIN, double duty){
    if (duty > 0){
        //Spin foreward
        digitalWrite(DIR1_PIN, HIGH);
        digitalWrite(DIR2_PIN, LOW);
    } else{
        digitalWrite(DIR2_PIN, HIGH);
        digitalWrite(DIR1_PIN, LOW);
    }
    analogWrite(PWM_PIN, abs(duty));
}

// Give a duty value of -255 to 255
void spinMotor1(double duty){
    spinMotor(M_DIR1, M_DIR2, M_PWM, duty);
}
void spinMotor2(double duty){
    spinMotor(M2_DIR1, M2_DIR2, M2_PWM, duty);
}


double PIcontrol1(int setpoint, int curr_point, double Kp, double KI){
    int error1 = setpoint - curr_point; 
    error_sum1 += error1; 
    Serial.printf("Setpoint: %d, CurPos: %d\n", setpoint, curr_point);
    return sat(KI*error_sum1 + Kp*error1); 
}

double PIcontrol2(int setpoint, int curr_point, double Kp, double KI){
    int error2 = setpoint - curr_point; 
    error_sum2 += error2; 
    Serial.printf("Setpoint2: %d, CurPos2: %d\n", setpoint, curr_point);
    return sat(KI*error_sum2 + Kp*error2); 
}

int myinterp1(double *sample_points, double *corr_points, double que_value){
  int i = 0; 
  double x_1 = 0.0; 
  double x_2 = 0.0;
  double y_1 = 0.0; 
  double y_2 = 0.0; 
  while (que_value > *(sample_points + i))
  {
    x_1 = *(sample_points + i);
    x_2 = *(sample_points + i + 1);
    y_1 = *(corr_points + i);
    y_2 = *(corr_points + i + 1);
    i++;
  }
  return (int)(y_1 + ((y_2 - y_1)/(x_2 - x_1))*(que_value - x_1)); 
}

void setup() {
  Serial.begin(38400);
  Serial.println("Ratio Test:\n");
  pinMode(M_PWM, OUTPUT);
  pinMode(M_DIR1, OUTPUT);
  pinMode(M_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);

  analogWriteFrequency(M_PWM, 375000);
  analogWriteFrequency(M2_PWM, 375000);
  FreqCount.begin(1000);
  FreqMeasure.begin();
}

int M1pos = 0;
int M2pos = 0;
int M1desiredpos = 1000;
int M2desiredpos = 1000;

void loop() {  
  M1desiredpos = myinterp1(&ratio[0], &primary_pos[0], gear_ratio);
  M2desiredpos = myinterp1(&ratio[0], &secondary_pos[0], gear_ratio);
  Serial.println(M1desiredpos);
  Serial.println(M2desiredpos);
  M1pos = enc_a.read();
  M2pos = enc_b.read();
  spinMotor1(PIcontrol1(M1desiredpos, M1pos, 2, 0));
  spinMotor2(PIcontrol2(M2desiredpos, M2pos, 2, 0));
  if (gear_ratio > .75){ 
    gear_ratio += .1;
  }
  else{
    gear_ratio -= .1;
  }
   
 
  //Serial.print("Position: ");
  //Serial.println(i);
  

   // if (FreqMeasure.available()) {
        // average several reading together
   //     sum = sum + FreqMeasure.read();
   //     count = count + 1;
   //     if (count > 30) {
   //       float frequency = FreqMeasure.countToFrequency(sum / count);
   //      Serial.println(frequency);
   // 
   //     }
   //   }


}