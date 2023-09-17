
#define IRSENS0 A0 //DO pin of the 1st sensor from left
#define IRSENS1 A1 
#define IRSENS2 A2
#define IRSENS3 A3
#define IRSENS4 A4
#define IRSENS5 A5

#define MOTOR_PWM_LEFT  5
#define MOTOR_PWM_RIGHT 6

#define MOTOR_INA_R 7
#define MOTOR_INB_R 8
#define MOTOR_INA_L 9
#define MOTOR_INB_L 10

#define DESIREDOUTPUT 2.5

#define MOTOR_BASE_POW 50

int sensor_pin[6] = {
  IRSENS0,
  IRSENS1,
  IRSENS2,
  IRSENS3,
  IRSENS4,
  IRSENS5
};

bool sensor_data[6] = { 0,0,0,0,0,0 };

void ir_sens_logic_read(void){
  for(int i = 0; i < 6; i++)
    sensor_data[i] = digitalRead(sensor_pin[i]);
}

double black_line_centrode(void){
  double sum  = 0;
  int    cnt0 = 0;
  for(int i = 0; i < 6; i++){
    if ( 1 == sensor_data[i]){
      sum += i;
      cnt0++;
    }
  }
  if(cnt0 == 0)
    return 2.5;
  return sum / cnt0;
}


double kp = 45;
double ki = 0.1;
double kd = 12;

double error   = 0;
double error_prev1 = 0;
double error_prev2 = 0;
double error_i = 0;
double error_d = 0;


double pid_compute(double error){
  error_d = (3 * error - 4 * error_prev1 + error_prev2)/2;
  //error_d = error - error_prev1;
  error_i = error_i + error;
  
  double pid_output = kp * error + ki * error_i + kd * error_d;

  error_prev2 = error_prev1;
  error_prev1 = error;

  return pid_output;
}

void drive(int motor_pow_l, int motor_pow_r){
  if (motor_pow_r > 0){
    digitalWrite(MOTOR_INA_R, 1);
    digitalWrite(MOTOR_INB_R, 0);
  }
  else if (motor_pow_r < 0){
    digitalWrite(MOTOR_INA_R, 0);
    digitalWrite(MOTOR_INB_R, 1);
  }
  if (motor_pow_r == 0){
    digitalWrite(MOTOR_INA_R, 0);
    digitalWrite(MOTOR_INB_R, 0);
  }

  if (motor_pow_l > 0){
    digitalWrite(MOTOR_INA_L, 1);
    digitalWrite(MOTOR_INB_L, 0);
  }
  else if (motor_pow_l < 0){
    digitalWrite(MOTOR_INA_L, 0);
    digitalWrite(MOTOR_INB_L, 1);
  }
  if (motor_pow_l == 0){
    digitalWrite(MOTOR_INA_L, 0);
    digitalWrite(MOTOR_INB_L, 0);
  }

  if(abs(motor_pow_l) > 100)
    motor_pow_l = 100;
  if(abs(motor_pow_r) > 100)
    motor_pow_r = 100;
  analogWrite(MOTOR_PWM_LEFT , map(abs(motor_pow_l),0,100,100,255));
  analogWrite(MOTOR_PWM_RIGHT, map(abs(motor_pow_r),0,100,100,255));
}


void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 6; i++)
    pinMode(sensor_pin[i], INPUT);
  pinMode(MOTOR_INA_R, OUTPUT); 
  pinMode(MOTOR_INB_R, OUTPUT);
  pinMode(MOTOR_INA_L, OUTPUT);
  pinMode(MOTOR_INB_L, OUTPUT);
  pinMode(MOTOR_PWM_LEFT ,OUTPUT);
  pinMode(MOTOR_PWM_RIGHT ,OUTPUT);
  Serial.begin(9600);

  analogWrite(MOTOR_PWM_LEFT, 255);
  analogWrite(MOTOR_PWM_RIGHT, 255);
  delay(100);

}

double input;
double output;

double motor_pow_l;
double motor_pow_r;

void loop() {
  ////// put your main code here, to run repeatedly:

  ir_sens_logic_read();
  input = black_line_centrode();
  
  error = DESIREDOUTPUT - input;
  output = pid_compute(error);
  
  motor_pow_l = MOTOR_BASE_POW - output;
  motor_pow_r = MOTOR_BASE_POW + output;
  drive(motor_pow_l, motor_pow_r);
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(",");
  Serial.print("ControlSignal:");
  Serial.println(output);
  delay(100);
  
  
}
