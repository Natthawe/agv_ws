#include <AccelStepper.h>
#define MOTOR_L_PWM_GPIO 30
#define MOTOR_R_PWM_GPIO 35
#define MOTOR_L_DIR_GPIO 29
#define MOTOR_R_DIR_GPIO 36

AccelStepper stepperL(AccelStepper::DRIVER, MOTOR_L_PWM_GPIO, MOTOR_L_DIR_GPIO);
AccelStepper stepperR(AccelStepper::DRIVER, MOTOR_R_PWM_GPIO, MOTOR_R_DIR_GPIO);
AccelStepper *stepper[] = {&stepperL, &stepperR};

IntervalTimer Interval;
IntervalTimer Interval_connect;

String incoming = "";
float cmd_speed[2] = {0, 0};
long pos[] = {0, 0};
int arrSize = *(&pos + 1) - pos;

void setup() {
  Serial.begin(9600);
  Interval.begin(loop_1ms, 2500);//us
  Interval_connect.begin(checker_interval, 6000);//us
  for (int i = 0; i < arrSize; i++) {
    stepper[i]->setMaxSpeed(60000.0);
    stepper[i]->setSpeed(0.0);
  }
}

int counting_checker = 0;
void checker_interval() {
  if (counting_checker > 50) {
    for (int i = 0; i < arrSize; i++) {
      cmd_speed[i] = 0.00;
    }
  }
}

int task_100ms = 0;
void loop_1ms() {
  if (task_100ms++ >= 10) {
    task_100ms = 0;
    for (int i = 0; i < arrSize - 1; i++) {
      Serial.print(pos[i]);
      Serial.print(",");
    }
    Serial.println(pos[arrSize - 1]);
    counting_checker++;
  }
  for (int i = 0; i < arrSize; i++) {
    stepper[i]->setSpeed(cmd_speed[i]);
    pos[i] = stepper[i]->currentPosition();
  }
}

int count_pack = 0;
void loop() {
  for (int i = 0; i < arrSize; i++) {
    stepper[i]->runSpeed();
  }
  
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 10 || c == 13)
    {
      cmd_speed[count_pack]=incoming.toFloat();
      counting_checker = int();
      incoming = String();
      count_pack = int();
    } else if (c == ',') {
      cmd_speed[count_pack] = incoming.toFloat();
      incoming = String();
      count_pack++;
    }
    else incoming += c;
  }
}
