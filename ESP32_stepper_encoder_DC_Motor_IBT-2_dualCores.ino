
/*run a DC motor with IBT-2 driver and control loop with quadrature encoder*/

//good link to use DC motors drivers : https://dronebotworkshop.com/dc-motor-drivers/


const int pinA = 13;  // Broche A de l'encodeur
const int pinB = 15;  // Broche B de l'encodeur

double targetRot;
double gearRatio = 170. / 24.  ;         //nb teeth of the big and motor wheels
int topsPerTurn = 1024 * 4     ;   //encoder tops per shaft turn (*4 because FULLQUAD)

long rawValue, prevRawValue ;
unsigned long encoderTimer = 0;


#include "FastInterruptEncoder.h"

//Encoder enc(pinA, pinB, SINGLE /* or HALFQUAD or FULLQUAD */, 250 /* Noise and Debounce Filter (default 0) */); //
Encoder enc(pinA, pinB, FULLQUAD, 100); // - Example for ESP32



/* create a hardware timer */
hw_timer_t * timer = NULL;

void IRAM_ATTR Update_IT_callback()
{
  enc.loop();
}


//stepper control


const int pinStep = 17;   // Steps input pin
const int pinDir = 16;     // Dir input pin

void IRAM_ATTR stepper_IT_callback()
{
  if (digitalRead(pinDir) == HIGH) targetRot += 5.12001; //= 1024/200 ==> gives a 200 steps/turn stepper !
  else targetRot -= 5.12001;
}



/*
  IBT-2 Motor Control Board driven by Arduino.
  Connection to the IBT-2 board:
  IBT-2 pins  7 (VCC) to ESP32 3.3V
  IBT-2 pin 8 (GND) to GND
  IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/
#define RPWM_PIN  19  // PWM output pin ; connect to IBT-2 pin 1 (RPWM)
#define LPWM_PIN  23   // PWM output pin ; connect to IBT-2 pin 2 (LPWM)
#define ENABLE_PIN_R 18 // IBT-2 Enable pins R ; connect to IBT-2 pin 3 
#define ENABLE_PIN_L 5 // IBT-2 Enable pins L ; connect to IBT-2 pin 4 

int pwmSpeed ;    //PWM duty cycle to command the DC motor speed
long timeOut;     //used to change setpoint 10s after motor stopped


//long tt ;
//long ttt;
//int minimum = 1000000;
//int maximum = -100000;

//PID
#include <PID_v1.h>   //https://github.com/br3ttb/Arduino-PID-Library
double setpoint, input, output; //used by PID lib
double filteredOutput = 0.;
double prevOutput;
//setpoint= nb Rotation of the motor shaft (expressed in encoder nb pulses),
//input = current rotation (expressed in encoder nb pulses),
//output is pwmSpeed of the motor

//Specify the links and initial tuning parameters
double Kp = .0001, Ki = .08, Kd = 0.;                         //will be modified into the loop
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);  //declare the PID

#define DEADBAND 250  //deadband where the motor stalls


//multitasking
TaskHandle_t Task1;

#define CLOCK_PERIOD 500

void core0Task1( void * parameter )   //this task will run for ever on core 0
{
  for (;;)  //put here code that could otherwise slow core1 loop...
  {
    delay(CLOCK_PERIOD);
    //  Serial.print("pos pwm\t");
    Serial.print(rawValue);
    Serial.print("\t");
    Serial.println(pwmSpeed);
    //    minimum = 1000000;
    //    maximum = -100000;
  }
}




void setup()
{
  Serial.begin(115200);
  //stepper control
  pinMode(pinStep, INPUT_PULLUP);
  pinMode(pinDir, INPUT_PULLUP);
  attachInterrupt(pinStep, stepper_IT_callback, FALLING);  //call interrupt any time pinStep is rising

  //IBT-2 DC motor driver
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(ENABLE_PIN_L, OUTPUT);
  pinMode(ENABLE_PIN_R, OUTPUT);

  ledcAttachPin(RPWM_PIN, 0); // assign PWM pins to channels
  ledcAttachPin(LPWM_PIN, 1); // assign PWM pins to channels
  // Initialize channels : ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 24000, 11); // 24 kHz PWM, 11-bit resolution (range 0-2047)
  ledcSetup(1, 24000, 11);
  //start motor driver
  pwmSpeed = 0;           //motor stopped at startup (range -2047 to 2047)
  timeOut = millis();

  digitalWrite(ENABLE_PIN_L, HIGH);   //enable the half bridges
  digitalWrite(ENABLE_PIN_R, HIGH);
  //digitalWrite(ENABLE_PIN_L, LOW);  //disable the half bridges ==> motor fully stopped
  //digitalWrite(ENABLE_PIN_R, LOW);

  //turn the PID on
  setpoint =  targetRot;              //setpoint is the number of turns on the motor shaft (not the wiper shaft)
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-2047, 2047); //output of the PID is the pwm value. coded on 11 bits and forward/reverse
  myPID.SetSampleTime(40);            //compute the PID every 40µs (see timing into loop). Needs modified version of PID lib (micros instead of millis)
  Kp = .45;                           // start with a PID controller with very soft Kd
  Ki = 0.01;
  Kd = 0.0002;
  myPID.SetTunings(Kp, Ki, Kd);
// settings corrects :
//  Kp = .45;                           // start with a PI controller
//  Ki = 0.01;
//  Kd = 0.0002;
//fluidnc :
// x:
//    steps_per_mm: 5666
//    max_rate_mm_per_min: 500
//    acceleration_mm_per_sec2: 25
//    max_travel_mm: 1000
//    motor0:
//      limit_all_pin: NO_PIN
//      limit_neg_pin: NO_PIN
//      limit_pos_pin: NO_PIN
//      hard_limits: false
//      pulloff_mm: 1
//      standard_stepper:
//        step_pin: gpio.14:pu
//        direction_pin: gpio.27:pu
//        disable_pin: NO_PIN
  //encoder

  if (enc.init()) Serial.println("Encoder Init OK");
  else
  {
    Serial.println("Encoder Init Failed");
    while (1);
  }

  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &Update_IT_callback, true);
  /* Set alarm to call onTimer function every 100 ms -> 100 Hz */
  timerAlarmWrite(timer, 10000, true);
  /* Start an alarm */
  timerAlarmEnable(timer);
  enc.resetTicks();           // reset the tick counter (cumulative nb of pulses = 0)
  rawValue = enc.getTicks();  //read the encoder (cumulative nb of pulses)
  prevRawValue = rawValue;
  encoderTimer = millis();


  //multitask
  xTaskCreatePinnedToCore     // this task can handle slow operations without blocking the main loop
  (
    core0Task1,               /* Task function. */
    "Task_1",                 /* name of task. */
    5000,                     /* Stack size of task */
    NULL,                     /* parameter of the task */
    1,                        /* priority of the task */
    &Task1,                   /* Task handle to keep track of created task */
    0
  );                       /* Core */


}


void loop()   //keep the main loop as fast as possible to avoid jitter. This task runs on core1
{
  //loop lasts between 13 and 35µs when motor driven at 55 kHz pulses
  //PID_Compute lasts 25µs max. So PID SetSampleTime(40) (40µs after mod of the library)
  //  tt = micros();
  //encoder
  rawValue = enc.getTicks();

  //PID stuff
  input = rawValue;
  setpoint =  targetRot;
  myPID.Compute();

  if (abs(output) < DEADBAND)
  {
    double delta = output - prevOutput;
    prevOutput = output;
    if (delta > 0)  output = DEADBAND + delta;
    else            output = - DEADBAND + delta;
  }
 
  //DC motor driver
  pwmSpeed = output;  //use output of the PID to drive the motor
  runMotor();         //will apply direction and pwm value to the driver

  //  ttt = micros();
  //  if ((ttt - tt) > maximum) maximum = ttt - tt;
  //  if ((ttt - tt) < minimum) minimum = ttt - tt;
}

void runMotor(void)
{
  if (pwmSpeed > 0)
  {
    ledcWrite(0, pwmSpeed);
    ledcWrite(1, 0);
  }
  else
  {
    ledcWrite(0, 0);
    ledcWrite(1, -pwmSpeed);
  }
}
