/*servo motor driver board control
 * https://srituhobby.com
 */
 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver srituhobby = Adafruit_PWMServoDriver();

#define servoMIN 150
#define servoMAX 900

byte servo = 0;

void setup() {
  Serial.begin(9600);
  srituhobby.begin();
  srituhobby.setPWMFreq(60);
}

void loop() {

  for (int pulse = servoMIN; pulse < servoMAX; pulse++) {
    srituhobby.setPWM(servo, 0, pulse);
    Serial.println(servo);
  }
  delay(10);

  for (int pulse = servoMAX; pulse > servoMIN; pulse--) {
    srituhobby.setPWM(servo, 0, pulse);
    Serial.println(servo);
  }
  delay(10);

  servo++;
  if (servo > 1) {
    servo = 0;
  }
}

//What is the min and max servo value that I can give to this code?

//#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
//
//// Initialize PCA9685
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//
//// Define PWM pulse range for the ESC
//#define PWM_MIN 150  // 1 ms pulse width (minimum throttle)
//#define PWM_MAX 400  // 2 ms pulse width (maximum throttle)
//#define PWM_NEUTRAL 175 // 1.5 ms pulse width (neutral)
//
//#define MOTOR_CHANNEL 2 // Connect motor ESC signal wire to PCA9685 channel 0
//
//void setup() {
//  Serial.begin(9600);
//  pwm.begin();
//  
//  // Set PWM frequency to 50 Hz
//  pwm.setPWMFreq(60);
//  
//  // Set ESC to neutral position
//  pwm.setPWM(MOTOR_CHANNEL, 0, PWM_NEUTRAL);
//  delay(2000); // Give the ESC time to initialize
//  Serial.println("ESC initialized to neutral.");
//}
//
//void loop() {
//  // Gradually increase motor speed
//  for (int pulse = PWM_NEUTRAL; pulse <= PWM_MAX; pulse += 10) {
//    pwm.setPWM(MOTOR_CHANNEL, 0, pulse);
//    Serial.print("Throttle: ");
//    Serial.println(pulse);
//    delay(10); // Adjust delay to control ramp-up speed
//  }
//
//  // Hold maximum throttle for 2 seconds
//  delay(2000);
//
//  // Gradually decrease motor speed
//  for (int pulse = PWM_MAX; pulse >= PWM_NEUTRAL; pulse -= 10) {
//    pwm.setPWM(MOTOR_CHANNEL, 0, pulse);
//    Serial.print("Throttle: ");
//    Serial.println(pulse);
//    delay(10); // Adjust delay to control ramp-down speed
//  }
//
//  // Hold neutral position for 2 seconds
//  delay(2000);
//}
