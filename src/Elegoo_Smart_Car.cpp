#include <Arduino.h>
#include <Servo.h>

#include "Elegoo_Smart_Car.h"

//  Trackers for the current speed of each side
uint8_t left_speed_current = 110;
uint8_t right_speed_current = 110;
uint8_t current_speed = 110;
int speed_increment_ms = 30;

//  Helper function to make using the struct easier
Movement movement(bool status, uint16_t dist){
  Movement move;

  move.status = status;
  move.distance = dist;

  return move;
}

//  Create the pan servo
Servo servo_pan;

//  Array to hold distance readings

void blink_led(uint8_t led_pin, uint16_t rate_ms) {
  digitalWrite(led_pin, HIGH);
  delay(rate_ms);
  digitalWrite(led_pin, LOW);
  delay(rate_ms);
}

void disable_motors(void) {
  digitalWrite(LEFT_ENABLE, LOW);
  digitalWrite(RIGHT_ENABLE, LOW);
}

void enable_motors(void) {
  digitalWrite(LEFT_ENABLE, HIGH);
  digitalWrite(RIGHT_ENABLE, HIGH);
}

void all_stop(void) {
  disable_motors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

uint16_t pingTime(void) {
  uint16_t pingTravelTime;

  digitalWrite(SONAR_TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SONAR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIGGER_PIN, LOW);

  pingTravelTime = pulseIn(SONAR_ECHO_PIN, HIGH);

  Serial.print(", pingTravelTime = ");
  Serial.print(pingTravelTime);

  return pingTravelTime;
}

float measure_distance_in(void) {
  uint16_t pingTravelTime;
  float distance_in;

  pingTravelTime = pingTime();
  distance_in = ((pingTravelTime * 761.0 * 5280.0 * 12.0) / (1000000.0 * 3600.0)) / 2;

  Serial.print(", distance_in = ");
  Serial.print(distance_in);

  return distance_in;
}

float measure_distance_cm(void) {
  float distance_cm = measure_distance_in() * 2.54;

  return distance_cm;
}

bool set_speed(uint8_t left_speed, uint8_t right_speed) {
  bool success = true;
  uint8_t count = 0;
  enable_motors();

  if (left_speed < MINIMUM_SPEED or left_speed + LEFT_SPEED_ADJUST > MAXIMUM_SPEED or
       right_speed < MINIMUM_SPEED or right_speed + RIGHT_SPEED_ADJUST > MAXIMUM_SPEED) {
    //  Either the left speed or right speed, or both, are out of range

    while(count < MAX_ERROR_LOOPS) {
      blink_led(ERROR_LED, ERROR_BLINK_RATE_MS);
      delay(750);
      count += 1;
    }

    success = false;
  } else {
    //  Set the speeds
    analogWrite(LEFT_ENABLE, left_speed + LEFT_SPEED_ADJUST);
    left_speed_current = left_speed;
    analogWrite(RIGHT_ENABLE, right_speed + RIGHT_SPEED_ADJUST);
    right_speed_current = right_speed;
  }

  return success;
}

ScanItAll scan_area(DistanceUnits units, uint8_t start_deg, uint8_t end_deg, uint8_t incr_deg=45) {
  static float measurements[MAX_NUM_MEASUREMENTS];
  ScanItAll result;
  bool success = true;
  float current_reading = 0.0;
  uint8_t current_pos = start_deg;
  uint8_t reading_count = 0;

  while (success and (current_pos <= end_deg) and (reading_count < MAX_NUM_MEASUREMENTS)) {
    servo_pan.write(current_pos);

    //  Debug only!
    Serial.print("reading_count = ");
    Serial.print(reading_count);
    Serial.print(", current_pos = ");
    Serial.print(current_pos);

    //  Take a distance reading
    switch (units) {
      case cm:
        current_reading = measure_distance_cm();
        measurements[reading_count] = current_reading;
        break;

      case mm:
        current_reading = measure_distance_cm();
        measurements[reading_count] = current_reading * 10.0;
        break;

      case feet:
        current_reading = measure_distance_in();
        measurements[reading_count] = current_reading / 12.0;
        break;

      case inches:
        current_reading = measure_distance_in();
        measurements[reading_count] = current_reading;
        break;

      //  Invalid units name or units are not implemented
      default:
        success = false;
        break;
    };

    Serial.print(", current_reading = ");
    Serial.print(current_reading);
    Serial.print(", measurements[reading_count] = ");
    Serial.println(measurements[reading_count]);

    current_pos += incr_deg;
    reading_count += 1;
  }

  result.status = success;
  result.meas = measurements;

  return result;
}

/*
  Move forward and check distance from objects. Distance has to
    be checked while the robot is running, so the run time will
    be split into segments.

  Param: distance_ft - distance to travel in feet

  Returns: struct movement
    bool    success   true = robot completed the run
                      false = robot stopped before completing the run
    uint8_t distance  distance traveled in inches
*/

Movement forward(float distance_ft) {
  Movement move;
  ScanItAll scandata;

  float run_time_ms;
  uint8_t distance_in;
  uint8_t increment_counter = 0;
  uint8_t num_increments;
  uint8_t increment_ms;

  move.status = true;
  enable_motors();

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  run_time_ms = distance_ft / SPEED_RATE * 1000;
  increment_ms = int8_t(run_time_ms / RUN_TIME_DIV_MS);
  num_increments = int8_t(run_time_ms / increment_ms);

  Serial.print("Distance calculations:\nrun_time_ms ");
  Serial.print(run_time_ms);
  Serial.print(", increment_ms = ");
  Serial.print(increment_ms);
  Serial.print(", num_increments = ");
  Serial.println(num_increments);

  while (move.status and (increment_counter < num_increments))  {
    distance_in = measure_distance_in();
    Serial.print("distance_in = ");
    Serial.println(distance_in);

    delay(increment_ms);
    increment_counter += 1;

    if (distance_in >= OBJECT_MIN_DIST_IN) {
      Serial.println("Object is too close - stopping");
      scandata = scan_area(inches, 0, 180, 45);

      //  Add the behavior to execute if the robot is too close to something
      all_stop();
    }

    if (not move.status) {
      Serial.println("Area scan is not valid!");
    }

  }

  move.status = scandata.status;
  move.distance = distance_in;

  return move;
}

void reverse(float distance) {
  float run_time_ms;

  enable_motors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(5000);

  run_time_ms = distance / SPEED_RATE * 1000;

  delay(run_time_ms);
}

void turn_left(uint16_t turn_degrees) {
  float run_time_ms;

  enable_motors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(5000);
  //run_time_ms = turn_degrees / 345.0 * 1000.0;

  run_time_ms = turn_degrees / ANGLE_RATE * 1000.0;

  delay(run_time_ms);

  all_stop();
}

void turn_right(uint16_t turn_degrees) {
  float run_time_ms;

  enable_motors();

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(5000);

  //run_time_ms = turn_degrees / 345.0 * 1000.0;
  run_time_ms = turn_degrees / ANGLE_RATE * 1000.0;

  delay(run_time_ms);

  all_stop();
}

/*
  These functions MAY be removed for release
*/
void calibrate_forward(void) {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(5000);
  all_stop();
}

void calibrate_reverse(void) {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(5000);
  all_stop();
}

void calibrate_right(void) {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(5000);
  all_stop();
}

void calibrate_left(void) {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(5000);
  all_stop();
}

void setup(void) {
  Serial.begin(115200);

  // Setup all the pins
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //  Allow Bluetooth time to connect
  Serial.println("Waiting for Bluetooth to connect");
  for (uint8_t i = 0; i < BLUETOOTH_DELAY_SEC * 2; i++) {
    blink_led(ERROR_LED, 500);
  }

  //  Enable the motors
  Serial.println("Enabling motors.");
  enable_motors();

  //  Make sure the motors are stopped
  Serial.println("Stopping all motors");
  all_stop();

  //  Set initial motor speeds
  Serial.println("Setting initial motor speeds");
  set_speed(LEFT_SPEED + LEFT_SPEED_ADJUST, RIGHT_SPEED + RIGHT_SPEED_ADJUST);

  servo_pan.attach(SERVO_PAN_PIN, SERVO_PAN_MIN_US, SERVO_PAN_MAX_US);
  servo_pan.write(SERVO_PAN_CENTER_DEG);

  //  Testing code for my Smart Car routines

  uint8_t index = 0;

  ScanItAll result;

  result = scan_area(inches, 0, 180, 45);

  Serial.print("scan area results: ");
  Serial.println(result.status ? "Successful" : "Not Successful");
  Serial.println();
  Serial.println();
  Serial.print("Data: ");

  for (index = 0; index < MAX_NUM_MEASUREMENTS; index++) {
      Serial.print(result.meas[index]);
      Serial.print(", ");
  }

  Serial.println();
}

void loop(void) {
  char command;
  Movement move;
  // int pos;

  blink_led(ERROR_LED, 1000);

  if (Serial.available()) {
    //  Handle commands from remote
    command = Serial.read();

    //  Convert lower to upper case for commands
    if (command >= 97 and command <= 122) {
      command -= 32;
    }

    switch(command) {
      case 'F':
        forward(0.25);
        break;

      case 'B':
        reverse(0.25);
        break;

      case 'R':
        turn_right(TURN_SIZE_DEG);
        break;

      case 'L':
        turn_left(TURN_SIZE_DEG);
        break;
    }
  } else {
    // Move forward 1 foot

/*
    move = forward(1.0);

    if (move.status) {
      Serial.println("Robot completed the run");
    } else {
      Serial.println("Robot did not complete the run");
    }


    for (pos = 0; pos <= SERVO_PAN_MAX_DEG; pos += SERVO_PAN_INCREMENT_DEG) { // goes from 0 degrees to 180 degrees
      // in steps of SERVO_PAN_INCREMENT_DEG degrees
      Serial.print("Position = ");
      Serial.println(pos);
      servo_pan.write(pos);              // tell servo to go to position in variable>
      delay(50);                       // waits 15ms for the servo to reach the po>
   }

   delay(1000);
   servo_pan.write(90);
   delay(2000);

   for (pos = SERVO_PAN_MAX_DEG; pos >= 0; pos -= SERVO_PAN_INCREMENT_DEG) { // goes from 180 degrees to 0 degrees
     Serial.print("");
      Serial.println(pos);
     servo_pan.write(pos);              // tell servo to go to position in variable>
     delay(50);                       // waits 15ms for the servo to reach the po>
   }

   delay(1000);
*/

//forward(5.0);
//      delay(2000);
//      reverse(5.0);
//     delay(2000);
/*
    //  Do autonomous stuff
    set_speed(MINIMUM_SPEED, MINIMUM_SPEED);

    while(true) {
      forward(5.0);

      delay(2000);

      reverse(5.0);

      delay(2000);

      //  Change speed ramping direction if neccessary
      if (increment >= 0) {
        if (current_speed <= 250) {
          ;
        } else if (current_speed >= 250) {
          increment = -30;
        }
      } else {
        if (current_speed >= 110) {
          ;
        } else if (current_speed <= 110) {
            increment = 30;
        }
      }

      left_speed_current += increment;
      right_speed_current += increment;
      current_speed += increment;
      set_speed(left_speed_current, right_speed_current);
    }
  }
*/
  delay(1000);

  }
}
