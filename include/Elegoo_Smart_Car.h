#include <Arduino.h>

// Motor control signals
#define LEFT_ENABLE               5
#define RIGHT_ENABLE              6
#define IN1                       7
#define IN2                       8
#define IN3                       9
#define IN4                       11

#define ERROR_LED                 13
#define ERROR_BLINK_RATE_MS       500

#define SONAR_TRIGGER_PIN         A5
#define SONAR_ECHO_PIN            A4

#define SERVO_PAN_PIN             3
#define SERVO_PAN_MIN_DEG         0
#define SERVO_PAN_MIN_US          450
#define SERVO_PAN_CENTER_DEG      90
#define SERVO_PAN_CENTER_US       1500
#define SERVO_PAN_MAX_DEG         180
#define SERVO_PAN_MAX_US          2500
#define SERVO_PAN_INCREMENT_DEG   5

#define BLUETOOTH_DELAY_SEC       5

//  Speed of the robot in feet/second
#define MAX_SPEED_RATE            2.40
#define SPEED_RATE                2.45

//  Angular rate in degrees/second
#define ANGLE_RATE                333.0

//  Speed adjustments for each side so it goes more straight

//  Minimum and maximum speeds the car is allowed to go
#define MINIMUM_SPEED             100
#define MAXIMUM_SPEED             250

//  Starting speed and adjustment settings
#define LEFT_SPEED                110
#define LEFT_SPEED_ADJUST         0

#define RIGHT_SPEED               110
#define RIGHT_SPEED_ADJUST        0

#define TURN_SIZE_DEG             15
#define RUN_TIME_DIV_MS           5
#define OBJECT_MIN_DIST_IN        3

#define MAX_NUM_MEASUREMENTS      20
#define MAX_ERROR_LOOPS           5

enum DistanceUnits {
  mm = 10,
  cm = 20,
  meters = 30,
  kilometers = 40,

  inches = 50,
  feet = 60,
  miles = 70
};

struct movement {
  bool status;
  uint16_t distance;
};

struct scandata {
  bool status;
  float * meas;
};

//  This is the return value from scan area functions
typedef struct scandata ScanItAll;

//  This is be the return value from movement functions
typedef struct movement Movement;

/*
  Function prototypes
*/
void blink_led(uint8_t , uint16_t);
void disable_motors(void);
void enable_motors(void);
void all_stop(void);
uint16_t pingTime(void);
float measure_distance_in(void);
float measure_distance_cm(void);
bool set_speed(uint8_t, uint8_t);
ScanItAll scan_area(DistanceUnits, uint8_t , uint8_t, uint8_t);
Movement forward(float);
void reverse(float);
void turn_left(uint16_t);
void turn_right(uint16_t);

void calibrate_forward(void);
void calibrate_reverse(void);
void calibrate_right(void);
void calibrate_left(void);

void setup(void);
void loop(void);  
/*
  End of prototypes
*/
