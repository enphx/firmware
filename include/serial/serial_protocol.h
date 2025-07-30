

typedef enum {
  MSG_START = 0x21, // Every actual message should start with this; looks like '!'.

  FLOAT_AHEAD, // These signify
  UINT_AHEAD,
  INT_AHEAD,

  SET, // These are the basic commands.
  GET,

  PID,

  ARM, // These are the things that you can SET and GET stuff for.
    TTBL,
    SHOULDER,
    ELBOW,
    CLAW,
  ENCODER_MOTOR,
  DRIVE_BASE,
  LIDAR,
  MAGNETOMETER,
  IR_BEACON,
  TAPE_SENSOR,
  ODOMETRY,

  ANGLE, // These are values relating to the above that one would want to read.
  VELOCITY,
  PID_ERROR,   // PID values to get and set.
  PID_SETPOINT,
  PID_ACCUMULATOR,
  PID_KP,
  PID_KI,
  PID_KD,
  PID_OUTPUT,
  ALL,

  NONE,

  // These are command modifiers.
  RAW,
  CONVERTED, // 
  LEFT,
  RIGHT, 
  
  MSG_TYPE_COUNTER,

  MSG_END = 0x0A, // And every message should end with this ('\n')
} MessageCode;

#define MESSAGE_COUNT (MSG_TYPE_COUNTER - MSG_START)
