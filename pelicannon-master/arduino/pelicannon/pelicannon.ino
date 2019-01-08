#include <Stepper.h>

#define ENABLE_STEPPER_PITCH 0

#define STEPPER_ANGLE(steps, stepsRot) 2*PI*((steps % stepsRot) / stepsRot)
#define LEAST_ANGLE_DIFF(x, y) atan2(sin(x-y), cos(x-y))
#define ROTATIONS_PER_MINUTE(av) abs(av) * 60.0 * 1.0/(2.0*PI)

struct State {
#if ENABLE_STEPPER_PITCH
  float speedPitch;
  float goalPitch;
#endif
  float speedYaw;
  float goalYaw;
  bool spin;
  bool fire;
};

#define RECV_SPIN(msg) (msg.flags & (1 << 0))
#define RECV_FIRE(msg) (msg.flags & (1 << 1))
#define RECV_SPEED(msg) (msg.flags & (1 << 2))

#pragma pack(1)
struct RecvMessage {
  unsigned char header[4];
#if ENABLE_STEPPER_PITCH
  float goalPitch;
#endif
  float goalYaw;
  unsigned char flags;
};
#pragma pack()

struct SendMessage {
  unsigned char header[4];
#if ENABLE_STEPPER_PITCH
  float pitch;
#endif
  float yaw;
};

#define PIN_FIRE 1
#define PIN_SPIN 2

#define STEPS_YAW 200
Stepper stepperYaw(STEPS_YAW, 8, 9, 10, 11);
int currentStepsYaw;

#if ENABLE_STEPPER_PITCH
#define STEPS_PITCH 200
#error "TODO: Configure stepperPitch pins"
Stepper stepperPitch(STEPS_PITCH, 8, 9, 10, 11);
int currentStepsPitch;
#endif

bool validMessageRecieved;
State state;

void setup() {
  validMessageRecieved = false;

#if ENABLE_STEPPER_PITCH
  currentStepsPitch = 0;
#endif
  currentStepsYaw = 0;

  allStop();

  Serial.begin(9600);
}

void allStop() {
  state.spin = false;
  state.fire = false;
  digitalWrite(PIN_SPIN, LOW);
  digitalWrite(PIN_FIRE, LOW);
    
#if ENABLE_STEPPER_PITCH
  state.speedPitch = 0.0;
  stepperPitch.setSpeed(0.0);
#endif

  state.speedYaw = 0.0;
  stepperYaw.setSpeed(0.0);

}

void loop() {

  bool sendUpdate = false;

  if (validMessageRecieved) {

      /* Move towards goal */
#if ENABLE_STEPPER_PITCH
      double thetaPitch = STEPPER_ANGLE(currentStepsPitch, STEPS_PITCH);
      double deltaPitch = LEAST_ANGLE_DIFF(state.goalPitch, thetaPitch);
      if ( abs(deltaPitch)  <= 2 * PI / 100 )
        state.speedPitch = 0.0;
      else{
        if(deltaPitch > 0){
          state.speedPitch = state.speedPitch;
        }else if(deltaPitch < 0){
          state.speedPitch = -state.speedPitch;
        }else{
          state.speedPitch = 0.0;
        }
      }
#endif

      double thetaYaw = STEPPER_ANGLE(currentStepsYaw, STEPS_YAW);
      double deltaYaw = LEAST_ANGLE_DIFF(state.goalYaw, thetaYaw);
      if ( abs(deltaYaw) <= 2 * PI / 100 )
        state.speedYaw = 0.0;
      else{
        if(deltaYaw > 0){
          state.speedYaw = state.speedYaw;
        }else if(deltaYaw < 0){
          state.speedYaw = -state.speedYaw;
        }else{
          state.speedYaw = 0.0;
        }
      }

    /* Step the motors */
#if ENABLE_STEPPER_PITCH
    stepperPitch.setSpeed(ROTATIONS_PER_MINUTE(state.speedPitch));

    if (state.speedPitch > 0.0) {
      stepperPitch.step(1);
      currentStepsPitch++;
      sendUpdate = true;
    } else if (state.speedPitch < 0.0) {
      stepperPitch.step(-1);
      currentStepsPitch--;
      sendUpdate = true;
    } else {
      stepperPitch.setSpeed(0.0);
    }
#endif

    stepperYaw.setSpeed(ROTATIONS_PER_MINUTE(state.speedYaw));

    if (state.speedYaw > 0.0) {
      stepperYaw.step(1);
      currentStepsYaw++;
      sendUpdate = true;
    } else if (state.speedYaw < 0.0) {
      stepperYaw.step(-1);
      currentStepsYaw--;
      sendUpdate = true;
    } else {
      stepperYaw.setSpeed(0.0);
    }

    /* Drive relays */
    if (state.fire)
      digitalWrite(PIN_FIRE, HIGH);
    else
      digitalWrite(PIN_FIRE, LOW);

    if (state.spin)
      digitalWrite(PIN_SPIN, HIGH);
    else
      digitalWrite(PIN_SPIN, LOW);

  } else {
    /* Invalid message, stop everything */
    allStop();
  }

  if (sendUpdate)
    sendPosition();

}


void sendPosition() {
  SendMessage message;

  message.header[0] = 0xDE;
  message.header[1] = 0xAD;
  message.header[2] = 0xBE;
  message.header[3] = 0xEF;

  message.yaw = STEPPER_ANGLE(currentStepsYaw, STEPS_YAW);

#if ENABLE_STEPPER_PITCH
  message.pitch = STEPPER_ANGLE(currentStepsPitch, STEPS_PITCH);
#endif

  Serial.write((char*)&message, sizeof(SendMessage));
}


void serialEvent() {

  RecvMessage message;

  while (Serial.available()) {
    Serial.readBytes((char*)&message, sizeof(RecvMessage));

    if (message.header[0] == 0xDE &&
        message.header[1] == 0xAD &&
        message.header[2] == 0xBE &&
        message.header[3] == 0xEF) {

      /* When speed flag is set the goal field is used to set the speed */
      if (RECV_SPEED(message)) {
#if ENABLE_STEPPER_PITCH
        state.speedPitch = message.goalPitch;
#endif
        state.speedYaw = message.goalYaw;
      } else {
#if ENABLE_STEPPER_PITCH
        state.goalPitch = message.goalPitch;
#endif
        state.goalYaw = message.goalYaw;
      }

      if (RECV_SPIN(message)) {
        state.spin = true;
      } else {
        state.spin = false;
      }

      if (RECV_FIRE(message)) {
        state.fire = true;
      } else {
        state.fire = false;
      }

      validMessageRecieved = true;
    }

  }
}
