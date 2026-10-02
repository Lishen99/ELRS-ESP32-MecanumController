#include "motors.h"

void Motors::begin(const eb::Config& cfg) {
  board_.begin(cfg.motorSdaPin, cfg.motorSclPin, cfg.motorAddr, cfg.motorType, cfg.motorEncPol);
}

void Motors::update(eb::Bridge& bridge) {
  int8_t speeds[eb::kNumMotors];
  if (!bridge.takeMotorCommands(speeds)) return;
  bridge.reportMotorBoard(board_.setSpeeds(speeds));
}

void Motors::stop() {
  const int8_t stop[eb::kNumMotors] = {0, 0, 0, 0};
  board_.setSpeeds(stop);
}
