/* -------------------------------------------------------------------------- */
/*                                   Set Up                                   */
/* -------------------------------------------------------------------------- */
#include "main.h"

void initialize() {
  rollers = std::make_unique<RollersMachine>(Rollers::Standby{});
  chassis = std::make_unique<ChassisMachine>(Chassis::Standby{});
  /*match = std::make_unique<Match>(std::array<Routine, 12>{redLeft,
                                                          redMidLeft,
                                                          redMidRight,
                                                          redRight,
                                                          redRow,
                                                          skills,
                                                          blueLeft,
                                                          blueMidLeft,
                                                          blueMidRight,
                                                          blueRight,
                                                          blueRow,
                                                          none});*/
  pros::Task controllerGUITask{controllerGUITaskFn};
  rollers->start();
  chassis->start();
  doomScreen();
}

void disabled() {
  rollers->setState(Rollers::Standby{});
  chassis->setState(Chassis::Standby{});
  for (;;) {
    match->update();
    bfb::wait(bfb::Wait::generalDelay);
  }
}

void competition_initialize() {
  for (;;) {
    match->update();
    bfb::wait(bfb::Wait::generalDelay);
  }
}

/* -------------------------------------------------------------------------- */
/*                                 Autonomous                                 */
/* -------------------------------------------------------------------------- */
void autonomous() {
  chassis->reset();
  match->getRoutine().execute();
}

/* -------------------------------------------------------------------------- */
/*                                   Driver                                   */
/* -------------------------------------------------------------------------- */
void opcontrol() {
  chassis->reset();
  for (;;) {
    chassisControls();
    rollerControls();
    contingencies();
    bfb::wait(10);
  }
}
