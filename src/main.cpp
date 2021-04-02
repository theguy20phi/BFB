#include "main.h"

std::unique_ptr<bfb::Chassis> chassis;
std::unique_ptr<bfb::Rollers> rollers;
pros::Controller remote(pros::E_CONTROLLER_MASTER);
bfb::Routine routine = bfb::get_routine(3275); // 3275, so that deploy is default.
pros::ADIPotentiometer auton_selector{{bfb::port::port_extender, bfb::port::auton_selector_pot}};

void remote_gui_task_fn() {
  remote.clear();
  for (;;) {
    if (!pros::competition::is_disabled()) {
      remote.print(0, 0, "%f                ", chassis->get_pose().x.convert(okapi::inch));
      bfb::wait(50);
      remote.print(1, 0, "%f                ", chassis->get_pose().y.convert(okapi::inch));
      bfb::wait(50);
      remote.print(2, 0, "%f                ", chassis->get_pose().h.convert(okapi::degree));
    } else {
      remote.print(0, 0, "%s                ", bfb::get_routine_name(routine));
      bfb::wait(50);
      remote.print(1, 0, "                  ");
      bfb::wait(50);
      remote.print(2, 0, "                  ");
    }
    bfb::wait(50);
  }
}

void initialize() {
  chassis = std::make_unique<bfb::Chassis>();
  rollers = std::make_unique<bfb::Rollers>();
  bfb::doom_screen();
  remote.clear();
  pros::Task remote_gui_task(remote_gui_task_fn);
}

void disabled() {
  for (;;) {
    routine = bfb::get_routine(auton_selector.get_value());
    bfb::wait(bfb::general_delay);
  }
}

void competition_initialize() {
  for (;;) {
    routine = bfb::get_routine(auton_selector.get_value());
    bfb::wait(bfb::general_delay);
  }
}

void autonomous() {
  switch (routine) {
  case bfb::Routine::Row:
    chassis->set_pose({});
    rollers->shoot_and_outtake();
    bfb::wait(500);
    rollers->intake();
    chassis->move_to({{1.5_tile, 1.5_tile, 135.0_deg}, {0.9_tile, 0.9_tile, 135.0_deg}});
    break;
  case bfb::Routine::Left:
    break;
  case bfb::Routine::Right:
    break;
  case bfb::Routine::Skills:
    chassis->set_pose({2.0_tile, 9.0_in, -90.0_deg});
    rollers->shoot_and_outtake();
    chassis->drive_voltage(-12000, 0, 0);
    bfb::wait(1500);
    rollers->intake();
    chassis->move_to({{2.15_tile, 0.35_tile, -45.0_deg},
                      {1.3_tile, 1.2_tile, -45.0_deg},
                      {1.0_tile, 1.0_tile, -135.0_deg}},
                     false,
                     3.0_s);
    rollers->standby();
    chassis->move_to({{0.0_tile, 0.0_tile, -135.0_deg}}, true, 3.0_s);
    rollers->shoot_until_empty();
    rollers->intake_two_balls();
    chassis->move_to({{1.0_tile, 1.0_tile, -180.0_deg}}, false, 2.0_s);
    rollers->outtake_until_empty();
    rollers->intake();
    chassis->move_to({{1.0_tile, 1.5_tile, 0.0_deg},
                      {1.0_tile, 3.0_tile, 0.0_deg},
                      {1.5_tile, 3.0_tile, -90.0_deg}},
                     false,
                     2.0_s);
    rollers->standby();
    chassis->move_to({{0.0_tile, 3.0_tile, -90.0_deg}}, true);
    rollers->shoot_until_empty();
    rollers->intake_one_ball();
    chassis->move_to({{1.5_tile, 3.0_tile, -180.0_deg}}, false, 2.0_s);
    rollers->outtake_until_empty();
    rollers->intake();
    chassis->move_to({{1.5_tile, 4.0_tile, 0.0_deg},
                      {1.5_tile, 5.5_tile, 0.0_deg},
                      {1.0_tile, 5.0_tile, -45.0_deg}},
                     false,
                     3.25_s);
    rollers->standby();
    chassis->move_to({{0.0_tile, 6.0_tile, -45.0_deg}}, true, 2.0_s);
    rollers->shoot_until_empty();
    rollers->intake_two_balls();
    chassis->move_to({{1.5_tile, 4.0_tile, -45.0_deg}}, false, 2.5_s);
    rollers->outtake_until_empty();
    rollers->intake();
    chassis->move_to({{1.5_tile, 4.0_tile, 90.0_deg}, {3.0_tile, 4.0_tile, 90.0_deg}}, false, 2.0_s);
    rollers->standby();
    chassis->move_to({{3.0_tile, 4.0_tile, 0.0_deg}, {3.0_tile, 6.0_tile, 0.0_deg}}, true, 2.0_s);
    rollers->shoot_until_empty();
    rollers->intake_one_ball();
    chassis->move_to({{3.0_tile, 4.0_tile, 0.0_deg}}, false, 2.0_s);
    rollers->outtake_until_empty();
    break;
  default:
    chassis->set_pose({});
    chassis->move_to({{0.0_in, 3.0_tile, 180.0_deg}, {}});
    break;
  };
}

void opcontrol() {
  for (;;) {
    chassis->drive_voltage(remote.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 100,
                           remote.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * 100,
                           remote.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 100);
    if (remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
      chassis->toggle_hold();

    if (remote.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
      rollers->toggle_shooter();
    rollers->slow_rollers(remote.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
    if (remote.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      if (remote.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        rollers->cycle();
      else if (remote.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        rollers->shoot_and_outtake();
      else
        rollers->shoot();
    } else {
      if (remote.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        rollers->intake();
      else if (remote.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        rollers->outtake();
      else
        rollers->standby();
    }

    bfb::wait(10);
  }
}
