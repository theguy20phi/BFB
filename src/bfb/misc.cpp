#include "misc.hpp"

namespace bfb {
Routine get_routine(int pot_reading) {
  if (pot_reading < 819)
    return Routine::Row;
  if (pot_reading < 1638)
    return Routine::Left;
  if (pot_reading < 2457)
    return Routine::Right;
  if (pot_reading < 3276)
    return Routine::Deploy;
  return Routine::Skills;
}

std::string get_routine_name(const Routine &routine) {
  switch (routine) {
  case Routine::Row:
    return "Full Row";
  case Routine::Left:
    return "Left Side";
  case Routine::Right:
    return "Right Side";
  case Routine::Deploy:
    return "Deploy";
  case Routine::Skills:
    return "Skills";
  default:
    return "Error";
  };
}

void doom_screen() {
  bfb::wait(500); // Give some time for the computer to collect its bearings.
  lv_obj_t *mainScreen = lv_obj_create(NULL, NULL);
  lv_obj_t *image = lv_img_create(mainScreen, NULL);
  lv_img_set_src(image, &DOOM);
  lv_scr_load(mainScreen);
}
} // namespace bfb