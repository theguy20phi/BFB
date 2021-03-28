#include "gui.hpp"

void remote_gui_task_fn() {
  master.clear();
  for (;;) {
    if (!pros::competition::is_disabled()) {
    } else {
      master.print(0, 0, "%s                ", match->getRoutine().getInfo().name);
      bfb::wait(50);
      master.print(1, 0, "%s                ", match->getRoutine().getInfo().description);
      bfb::wait(50);
      master.print(2, 0, "%s                ", match->getRoutine().getInfo().setup);
    }
    bfb::wait(50);
  }
}

void doom_screen() {
  lv_obj_t *mainScreen = lv_obj_create(NULL, NULL);
  lv_obj_t *image = lv_img_create(mainScreen, NULL);
  lv_img_set_src(image, &DOOM);
  lv_scr_load(mainScreen);
}