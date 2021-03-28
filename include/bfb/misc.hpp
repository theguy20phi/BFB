#pragma once

#include "geometry.hpp"
#include "pros/apix.h"
#include "values.hpp"
#include "wait.hpp"

LV_IMG_DECLARE(DOOM)

namespace bfb {
enum class Routine { Row, Left, Right, Deploy, Skills };
Routine get_routine(int pot_reading);
std::string get_routine_name(const Routine &routine);
void doom_screen();
} // namespace bfb