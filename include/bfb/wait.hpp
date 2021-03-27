#pragma once

#include "api.h"
#include <functional>

namespace bfb {
constexpr int general_delay{10};
void wait_until(const std::function<bool()> &condition, const int max_delay);
void wait_until(const std::function<bool()> &condition);
void wait(const int time);
} // namespace bfb