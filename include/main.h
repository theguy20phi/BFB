#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_LITERALS
#define PROS_USE_SIMPLE_NAMES
#include "api.h"
#include "okapi/api.hpp"
#include "bfb/bfb.hpp"

#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif // _PROS_MAIN_H_
