// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "configuration.h"
#include "planner.h"
#include "stepper.h"
#include "language.h"
#include "communication.h"
#include "motion_control.h"
#include "connect_bsp.h"

#define	PSTR(s) ((const char*)(s))
#define false		0
#define true		1
	
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define square(x) x*x
	
#define CRITICAL_SECTION_START  __disable_irq();
#define CRITICAL_SECTION_END    __enable_irq();

#endif
