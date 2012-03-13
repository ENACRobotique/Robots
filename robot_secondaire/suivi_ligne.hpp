#ifndef _SUIVI_LIGNE_H
#define _SUIVI_LIGNE_H

#include "main.hpp"
extern "C" {
#include "pos_estimate.h"
}

void init_line();
void deinit_line();

#ifdef DEBUG
extern unsigned int _pos_c;
#endif

#endif

