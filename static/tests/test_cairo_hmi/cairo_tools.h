/*
 * cairo_tools.h
 *
 *  Created on: 8 oct. 2014
 *      Author: ludo6431
 */

#ifndef CAIRO_TOOLS_H_
#define CAIRO_TOOLS_H_

#include "context.h"

void cairo_ellipse(cairo_t *cr, double xc, double yc, double a, double b, double rot);
void cairo_text(cairo_t *cr, double x, double y, double size, const char *txt);

#endif /* CAIRO_TOOLS_H_ */
