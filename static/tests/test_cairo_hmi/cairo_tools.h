/*
 * cairo_tools.h
 *
 *  Created on: 8 oct. 2014
 *      Author: ludo6431
 */

#ifndef CAIRO_TOOLS_H_
#define CAIRO_TOOLS_H_

#include "context.h"

gboolean invalidate_all(sContext *ctx);
void cairo_ellipse(cairo_t *cr, double xc, double yc, double sq_a, double sq_b, double rot);

#endif /* CAIRO_TOOLS_H_ */
