/*
 * context.h
 *
 *  Created on: 8 oct. 2014
 *      Author: ludo6431
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include <gtk/gtk.h>
#include "messages-statuses.h"

typedef struct {
    // gui
    GtkWidget *window;
    GtkWidget *drawing_area;
    gint da_width, da_height;

    // world
    double wld_width, wld_height;

    // mouse interaction
    gboolean pressed;
    double press_x, press_y;
    gboolean moved;
    double move_x, move_y;

    // test
    sGenericStatus i1;
    sGenericStatus i2;
} sContext;



#endif /* CONTEXT_H_ */
