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
    gint da_width, da_height; // (px)
    GtkWidget *scrolledwindowConsole;
    GtkWidget *console;

    // world
    double wld_width, wld_height; // (cm)
    double scale; // (px/cm)
    double center_x, center_y; // (cm)
    gboolean center_moved;
    double start_x, start_y; // (px)
    double center_x_incr, center_y_incr; // (px)
    gboolean in_movement;

    // mouse interaction
    gboolean mouse_lastpress_moved;
    double mouse_lastpress_x, mouse_lastpress_y;
    gboolean mouse_moved;
    double mouse_x, mouse_y;

    // test
    sGenericStatus i1;
    sGenericStatus i2;
} sContext;



#endif /* CONTEXT_H_ */
