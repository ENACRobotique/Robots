/*
 * context.h
 *
 *  Created on: 8 oct. 2014
 *      Author: ludo6431
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include <gtk/gtk.h>

typedef struct {
    // gui
    GtkWidget *window;
    GtkWidget *drawing_area;
    gint da_width, da_height;

    // world
    double wld_width, wld_height;

    // mouse interaction
    gboolean pressed;
    double x, y;
} sContext;



#endif /* CONTEXT_H_ */
