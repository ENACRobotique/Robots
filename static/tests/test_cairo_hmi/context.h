/*
 * context.h
 *
 *  Created on: 8 oct. 2014
 *      Author: ludo6431
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include <drawing_area.h>
#include <gtk/gtk.h>
#include <messages-statuses.h>

typedef struct {
    // gui
    GtkWidget *window;
    sDrawingArea da;
    GtkWidget *console;
    GtkWidget *notebook;

    // test
    sGenericStatus i1;
    sGenericStatus i2;
} sContext;

#endif /* CONTEXT_H_ */
