/*
 * cairo_tools.c
 *
 *  Created on: 8 oct. 2014
 *      Author: ludo6431
 */

#include <context.h>
#include <gtk/gtk.h>
#include <math.h>

#include "cairo_tools.h"

gboolean invalidate_all(sContext *ctx) {
    // for now, invalidate all
    GdkRectangle rect = { .x = 0, .y = 0, .width = ctx->da_width, .height = ctx->da_height };

    gdk_window_invalidate_rect(gtk_widget_get_window(ctx->drawing_area), &rect, 1);

    return TRUE; // continue to be called
}

void cairo_ellipse(cairo_t *cr, double xc, double yc, double a, double b, double rot) {
    cairo_save(cr);
    cairo_translate(cr, xc, yc);
    cairo_rotate(cr, rot);
    cairo_scale(cr, a, b);
    cairo_arc(cr, 0., 0., 1., 0., 2 * M_PI);
    cairo_restore(cr);
}
