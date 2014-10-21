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
    GdkRectangle rect = { .x = 0, .y = 0, .width = ctx->da.widget_width__px, .height = ctx->da.widget_height__px };

    gdk_window_invalidate_rect(gtk_widget_get_window(ctx->da.widget), &rect, 1);

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

void cairo_text(cairo_t *cr, double x, double y, double size, const char *txt) {
    // draw some moving text
    cairo_save(cr);
    cairo_translate(cr, x, y);
    cairo_scale(cr, 1, -1);
    cairo_move_to(cr, 0, 0);
    cairo_select_font_face(cr, "Sans", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    cairo_set_font_size(cr, size);
    cairo_show_text(cr, txt);
    cairo_restore(cr);
}
