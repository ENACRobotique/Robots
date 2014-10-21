/*
 * drawing_area.c
 *
 *  Created on: 13 oct. 2014
 *      Author: ludo6431
 */

#include "drawing_area.h"

void da_prepare_draw(sDrawingArea *da, cairo_t *cr) {
    da->widget_width__px = gtk_widget_get_allocated_width(da->widget);
    da->widget_height__px = gtk_widget_get_allocated_height(da->widget);

    if (da->center_cm_moved) {
        da->center_x__cm += da->center_x_incr__cm;
        da->center_y__cm += da->center_y_incr__cm;
        da->center_px_moved = FALSE;
        da->center_cm_moved = FALSE;
    }

    if (da->mouse_bt1_dragging || da->center_px_moved) {
        cairo_translate(cr, da->center_x_incr__px, da->center_y_incr__px);
    }

    cairo_translate(cr, (double) da->widget_width__px / 2., (double) da->widget_height__px / 2.);
    cairo_scale(cr, 1, -1); // the y is axis is inverted between the image frame and the playground frame
    cairo_scale(cr, da->scale, da->scale);
    cairo_translate(cr, -da->center_x__cm, -da->center_y__cm);

//        cairo_matrix_t m;
//        cairo_get_matrix(cr, &m);
//        printf("%.2f\t%.2f\t%.2f\n%.2f\t%.2f\t%.2f\n", m.xx, m.xy, m.x0, m.yx, m.yy, m.y0);

    if (da->center_px_moved) {
        da->center_x__cm = (double) da->widget_width__px / 2.;
        da->center_y__cm = (double) da->widget_height__px / 2.;
        cairo_device_to_user(cr, &da->center_x__cm, &da->center_y__cm);
        da->center_px_moved = FALSE;
        da->center_cm_moved = FALSE;
    }
}
