/*
 * drawing_area.c
 *
 *  Created on: 13 oct. 2014
 *      Author: ludo6431
 */

#include "drawing_area.h"

void da_invalidate_all(sDrawingArea *da) {
    GdkRectangle rect = { .x = 0, .y = 0, .width = da->widget_width__px, .height = da->widget_height__px };

    gdk_window_invalidate_rect(gtk_widget_get_window(da->widget), &rect, TRUE);
}

void da_prepare_draw(sDrawingArea *da, cairo_t *cr) {
    cairo_matrix_t P0;
    cairo_get_matrix(cr, &P0);

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
        cairo_matrix_transform_point(&P0, &da->center_x__cm, &da->center_y__cm);
        cairo_device_to_user(cr, &da->center_x__cm, &da->center_y__cm);
        da->center_px_moved = FALSE;
        da->center_cm_moved = FALSE;
    }

    if (da->mouse_moved) {
        da->mouse_x__cm = da->mouse_x__px;
        da->mouse_y__cm = da->mouse_y__px;
        cairo_matrix_transform_point(&P0, &da->mouse_x__cm, &da->mouse_y__cm);
        cairo_device_to_user(cr, &da->mouse_x__cm, &da->mouse_y__cm);
        da->user_mouse_moved = TRUE;
        da->mouse_moved = FALSE;
    }

    if (da->mouse_lastpress_moved) {
        da->mouse_lastpress_x__cm = da->mouse_lastpress_x__px;
        da->mouse_lastpress_y__cm = da->mouse_lastpress_y__px;
        cairo_matrix_transform_point(&P0, &da->mouse_lastpress_x__cm, &da->mouse_lastpress_y__cm);
        cairo_device_to_user(cr, &da->mouse_lastpress_x__cm, &da->mouse_lastpress_y__cm);
        da->user_mouse_lastpress_moved = TRUE;
        da->mouse_lastpress_moved = FALSE;
    }
}

gboolean da_need_early_update(sDrawingArea *da) {
    return da->mouse_lastpress_moved || da->mouse_moved || da->mouse_bt1_dragging || da->center_px_moved || da->center_cm_moved;
}

void da_event_click__px(sDrawingArea *da, double x, double y) {
    da->mouse_lastpress_x__px = x;
    da->mouse_lastpress_y__px = y;
    da->mouse_lastpress_moved = TRUE;
}

gboolean da_state_click_moved(sDrawingArea *da) {
    return da->user_mouse_lastpress_moved ? (da->user_mouse_lastpress_moved = FALSE, TRUE) : FALSE;
}

void da_state_get_click__cm(sDrawingArea *da, double *x, double *y) {
    *x = da->mouse_lastpress_x__cm;
    *y = da->mouse_lastpress_y__cm;
}

void da_event_hover__px(sDrawingArea *da, double x, double y) {
    da->mouse_x__px = x;
    da->mouse_y__px = y;
    da->mouse_moved = TRUE;
}

gboolean da_state_hover_moved(sDrawingArea *da) {
    return da->user_mouse_moved ? (da->user_mouse_moved = FALSE, TRUE) : FALSE;
}

void da_state_get_hover__cm(sDrawingArea *da, double *x, double *y) {
    *x = da->mouse_x__cm;
    *y = da->mouse_y__cm;
}

void da_state_get_hover__cm__float(sDrawingArea *da, float *x, float *y) {
    *x = da->mouse_x__cm;
    *y = da->mouse_y__cm;
}

gboolean da_state_is_panning(sDrawingArea *da) {
    return da->mouse_bt1_dragging;
}

void da_event_pan_start__px(sDrawingArea *da, double x, double y) {
    da->start_x__px = x;
    da->start_y__px = y;
    da->center_x_incr__px = 0;
    da->center_y_incr__px = 0;
    da->mouse_bt1_dragging = TRUE;
}

void da_event_pan_update_abs__px(sDrawingArea *da, double x, double y) {
    da->center_x_incr__px = x - da->start_x__px;
    da->center_y_incr__px = y - da->start_y__px;
}

void da_event_pan_stop(sDrawingArea *da) {
    da->start_x__px = 0;
    da->start_y__px = 0;
    da->center_px_moved = TRUE;
    da->mouse_bt1_dragging = FALSE;
}

void da_event_scale_rel(sDrawingArea *da, double s) {
    if (!da->center_px_moved && !da->mouse_moved) {
        da->scale *= s;

        da->center_x_incr__cm = (1. - s) / s * (da->center_x__cm - da->mouse_x__cm);
        da->center_y_incr__cm = (1. - s) / s * (da->center_y__cm - da->mouse_y__cm);
        da->center_cm_moved = TRUE;

        // scale changed, update mouse position
//        da->mouse_moved = TRUE;
    }
    else {
        printf("still performing operation...\n");
    }
}
