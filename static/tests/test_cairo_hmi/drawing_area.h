/*
 * drawing_area.h
 *
 *  Created on: 13 oct. 2014
 *      Author: ludo6431
 */

#ifndef DRAWING_AREA_H_
#define DRAWING_AREA_H_

#include <gtk/gtk.h>

typedef struct {
    GtkWidget *widget;
    gint widget_width__px, widget_height__px; // (px)

    // world
    double wld_width, wld_height; // (cm)
    double scale; // (px/cm)
    double center_x__cm, center_y__cm; // (cm)
    gboolean center_px_moved;
    double start_x__px, start_y__px; // (px)
    double center_x_incr__px, center_y_incr__px; // (px)
    gboolean mouse_bt1_dragging;
    gboolean center_cm_moved;
    double center_x_incr__cm, center_y_incr__cm; // (cm)

    // mouse interaction
    gboolean mouse_lastpress_moved;
    double mouse_lastpress_x__px, mouse_lastpress_y__px;
    gboolean mouse_moved;
    double mouse_x__px, mouse_y__px; // (px)
    double mouse_x__cm, mouse_y__cm; // (cm)
} sDrawingArea;

// rendering management
void da_invalidate_all(sDrawingArea *da);
void da_prepare_draw(sDrawingArea *da, cairo_t *cr);
gboolean da_need_early_update(sDrawingArea *da);

// point selection
void da_event_click__px(sDrawingArea *da, double x, double y);

// hover
void da_event_hover__px(sDrawingArea *da, double x, double y);

// panning
gboolean da_state_is_panning(sDrawingArea *da);
void da_event_pan_start__px(sDrawingArea *da, double x, double y);
void da_event_pan_update_abs__px(sDrawingArea *da, double x, double y);
void da_event_pan_stop(sDrawingArea *da);

// scaling
void da_event_scale_rel(sDrawingArea *da, double s);

#endif /* DRAWING_AREA_H_ */
