/*
 * DrawingArea.cpp
 *
 *  Created on: 25 oct. 2014
 *      Author: ludo6431
 */

#include <cairomm/matrix.h>
#include <gdkmm/device.h>
#include <gdkmm/rectangle.h>
#include <gdkmm/window.h>
#include <glib/gmacros.h>
#include <glib/gmain.h>
#include <gui/DAPlayground.h>
#include <cmath>
#include <cstdio>
#include <utility>

using namespace Cairo;
using namespace Gtk;
using namespace Gdk;
using namespace std;

static bool invalidate(DAPlayground *dap) {
    dap->get_window()->invalidate(true);

    return true;
}

DAPlayground::DAPlayground() :
        DrawingArea() {
    // gtk widget
    widget_width__px = 0;
    widget_height__px = 0;

    // world
    wld_width__cm = 300;
    wld_height__cm = 200;
    scale = 3.; // (px/cm)

    // rendering
    center_x__cm = wld_width__cm / 2.;
    center_y__cm = wld_height__cm / 2.;
    center_px_moved = false;

    // panning
    start_x__px = 0;
    start_y__px = 0;
    center_x_incr__px = 0;
    center_y_incr__px = 0;
    mouse_bt1_dragging = false;
    center_cm_moved = false;
    center_x_incr__cm = 0;
    center_y_incr__cm = 0;

    // mouse interaction
    mouse_lastpress_moved = false;
    user_mouse_lastpress_moved = false;
    mouse_lastpress_x__px = 0;
    mouse_lastpress_y__px = 0;
    mouse_lastpress_x__cm = 0;
    mouse_lastpress_y__cm = 0;
    mouse_moved = false;
    user_mouse_moved = false;
    mouse_x__px = 0;
    mouse_y__px = 0;
    mouse_x__cm = 0;
    mouse_y__cm = 0;

    add_events(BUTTON_PRESS_MASK | BUTTON_RELEASE_MASK | POINTER_MOTION_MASK | SCROLL_MASK | KEY_PRESS_MASK | BUTTON1_MOTION_MASK);

    g_timeout_add(1000 / 25, (GSourceFunc) invalidate, this); // 25 Hz drawing area update rate
}

DAPlayground::~DAPlayground() {

}

// point selection
void DAPlayground::event_click__px(double x, double y) {
    mouse_lastpress_x__px = x;
    mouse_lastpress_y__px = y;
    mouse_lastpress_moved = true;
}
bool DAPlayground::state_click_moved() {
    return user_mouse_lastpress_moved ? (user_mouse_lastpress_moved = false, true) : false;
}

void DAPlayground::state_get_click__cm(double& x, double& y) {
    x = mouse_lastpress_x__cm;
    y = mouse_lastpress_y__cm;
}

// hover
void DAPlayground::event_hover__px(double x, double y) {
    mouse_x__px = x;
    mouse_y__px = y;
    mouse_moved = true;
}
bool DAPlayground::state_hover_moved() {
    return user_mouse_moved ? (user_mouse_moved = false, true) : false;
}
void DAPlayground::state_get_hover__cm(double& x, double& y) {
    x = mouse_x__cm;
    y = mouse_y__cm;
}
void DAPlayground::state_get_hover__cm(float& x, float& y) {
    x = mouse_x__cm;
    y = mouse_y__cm;
}

// panning
bool DAPlayground::state_is_panning() const {
    return mouse_bt1_dragging;
}
void DAPlayground::event_pan_start__px(double x, double y) {
    start_x__px = x;
    start_y__px = y;
    center_x_incr__px = 0;
    center_y_incr__px = 0;
    mouse_bt1_dragging = true;
}
void DAPlayground::event_pan_update_abs__px(double x, double y) {
    center_x_incr__px = x - start_x__px;
    center_y_incr__px = y - start_y__px;
}
void DAPlayground::event_pan_stop() {
    start_x__px = 0;
    start_y__px = 0;
    center_px_moved = true;
    mouse_bt1_dragging = false;
}

// scaling
void DAPlayground::event_scale_rel(double s) {
    if (!center_px_moved && !mouse_moved) {
        scale *= s;

        center_x_incr__cm = (1. - s) / s * (center_x__cm - mouse_x__cm);
        center_y_incr__cm = (1. - s) / s * (center_y__cm - mouse_y__cm);
        center_cm_moved = true;

        // scale changed, update mouse position
//        mouse_moved = true;
    }
    else {
        printf("still performing operation...\n");
    }
}

void DAPlayground::prepare_draw(const RefPtr<Context>& cr) {
    Matrix P0(cr->get_matrix());

    widget_width__px = get_allocated_width();
    widget_height__px = get_allocated_height();

//    static int prevw = -1, prevh = -1;
//    if (widget_width__px != prevw || widget_height__px != prevh) {
//        prevw = widget_width__px;
//        prevh = widget_height__px;
//        printf("w:%i\th:%i\n", widget_width__px, widget_height__px);
//    }

    if (center_cm_moved) {
        center_x__cm += center_x_incr__cm;
        center_y__cm += center_y_incr__cm;
        center_px_moved = false;
        center_cm_moved = false;
    }

    if (mouse_bt1_dragging || center_px_moved) {
        cr->translate(center_x_incr__px, center_y_incr__px);
    }

    cr->translate((double) widget_width__px / 2., (double) widget_height__px / 2.);
    cr->scale(1, -1); // the y is axis is inverted between the image frame and the playground frame
    cr->scale(scale, scale);
    cr->translate(-center_x__cm, -center_y__cm);

//    Matrix m = cr->get_matrix();
//    printf("  %.2f\t%.2f\t%.2f\n  %.2f\t%.2f\t%.2f\n", m.xx, m.xy, m.x0, m.yx, m.yy, m.y0);

    if (center_px_moved) {
        center_x__cm = (double) widget_width__px / 2.;
        center_y__cm = (double) widget_height__px / 2.;
        P0.transform_point(center_x__cm, center_y__cm);
        cr->device_to_user(center_x__cm, center_y__cm);
        center_px_moved = false;
        center_cm_moved = false;
    }

    if (mouse_moved) {
        mouse_x__cm = mouse_x__px;
        mouse_y__cm = mouse_y__px;
//        P0.transform_point(mouse_x__cm, mouse_y__cm);
        da_to_device(mouse_x__cm, mouse_y__cm);
        cr->device_to_user(mouse_x__cm, mouse_y__cm);
        user_mouse_moved = true;
        mouse_moved = false;
    }

    if (mouse_lastpress_moved) {
        mouse_lastpress_x__cm = mouse_lastpress_x__px;
        mouse_lastpress_y__cm = mouse_lastpress_y__px;
//        P0.transform_point(mouse_lastpress_x__cm, mouse_lastpress_y__cm);
        da_to_device(mouse_lastpress_x__cm, mouse_lastpress_y__cm);
        cr->device_to_user(mouse_lastpress_x__cm, mouse_lastpress_y__cm);
        user_mouse_lastpress_moved = true;
        mouse_lastpress_moved = false;
    }
}

bool DAPlayground::on_draw(const RefPtr<Context>& cr) {
    // prepare
    prepare_draw(cr);

    // draw previous click
    {
        if (state_click_moved()) {
            printf("x:%.2f, y:%.2f\n", mouse_lastpress_x__px, mouse_lastpress_y__px);
        }

        cr->arc(mouse_lastpress_x__cm, mouse_lastpress_y__cm, 5, 0, 2 * M_PI);
        cr->stroke();
    }

    cr->set_source_rgb(1, 0, 0);
    cr->arc(mouse_x__cm, mouse_y__cm, 5, 0, 2 * M_PI);
    cr->stroke();

    // iterate through layers and draw them in order of appearance
    for (auto& l : layers) {
        if (!l.second->is_visible()) {
            continue;
        }

        cr->save();
        l.second->on_draw(*this, cr);
        cr->restore();
    }

    return false;
}

bool DAPlayground::on_button_press_event(GdkEventButton* event) {
    if (event->button == 1) {
        event_click__px(event->x, event->y);

        event_pan_start__px(event->x, event->y);
    }

    return true;
}

bool DAPlayground::on_motion_notify_event(GdkEventMotion* event) {
    if (state_is_panning()) {
        event_pan_update_abs__px(event->x, event->y);
    }

    event_hover__px(event->x, event->y);

    printf("x:%lf\ty:%lf\n", event->x, event->y);

    return true;
}

bool DAPlayground::on_button_release_event(GdkEventButton* event) {
    if (event->button == 1) {
        event_pan_update_abs__px(event->x, event->y);
        event_pan_stop();
    }

    return true;
}

bool DAPlayground::on_scroll_event(GdkEventScroll* event) {
    switch (event->direction) {
    case GDK_SCROLL_UP:
        event_scale_rel(1.1);
        break;
    case GDK_SCROLL_DOWN:
        event_scale_rel(1. / 1.1);
        break;
    default:
        break;
    }

    return true;
}
