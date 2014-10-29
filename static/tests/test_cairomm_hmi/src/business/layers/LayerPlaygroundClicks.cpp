/*
 * LayerFrame.cpp
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#include <business/layers/LayerPlaygroundClicks.h>
#include <gui/DAPlayground.h>
#include <cmath>
#include <cstdio>

using namespace Cairo;
using namespace std;

bool LayerClicks::on_event(DAPlayground& dap, const GdkEvent& event) {
    switch (event.type) {
    case GDK_BUTTON_PRESS:
        if (event.button.button == 1) {
            event_click__px(event.button.x, event.button.y);
        }
        break;
    default:
        break;
    }

    return true; // handled (less prior layers won't receive this event
}

void LayerClicks::on_draw(DAPlayground& dap, const RefPtr<Context>& cr) {
    if (mouse_lastpress_moved) {
        mouse_lastpress_x__cm = mouse_lastpress_x__px;
        mouse_lastpress_y__cm = mouse_lastpress_y__px;
        dap.da_to_device(mouse_lastpress_x__cm, mouse_lastpress_y__cm);
        cr->device_to_user(mouse_lastpress_x__cm, mouse_lastpress_y__cm);
        user_mouse_lastpress_moved = true;
        mouse_lastpress_moved = false;
    }

    if (state_click_moved()) {
        printf("x:%.2f, y:%.2f\n", mouse_lastpress_x__px, mouse_lastpress_y__px);
    }

    cr->set_source_rgb(0, 0, 0);
    cr->arc(mouse_lastpress_x__cm, mouse_lastpress_y__cm, 5, 0, 2 * M_PI);
    cr->stroke();
}
