/*
 * LayerFrame.cpp
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#include <business/layers/LayerPlaygroundOutline.h>
#include <glib/gmacros.h>
#include <gui/DAPlayground.h>

using namespace Cairo;
using namespace std;

void LayerOutline::on_draw(DAPlayground& dap, const RefPtr<Context>& cr) {
    double x = 1.5, y = 1.5; // (px)
    cr->device_to_user_distance(x, y);
    cr->set_line_width(MAX(x, y));
    cr->set_source_rgba(0, 0, 0, 0.5);
    cr->rectangle(0., 0., dap.world_width__cm(), dap.world_height__cm());
    cr->stroke();
}
