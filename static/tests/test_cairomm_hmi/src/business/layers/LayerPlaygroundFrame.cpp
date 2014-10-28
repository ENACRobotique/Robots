/*
 * LayerFrame.cpp
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#include <business/layers/LayerPlaygroundFrame.h>
#include <algorithm>

using namespace Cairo;
using namespace std;

void LayerFrame::on_draw(DAPlayground& dap, const RefPtr<Context>& cr) {
    constexpr double length = 10; // (cm)
    double x = 10, y = 10; // (px)
    cr->device_to_user_distance(x, y);
    cr->set_line_width(min(max(x, y), length / 3.)); // set line width, not thicker than a third of its length

    // draw x axis
    cr->set_source_rgb(1, 0, 0);
    cr->move_to(0, 0);
    cr->line_to(length, 0);
    cr->stroke();

    // draw y axis
    cr->set_source_rgb(0, 1, 0);
    cr->move_to(0, 0);
    cr->line_to(0, length);
    cr->stroke();
}
