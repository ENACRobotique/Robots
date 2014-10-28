/*
 * LayerGrid.cpp
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#include <business/layers/LayerPlaygroundGrid.h>
#include <gdkmm/window.h>
#include <glib/gmacros.h>
#include <gui/DAPlayground.h>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
using namespace Gtk;
using namespace Cairo;
;

void LayerGrid::on_draw(DAPlayground& dap, const RefPtr<Context>& cr) {
    double x = 1.2, y = 1.2;
    int i;
    cr->device_to_user_distance(x, y);
    cr->set_line_width(MAX(x, y));
    cr->set_dash((vector<double> ) { 3. / 2., 2. / 2. }, 0);
    cr->set_source_rgba(0, 0, 0, 0.3);

    double incr = 10.;

    for (i = 1; i < ceil(dap.world_width__cm() / incr); i++) {
        cr->move_to((double) i * incr, 0);
        cr->line_to((double) i * incr, dap.world_height__cm());
    }

    for (i = 1; i < ceil(dap.world_height__cm() / incr); i++) {
        cr->move_to(0, (double) i * incr);
        cr->line_to(dap.world_width__cm(), (double) i * incr);
    }

    cr->stroke();
    cr->set_dash((vector<double> ) { }, 0);
}
