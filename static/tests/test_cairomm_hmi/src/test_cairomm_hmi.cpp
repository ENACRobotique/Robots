/*
 ============================================================================
 Name        : test_cairomm_hmi.cpp
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in gttkmm
 ============================================================================
 */

#include <business/layers/LayerPlaygroundClicks.h>
#include <business/layers/LayerPlaygroundFrame.h>
#include <business/layers/LayerPlaygroundGrid.h>
#include <business/layers/LayerPlaygroundOutline.h>
#include <glibmm/refptr.h>
#include <gtkmm/application.h>
#include <gtkmm/enums.h>
#include <gui/DAPlayground.h>
#include <gui/WinMain.h>

using namespace Glib;
using namespace Gtk;

int main(int argc, char *argv[]) {
    double initialSizeFactor = 3.;
    RefPtr<Application> app = Application::create(argc, argv, "org.gtkmm.example");

    WinMain win;
    DAPlayground& dap = win.daplayground;
    win.set_title("test_cairomm_hmi");
    win.set_position(WIN_POS_CENTER);
    win.set_default_size((initialSizeFactor + 0.5) * dap.world_width__cm(), initialSizeFactor * dap.world_height__cm());

    dap.layers["outline"] = new LayerOutline();
    dap.layers["grid"] = new LayerGrid();
    dap.layers["frame"] = new LayerFrame();
    dap.layers["clicks"] = new LayerClicks();

    return app->run(win);
}
