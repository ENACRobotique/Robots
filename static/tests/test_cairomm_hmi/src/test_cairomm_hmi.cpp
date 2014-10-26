/*
 ============================================================================
 Name        : test_cairomm_hmi.cpp
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in gttkmm
 ============================================================================
 */

#include <glibmm/refptr.h>
#include <gtkmm/application.h>
#include <gtkmm/enums.h>

#include "DAPlayground.h"
#include "MonitoringHMI.h"

using namespace Glib;
using namespace Gtk;

int main(int argc, char *argv[]) {
    double initialSizeFactor = 3.;
    RefPtr<Application> app = Application::create(argc, argv, "org.gtkmm.example");

    MonitoringHMI win;
    DAPlayground& dap = win.daplayground();
    win.set_title("test_cairomm_hmi");
    win.set_position(WIN_POS_CENTER);
    win.set_default_size(initialSizeFactor * dap.world_width__cm(), (initialSizeFactor + 0.5) * dap.world_height__cm());

    return app->run(win);
}
