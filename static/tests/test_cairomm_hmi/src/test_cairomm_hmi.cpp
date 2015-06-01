/*
 ============================================================================
 Name        : test_cairomm_hmi.cpp
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in gttkmm
 ============================================================================
 */

#include <business/views/ViewLayersList.h>
#include <glibmm/refptr.h>
#include <gtkmm/application.h>
#include <gtkmm/button.h>
#include <gtkmm/enums.h>
#include <gui/DAPlayground.h>
#include <gui/parts/Perspective.h>
#include <gui/parts/View.h>
#include <gui/WinMain.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace Glib;
using namespace Gtk;
using namespace std;

int main(int argc, char *argv[]) {
    double initialSizeFactor = 3.;
    RefPtr<Application> app = Application::create(argc, argv, "org.gtkmm.example");

    WinMain win;
    DAPlayground& dap = win.daplayground;
    win.set_title("test_cairomm_hmi");
    win.set_position(WIN_POS_CENTER);
    win.set_default_size((initialSizeFactor + 0.5) * dap.world_width__cm(), initialSizeFactor * dap.world_height__cm());

    {
        Perspective p;
        Button b("blah");

        p.add(new ViewLayersList(dap.layers, "layers"));
        p.add(new View("", b, "hello"));

        cout << "all views:" << endl;
        const unordered_multimap<string, View*>& views = p.get_views();
        for_each(views.begin(), views.end(), [] (pair<string, View*> v) {
            std::cout << *v.second << std::endl;
        });

        cout << "layers views:" << endl;
        vector<View*> views_layers(p.get_views_by_id(ViewLayersList::VID));
        for_each(views_layers.begin(), views_layers.end(), [] (View* v) {
            std::cout << *v << std::endl;
        });
    }

// FIXME re-design layers storage for easier update management and easier iteration on group/name
//    dap.layers["outline"] = new LayerOutline();
//    dap.layers["grid"] = new LayerGrid();
//    dap.layers["frame"] = new LayerFrame();
//    dap.layers["clicks"] = new LayerClicks();

    return app->run(win);
}
