/*
 ============================================================================
 Name        : test_cairomm_hmi.cpp
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in gtkmm
 ============================================================================
 */

#include <business/layers/LayerPlaygroundClicks.h>
#include <business/layers/LayerPlaygroundFrame.h>
#include <business/layers/LayerPlaygroundGrid.h>
#include <business/layers/LayerPlaygroundOutline.h>
#include <business/views/ViewLayersList.h>
#include <glibmm/signalproxy.h>
#include <gtkmm/enums.h>
#include <gui/WinMain.h>
#include <sigc++/connection.h>
#include <sigc++/functors/mem_fun.h>

using namespace Gtk;
using namespace Cairo;
using namespace std;

WinMain::WinMain() :
        _paned(ORIENTATION_HORIZONTAL), _button("Hello World") {

    // FIXME re-design layers storage for easier update management and easier iteration on group/name
    daplayground.layers["outline"] = new LayerOutline();
    daplayground.layers["grid"] = new LayerGrid();
    daplayground.layers["frame"] = new LayerFrame();
    daplayground.layers["clicks"] = new LayerClicks();

    add(_paned);
    {
        _paned.pack1(daplayground, true, false);
        {
            daplayground.set_hexpand(true);
            daplayground.set_vexpand(true);
        }
        daplayground.show();

        _paned.pack2(_f, false, false);
        {
            _f.add(new ViewLayersList(daplayground.layers, "layers"));
            _f.add(new View("", _button, "hello"));
            _button.signal_clicked().connect(sigc::mem_fun(*this, &WinMain::on_button_clicked));
            _button.show();
        }
        _f.show();
    }
    _paned.show();
}
