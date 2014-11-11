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
#include <glibmm/signalproxy.h>
#include <gtkmm/enums.h>
#include <gui/parts/ViewLayersList.h>
#include <gui/WinMain.h>
#include <sigc++/connection.h>
#include <sigc++/functors/mem_fun.h>

using namespace Gtk;
using namespace Cairo;
using namespace std;

WinMain::WinMain() :
        m_paned(ORIENTATION_HORIZONTAL), m_button("Hello World") {

    // FIXME re-design layers storage for easier update management and easier iteration on group/name
    daplayground.layers["outline"] = new LayerOutline();
    daplayground.layers["grid"] = new LayerGrid();
    daplayground.layers["frame"] = new LayerFrame();
    daplayground.layers["clicks"] = new LayerClicks();

    add(m_paned);
    {
        m_paned.pack1(daplayground, true, false);
        {
            daplayground.set_hexpand(true);
            daplayground.set_vexpand(true);
        }
        daplayground.show();

        m_paned.pack2(f, false, false);
        {
            f.add(new ViewLayersList(daplayground.layers, "layers"));
            f.add(new View(m_button, "hello"));
            m_button.signal_clicked().connect(sigc::mem_fun(*this, &WinMain::on_button_clicked));
            m_button.show();
        }
        f.show();
    }
    m_paned.show();
}
