/*
 ============================================================================
 Name        : test_cairomm_hmi.cpp
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in gtkmm
 ============================================================================
 */

#include "WinMain.h"

#include <glibmm/signalproxy.h>
#include <gtkmm/enums.h>
#include <sigc++/connection.h>
#include <sigc++/functors/mem_fun.h>
#include <iostream>

using namespace Gtk;
using namespace Cairo;
using namespace std;

WinMain::WinMain() :
        m_paned(ORIENTATION_VERTICAL), m_button("Hello World") {
//    m_button.signal_clicked().connect(sigc::mem_fun(*this, &WinMain::on_button_clicked));
//    m_paned.add1(m_button);
//    m_button.show();
//
//    m_paned.add2(daplayground);
//    daplayground.show();
//
//    add(m_paned);
//    m_paned.show();
    m_button.signal_clicked().connect(sigc::mem_fun(*this, &WinMain::on_button_clicked));
    m_box.add(m_button);
    m_button.show();

    m_box.add(daplayground);
    daplayground.set_hexpand(true);
    daplayground.set_vexpand(true);
    daplayground.show();

    add(m_box);
    m_box.show();
}
