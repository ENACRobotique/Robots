/*
 ============================================================================
 Name        : test_cairomm_hmi.cpp
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in gtkmm
 ============================================================================
 */

#include "MonitoringHMI.h"

#include <glibmm/signalproxy.h>
#include <gtkmm/enums.h>
#include <sigc++/connection.h>
#include <sigc++/functors/mem_fun.h>
#include <iostream>

using namespace Gtk;
using namespace Cairo;
using namespace std;

MonitoringHMI::MonitoringHMI() :
        m_paned(ORIENTATION_VERTICAL), m_button("Hello World") {
    m_button.signal_clicked().connect(sigc::mem_fun(*this, &MonitoringHMI::on_button_clicked));
    m_paned.add1(m_button);
    m_button.show();

    m_paned.add2(m_da);
    m_da.show();

    add(m_paned);
    m_paned.show();
}

MonitoringHMI::~MonitoringHMI() {
}

DAPlayground& MonitoringHMI::daplayground() {
    return m_da;
}

void MonitoringHMI::on_button_clicked() {
    cout << "Hello World" << endl;
}
