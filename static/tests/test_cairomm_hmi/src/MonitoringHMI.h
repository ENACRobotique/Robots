#ifndef GTKMM_EXAMPLE_HELLOWORLD_H
#define GTKMM_EXAMPLE_HELLOWORLD_H

#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <gtkmm/button.h>
#include <gtkmm/paned.h>
#include <gtkmm/window.h>

#include "DAPlayground.h"

class MonitoringHMI: public Gtk::Window {

public:
    MonitoringHMI();
    virtual ~MonitoringHMI();

    DAPlayground& daplayground();

protected:
    //Signal handlers:
    void on_button_clicked();
    bool on_da_draw(const Cairo::RefPtr<Cairo::Context>& cr);

    //Member widgets:
    Gtk::Paned m_paned;
    Gtk::Button m_button;
    DAPlayground m_da;
};

#endif // GTKMM_EXAMPLE_HELLOWORLD_H
