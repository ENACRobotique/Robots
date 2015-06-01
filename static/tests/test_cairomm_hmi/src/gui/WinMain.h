#ifndef GTKMM_EXAMPLE_HELLOWORLD_H
#define GTKMM_EXAMPLE_HELLOWORLD_H

#include <business/layers/Layer.h>
#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <gtkmm/box.h>
#include <gtkmm/button.h>
#include <gtkmm/paned.h>
#include <gtkmm/window.h>
#include <gui/DAPlayground.h>
#include <gui/parts/Folder.h>
#include <iostream>

class WinMain: public Gtk::Window {

public:
    WinMain();
    virtual ~WinMain() {
    }

    DAPlayground daplayground;

protected:
    //Signal handlers:
    void on_button_clicked() {
        std::cout << "Hello World" << std::endl;

        daplayground.layers["grid"]->toggle_active();
    }

    //Member widgets:
    Gtk::Paned _paned;
    Gtk::Box _box;
    Gtk::Button _button;

    Folder _f;
};

#endif // GTKMM_EXAMPLE_HELLOWORLD_H
