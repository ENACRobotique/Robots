/*
 * View.h
 *
 *  Created on: 5 nov. 2014
 *      Author: ludo6431
 */

#ifndef GUI_PARTS_VIEW_H_
#define GUI_PARTS_VIEW_H_

#include <glibmm/ustring.h>
#include <gtkmm/label.h>

class View {
private:
    Gtk::Widget& child;
    Gtk::Widget* const lbl;

public:
    View(Gtk::Widget& child, const Glib::ustring& label) :
            child(child), lbl(new Gtk::Label(label)) {
        lbl->show();
    }

    Gtk::Widget& widget() const {
        return child;
    }

    Gtk::Widget& label() const {
        return *lbl;
    }
};

#endif /* GUI_PARTS_VIEW_H_ */
