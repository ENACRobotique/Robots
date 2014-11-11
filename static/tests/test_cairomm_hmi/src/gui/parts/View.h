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
    Gtk::Widget& _widget;
    Gtk::Widget* const _lbl;

public:
    View(Gtk::Widget& widget, const Glib::ustring& label) :
            _widget(widget), _lbl(new Gtk::Label(label)) {
        _lbl->show();
    }

    virtual ~View() {
    }

    Gtk::Widget& widget() const {
        return _widget;
    }

    Gtk::Widget& label() const {
        return *_lbl;
    }

    virtual Gtk::Widget& child() const {
        return _widget;
    }
};

#endif /* GUI_PARTS_VIEW_H_ */
