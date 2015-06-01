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
#include <iostream>

class View {
private:
    const std::string& _vid;
    Gtk::Widget& _widget;
    Gtk::Widget* const _lbl;

public:
    View(const std::string& vid, Gtk::Widget& widget, const Glib::ustring& label) :
            _vid(vid), _widget(widget), _lbl(new Gtk::Label(label)) {
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

    const std::string& id() const {
        return _vid;
    }

    virtual Gtk::Widget& child() const {
        return _widget;
    }

    friend std::ostream& operator<<(std::ostream& os, const View& v);
};

#endif /* GUI_PARTS_VIEW_H_ */
