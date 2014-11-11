/*
 * View.h
 *
 *  Created on: 5 nov. 2014
 *      Author: ludo6431
 */

#ifndef GUI_PARTS_SCROLLEDVIEW_H_
#define GUI_PARTS_SCROLLEDVIEW_H_

#include <glibmm/ustring.h>
#include <gtkmm/enums.h>
#include <gtkmm/scrolledwindow.h>
#include <gui/parts/View.h>

class ScrolledView: public View {
private:
    Gtk::ScrolledWindow _swin;
    Gtk::Widget& _schild;

public:
    ScrolledView(Gtk::Widget& child, const Glib::ustring& label) :
            View(_swin, label), _schild(child) {
        _swin.add(_schild);
        _swin.set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
        _swin.show();
    }

    virtual ~ScrolledView() {
    }

    virtual Gtk::Widget& child() const override {
        return _schild;
    }

    Gtk::ScrolledWindow& swin() {
        return _swin;
    }
};

#endif /* GUI_PARTS_SCROLLEDVIEW_H_ */
