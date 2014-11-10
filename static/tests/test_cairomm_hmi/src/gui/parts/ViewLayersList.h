/*
 * LayersList.h
 *
 *  Created on: 4 nov. 2014
 *      Author: ludo6431
 */

#ifndef GUI_PARTS_VIEWLAYERSLIST_H_
#define GUI_PARTS_VIEWLAYERSLIST_H_

#include <gtkmm/button.h>
#include <gui/parts/View.h>

class ViewLayersList: public Gtk::Button, public View {
public:
    ViewLayersList() :
            Button("bt ll"), View(*this, "layers list") {
    }
};

#endif /* GUI_PARTS_VIEWLAYERSLIST_H_ */
