/*
 * Folder.h
 *
 *  Created on: 5 nov. 2014
 *      Author: ludo6431
 */

#ifndef GUI_PARTS_FOLDER_H_
#define GUI_PARTS_FOLDER_H_

#include <gtkmm/notebook.h>
#include <gui/parts/View.h>
#include <vector>

class Folder: public Gtk::Notebook {
private:
    std::vector<View*> views;

public:
    void add(View* v) {
        views.push_back(v);

        append_page(v->widget(), v->label());

        v->widget().show();
    }
};

#endif /* GUI_PARTS_FOLDER_H_ */
