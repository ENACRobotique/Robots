/*
 * LayersList.h
 *
 *  Created on: 4 nov. 2014
 *      Author: ludo6431
 */

#ifndef GUI_PARTS_VIEWLAYERSLIST_H_
#define GUI_PARTS_VIEWLAYERSLIST_H_

#include <business/layers/Layer.h>
#include <glibmm/refptr.h>
#include <glibmm/ustring.h>
#include <gtkmm/treeiter.h>
#include <gtkmm/treemodel.h>
#include <gtkmm/treemodelcolumn.h>
#include <gtkmm/treestore.h>
#include <gtkmm/treeview.h>
#include <gui/parts/ScrolledView.h>
#include <map>
#include <string>
#include <utility>

class ViewLayersList: public Gtk::TreeView, public ScrolledView {
private:
    Glib::RefPtr<Gtk::TreeStore> _treeModel;
    Gtk::TreeModel::ColumnRecord _columnsModel;
    Gtk::TreeModelColumn<int> _col_id;
    Gtk::TreeModelColumn<Glib::ustring> _col_name;
    Gtk::TreeModelColumn<bool> _col_draggable;

public:
    ViewLayersList(std::map<std::string, Layer*>& layers, const Glib::ustring& label) :
            ScrolledView(*this, label) {
        _columnsModel.add(_col_id);
        _columnsModel.add(_col_name);
        _columnsModel.add(_col_draggable);

        _treeModel = Gtk::TreeStore::create(_columnsModel);

        set_model(_treeModel);

        append_column("ID", _col_id);
        append_column("Name", _col_name);
        append_column_editable("Draggable", _col_draggable);

        swin().set_size_request(150, -1);

        int i = 0;
        for (auto& l : layers) {
            Gtk::TreeModel::Row row = *(_treeModel->append());
            row[_col_id] = i;
            row[_col_name] = l.second->get_group() + ": " + l.second->get_name();
            row[_col_draggable] = l.second->is_active();

            i++;
        }
    }
};

#endif /* GUI_PARTS_VIEWLAYERSLIST_H_ */
