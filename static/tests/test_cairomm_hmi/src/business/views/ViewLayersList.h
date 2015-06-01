/*
 * LayersList.h
 *
 *  Created on: 4 nov. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_VIEWS_VIEWLAYERSLIST_H_
#define BUSINESS_VIEWS_VIEWLAYERSLIST_H_

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
    Glib::RefPtr<Gtk::TreeStore> _tree_model;
    Gtk::TreeModel::ColumnRecord _columns_model;
    Gtk::TreeModelColumn<int> _colid_id;
    Gtk::TreeModelColumn<Glib::ustring> _colid_name;
    Gtk::TreeModelColumn<bool> _colid_draggable;

public:
    static const std::string VID;

    ViewLayersList(std::map<std::string, Layer*>& layers, const Glib::ustring& label) :
            ScrolledView(VID, *this, label) {
        _columns_model.add(_colid_id);
        _columns_model.add(_colid_name);
        _columns_model.add(_colid_draggable);

        _tree_model = Gtk::TreeStore::create(_columns_model);

        set_model(_tree_model);

        append_column("ID", _colid_id);
        append_column("Name", _colid_name);
        append_column_editable("Draggable", _colid_draggable);

        swin().set_size_request(150, -1);

        int i = 0;
        for (auto& l : layers) {
            Gtk::TreeModel::Row row = *(_tree_model->append());
            row[_colid_id] = i;
            row[_colid_name] = l.second->get_group() + ": " + l.second->get_name();
            row[_colid_draggable] = l.second->is_active();

            i++;
        }
    }
};

#endif /* BUSINESS_VIEWS_VIEWLAYERSLIST_H_ */
