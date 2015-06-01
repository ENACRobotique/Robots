/*
 * ViewsManager.h
 *
 *  Created on: 14 nov. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_VIEWS_VIEWSMANAGER_H_
#define BUSINESS_VIEWS_VIEWSMANAGER_H_

#include <gui/parts/View.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class ViewsManager {
private:
    std::unordered_multimap<std::string, View*> _views;

    void add(View* v) {
        _views.insert( { v->id(), v });
    }

public:
    ViewsManager() {

    }

    const std::unordered_multimap<std::string, View*>& get_views() const {
        return _views;
    }

    std::vector<View*> get_views_by_id(const std::string& view_id) {
        std::vector<View*> views;

        auto interval = _views.equal_range(view_id);

        // loop to avoid items with same hash :/
        for_each(interval.first, interval.second, [&views, &view_id] (std::pair<std::string, View*> v) {
            if(v.second->id() == view_id) {
                views.push_back(v.second);
            }
        });

        return views;
    }
};

#endif /* BUSINESS_VIEWS_VIEWSMANAGER_H_ */
