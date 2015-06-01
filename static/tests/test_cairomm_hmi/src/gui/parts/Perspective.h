/*
 * Perspective.h
 *
 *  Created on: 5 nov. 2014
 *      Author: ludo6431
 */

#ifndef GUI_PARTS_PERSPECTIVE_H_
#define GUI_PARTS_PERSPECTIVE_H_

#include <gui/parts/View.h>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class Perspective {
private:
    std::unordered_multimap<std::string, View*> _views;

public:
    void add(View* v) {
        _views.insert({v->id(), v});
    }

    Perspective(){

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

#endif /* GUI_PARTS_PERSPECTIVE_H_ */
