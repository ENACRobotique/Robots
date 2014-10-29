/*
 * Layer.h
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_LAYERS_LAYER_H_
#define BUSINESS_LAYERS_LAYER_H_

#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <gdk/gdk.h>
#include <string>

class DAPlayground;

class Layer {
public:
    Layer(std::string name) :
            name(name), group(""), active(false) {
    }

    Layer(std::string name, std::string group) :
            name(name), group(group), active(false) {
    }

    virtual ~Layer() {
    }

    const std::string& get_name() const {
        return name;
    }

    const std::string& get_group() const {
        return group;
    }

    void set_active(bool active) {
        this->active = active;
    }

    void toggle_active() {
        active = !active;
    }

    bool is_active() const {
        return active;
    }

    virtual bool on_event(DAPlayground& dap, const GdkEvent& event) {
        return false; // return true if handled here
    }

    virtual void on_draw(DAPlayground& dap, const Cairo::RefPtr<Cairo::Context>& cr) {
    }

protected:
    std::string name;
    std::string group;
    bool active;
};

#endif /* BUSINESS_LAYERS_LAYER_H_ */
