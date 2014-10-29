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
            name(name), group(""), visible(false) {
    }

    Layer(std::string name, std::string group) :
            name(name), group(group), visible(false) {
    }

    virtual ~Layer() {
    }

    const std::string& get_name() const {
        return name;
    }

    const std::string& get_group() const {
        return group;
    }

    void set_visible(bool visible) {
        this->visible = visible;
    }

    void toggle_visible() {
        visible = !visible;
    }

    bool is_visible() const {
        return visible;
    }

    virtual bool on_event(DAPlayground& dap, const GdkEvent& event) {
        return false; // return true if handled here
    }

    virtual void on_draw(DAPlayground& dap, const Cairo::RefPtr<Cairo::Context>& cr) {
    }

protected:
    std::string name;
    std::string group;
    bool visible;
};

#endif /* BUSINESS_LAYERS_LAYER_H_ */
