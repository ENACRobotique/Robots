/*
 * LayerFrame.h
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_LAYERS_LAYERPLAYGROUNDCLICKS_H_
#define BUSINESS_LAYERS_LAYERPLAYGROUNDCLICKS_H_

#include <business/layers/Layer.h>
#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <gdk/gdk.h>
#include <string>

class LayerClicks: public Layer {
public:
    LayerClicks() :
            LayerClicks("Last click", "Playground") {
    }

    LayerClicks(std::string name) :
            LayerClicks(name, "Playground") {
    }

    LayerClicks(std::string name, std::string group) :
            Layer(name, group) {
        active = true;

        mouse_lastpress_moved = false;
        user_mouse_lastpress_moved = false;
        mouse_lastpress_x__px = 0;
        mouse_lastpress_y__px = 0;
        mouse_lastpress_x__cm = 0;
        mouse_lastpress_y__cm = 0;
    }

    virtual bool on_event(DAPlayground& dap, const GdkEvent& event) override;

    virtual void on_draw(DAPlayground& dap, const Cairo::RefPtr<Cairo::Context>& cr) override;

protected:
    bool mouse_lastpress_moved, user_mouse_lastpress_moved;
    double mouse_lastpress_x__px, mouse_lastpress_y__px; // (px)
    double mouse_lastpress_x__cm, mouse_lastpress_y__cm; // (cm)

    void event_click__px(double x, double y) {
        mouse_lastpress_x__px = x;
        mouse_lastpress_y__px = y;
        mouse_lastpress_moved = true;
    }
    bool state_click_moved() {
        return user_mouse_lastpress_moved ? (user_mouse_lastpress_moved = false, true) : false;
    }

    void state_get_click__cm(double& x, double& y) {
        x = mouse_lastpress_x__cm;
        y = mouse_lastpress_y__cm;
    }
};

#endif /* BUSINESS_LAYERS_LAYERPLAYGROUNDCLICKS_H_ */
