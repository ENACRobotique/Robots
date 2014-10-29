/*
 * LayerFrame.h
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_LAYERS_LAYERPLAYGROUNDOUTLINE_H_
#define BUSINESS_LAYERS_LAYERPLAYGROUNDOUTLINE_H_

#include <business/layers/Layer.h>
#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <string>

class LayerOutline: public Layer {
public:
    LayerOutline() :
            Layer("Outline", "Playground") {
        visible = true;
    }

    LayerOutline(std::string name) :
            Layer(name, "Playground") {
        visible = true;
    }

    LayerOutline(std::string name, std::string group) :
            Layer(name, group) {
        visible = true;
    }

    virtual void on_draw(DAPlayground& dap, const Cairo::RefPtr<Cairo::Context>& cr) override;
};

#endif /* BUSINESS_LAYERS_LAYERPLAYGROUNDOUTLINE_H_ */
