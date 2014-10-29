/*
 * LayerFrame.h
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_LAYERS_LAYERPLAYGROUNDFRAME_H_
#define BUSINESS_LAYERS_LAYERPLAYGROUNDFRAME_H_

#include <business/layers/Layer.h>
#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <string>

class LayerFrame: public Layer {
public:
    LayerFrame() :
            Layer("Frame", "Playground") {
    }

    LayerFrame(std::string name) :
            Layer(name, "Playground") {
    }

    LayerFrame(std::string name, std::string group) :
            Layer(name, group) {
    }

    virtual void on_draw(DAPlayground& dap, const Cairo::RefPtr<Cairo::Context>& cr) override;
};

#endif /* BUSINESS_LAYERS_LAYERPLAYGROUNDFRAME_H_ */
