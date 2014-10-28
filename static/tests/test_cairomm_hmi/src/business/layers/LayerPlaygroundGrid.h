/*
 * LayerGrid.h
 *
 *  Created on: 27 oct. 2014
 *      Author: ludo6431
 */

#ifndef BUSINESS_LAYERS_LAYERPLAYGROUNDGRID_H_
#define BUSINESS_LAYERS_LAYERPLAYGROUNDGRID_H_

#include <business/layers/Layer.h>
#include <cairomm/context.h>
#include <cairomm/refptr.h>
#include <string>

class DAPlayground;

class LayerGrid: public Layer {
public:
    LayerGrid() :
            Layer("Grid", "Playground") {
    }

    LayerGrid(std::string name) :
            Layer(name, "Playground") {
    }

    LayerGrid(std::string name, std::string group) :
            Layer(name, group) {
    }

    virtual void on_draw(DAPlayground& dap, const Cairo::RefPtr<Cairo::Context>& cr);
};

#endif /* BUSINESS_LAYERS_LAYERPLAYGROUNDGRID_H_ */
