/*
 * duneheap.h
 *
 *  Created on: 24 févr. 2016
 *      Author: guilhem
 */

#ifndef OBJ_DUNEHEAP_H_
#define OBJ_DUNEHEAP_H_

#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"

using namespace std;

class Dune_heap : public Obj{
public:
	Dune_heap(unsigned int num);
	virtual ~Dune_heap();

    void initObj(paramObj) override;
    int loopObj(paramObj) override;
    eObj_t type() const override {return E_DUNEHEAP;};

private:
    unsigned int num;
};

#endif /* OBJ_DUNEHEAP_H_ */
