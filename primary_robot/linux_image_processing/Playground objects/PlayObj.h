/*
 * Play_Obj.h
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#ifndef PLAYOBJ_H_
#define PLAYOBJ_H_

#include <tools/AbsPos2D.h>
#include <string.h>
#include "Play_obj_type.h"

using namespace std;

class Play_Obj {
public:
    Play_Obj(eObjType type, eObjShape shape, vector<float>& dim, eObjCol color);
    virtual ~Play_Obj();

    void setConf(AbsPos2D<float> conf);
    bool isDimEqual(std::vector<float> dim, float eps);
    eObjType getType() const;
    eObjShape getShape()const;
    eObjCol getCol()const;
    std::vector<float> getDim() const;
    void print();

private:
    eObjType _type;
    eObjShape _shape;
    eObjCol _col;
    vector<float> _dim;
    AbsPos2D<float> _conf;
};

#endif /* PLAYOBJ_H_ */
