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
    Play_Obj(eObjType type, vector<double>& dim, vector<int>& RGB_color);
    virtual ~Play_Obj();

    void setConf(AbsPos2D<double> conf);
    void print();

private:
    eObjType _type;
    vector<double> _dim;
    vector<int> _RGB_color;
    AbsPos2D<double> _conf;
};

#endif /* PLAYOBJ_H_ */
