/*
 * ProcIDObj.h
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#ifndef PROCIDOBJ_H_
#define PROCIDOBJ_H_

#include <opencv2/core/core.hpp>
#include "../Process.h"
#include "../../Playground objects/PlayObj.h"
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <Transform2D.h>
#include <string>
#include <vector>

class ProcIDObj: public Process{
public:
    ProcIDObj(Cam* c, const std::string& objPlgrdFile);
    virtual ~ProcIDObj();

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
    void printObjList();

private:
    int loadListObj(const std::string& objPlgrdFile);

protected:
    std::vector<Play_Obj*> _objList;
};

#endif /* PROCIDOBJ_H_ */
