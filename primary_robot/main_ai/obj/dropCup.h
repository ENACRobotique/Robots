/*
 * dropCup.h
 *
 *  Created on: 4 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef OBJ_DROPCUP_H_
#define OBJ_DROPCUP_H_


#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"


using namespace std;

class DropCup : public Obj{
    public:
        DropCup(eColor_t color) : Obj(E_DROP_CUP), EP_selected(-1){
            vector<Point2D<float>> listEP{{50, 100}, {280, 50}, {280, 150}}; //Yellow
            sObjEntry_t objEP;

            _state = WAIT_MES;

            for(unsigned int i = 0 ; i < 3 ; i++){
                objEP.type = E_CIRCLE;
                if(color == YELLOW)
                    objEP.cir.c = {listEP[i].x, listEP[i].y};
                else
                    objEP.cir.c = {300 - listEP[i].x, listEP[i].y};
                objEP.cir.r = 10.;
                _access.push_back(objEP);
            }

        }
        virtual ~DropCup(){}

        void initObj(Point2D<float> , vector<astar::sObs_t>&, vector<Obj*>& listObj) override {
            //Drop the cup and back
            for(Obj* j : listObj){
                if(j->type() == E_DROP_CUP){
                    j->_access[_access_point_select].cir.c = {0,0};
                    /*
                    for(vector<>::iterator it = j->access().begin(); it != j->access().end(); ){
                        if(it-> == _access_select){
                            it = j->access().erase(it);
                        }
                        else {
                            it++;
                        }
                    }
                    */
                }
            }
        }
        int loopObj(vector<Obj*>&) override{
            _state = FINISH;
            return 0;
        }
        eObj_t type() const override {
            return E_DROP_CUP;
        }

    public:
        int EP_selected;

};




#endif /* OBJ_DROPCUP_H_ */
