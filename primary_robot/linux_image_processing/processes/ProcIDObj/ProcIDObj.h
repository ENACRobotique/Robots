/*
 * ProcIDObj.h
 *
 *  Created on: Nov 11, 2015
 *      Author: yoyo
 */

#ifndef PROCIDOBJ_H_
#define PROCIDOBJ_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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
    cv::Mat getBinaryImage(cv::Mat m, int col);
    void compContrs(const cv::Mat m, vector<vector<cv::Point>>& listCtrs);
    void compApproxCtr(const vector<cv::Point>& ctr, vector<cv::Point>& approxCtr);
    eObjShape recogShape(const vector<cv::Mat>& vertexes, vector<Vector3D<float>>& edges);
    eObjType recogObjType(vector<cv::Mat>& vertexes, eObjCol col, eObjShape shape);
    Play_Obj *recogObj(vector<cv::Mat>& ctr, eObjCol col);
    vector<float> getPosOfShape(vector<cv::Mat>& ctr, eObjShape t);
    void setDim(std::vector<float>& dim, string& s, int n);
    void setColors(eObjCol& c, string& s);
    bool isColObjExist(const eObjCol c);

protected:
    std::vector<Play_Obj*> _listObj;
    std::vector<Play_Obj*> _listRefObj;
    std::vector<std::tuple<eObjCol, cv::Scalar, cv::Scalar>> _listColObj;
    vector<Play_Obj*> getSameInListRefObj(eObjCol col = objColMax, eObjShape shape = objShapeMax,
            vector<float> dim = vector<float>(0), eObjType type = objTypeMax, float epsDim = 0.05) const;
};

#endif /* PROCIDOBJ_H_ */
