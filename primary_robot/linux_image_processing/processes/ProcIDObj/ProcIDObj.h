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
#include "tools/Pos3D.h"
#include <tools/Acq.h>
#include <Transform2D.h>
#include <string>
#include <vector>
#include "tools.hpp"


typedef enum eStateCalib{
    notDone,
    succes,
    fail
}eSateCalib;

typedef std::map<eObjCol, std::pair<cv::Scalar, cv::Scalar>> mapCol_T;
typedef std::map<int, mapCol_T> mapsMapCol_T;

class ProcIDObj: public Process{
public:
    ProcIDObj(Cam* c, const std::string& objPlgrdFile, eVidTypeProc typeProcess);
    virtual ~ProcIDObj();

    void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
    void printObjList();

private:
    // Methods
    int loadListObj(const std::string& objPlgrdFile);
    int loadCalibCam(const std::string& calibCamFile, map<int, vector<pair<int, int>>>& listCalibArea);
    cv::Mat getBinaryImage(cv::Mat m, eObjCol col, const int idCam);
    void compContrs(const cv::Mat m, vector<vector<cv::Point>>& listCtrs);
    void compApproxCtr(const vector<cv::Point>& ctr, vector<cv::Point>& approxCtr);
    vector<pair<eObjShape, Pos3D<float>>> recogShape(const Acq& acq, const vector<cv::Mat>& vertexes);
    eObjType recogObjType(eObjCol col, eObjShape shape);
    vector<Play_Obj*> recogObj(const Acq& acq, vector<cv::Mat>& ctr, eObjCol col);
    vector<float> getPosOfShape(vector<cv::Mat>& ctr, eObjShape t);
    void setDim(std::vector<float>& dim, string& s, int n);
    void setColors(eObjCol& c, string& s, const int idCam);
    vector<float> getPosOfObj(const eObjType t, const vector<cv::Mat>& vertexes, const vector<Vector3D<float>>& edges);
    void CalibHSV(const std::vector<Acq*>& acqList);
    vector<Pos3D<float>> recoCubeAside(const cv::Mat C2R, const vector<cv::Mat>& vertexes);
    int compNbIdenticObjH(const eObjType objType, const cv::Mat& pt1, const cv::Mat& pt2, const float err = 0.1);
    int compNbIdenticObjV(const eObjType objType, const cv::Mat& ptCam_R, const cv::Mat& ptTable, const cv::Mat& ptProj, const float err = 0.1);
    bool isThereCube(const vector<cv::Mat>& vertexes, const Acq& acq);

protected:
    int _stateCalib;
    std::vector<int> listIdCamOfInterest;  // TODO: For future improvement
    std::vector<Play_Obj*> _listObj;
    std::vector<Play_Obj*> _listRefObj;
    mapsMapCol_T _listAcqColObj;
    vector<Play_Obj*> getSameInListRefObj(eObjCol col = objColMax, eObjShape shape = objShapeMax,
            vector<float> dim = vector<float>(0), eObjType type = objTypeMax, float epsDim = 0.05) const;
    Play_Obj* getObjInListRef(eObjType objType);
};

#endif /* PROCIDOBJ_H_ */
