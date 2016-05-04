/*
 * ProcAbsPos.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef PROCESSES_PROCABSPOS_H_
#define PROCESSES_PROCABSPOS_H_

#include <opencv2/core/core.hpp>
#include <processes/Process.h>
#include <tools/AbsPos2D.h>
#include <tools/Acq.h>
#include <tools/TestPoint.h>
#include <Transform2D.h>
#include <random>
#include <string>
#include <vector>

class ProjAcq;

#define COMP_TESTPOINTS
#define COMP_SIMULATED
//#define COMP_PLAYGROUND
//#define COMP_HSV
//#define HSV_TO_HGRAY
//#define HSV_TO_VGRAY

#define WRITE_IMAGES
//#define SHOW_IMAGES

class ProcAbsPos: public Process {
protected:
    std::vector<TestPoint> staticTP;

    std::vector<TestPoint> getPosDependentTP(const Pos& robPos);
    float getEnergy(ProjAcq& pAcq, const Pos& robPos);
    cv::Mat getSimulatedAt(ProjAcq& pAcq, const Pos& robPos) const;
    cv::Mat getSimulatedAt(ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const;
    void addTestPointsAtTo(cv::Mat& im, ProjAcq& pAcq, const Transform2D<float>& tr_pg2rob) const;
    cv::Mat getTestPointsAt(ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const;
    cv::Mat getPgWithSimulatedAt(ProjAcq& pAcq, const Pos& robPos) const;

    void handleStart(ProjAcq& pAcq, AbsPos2D<float> const& pos) const;
    void handleStep(ProjAcq& pAcq, AbsPos2D<float> const& endPos, int i) const;
    void handleEnd(ProjAcq& pAcq, AbsPos2D<float> const& endPos) const;

    cv::Mat pg;

    // prepare random generator
    std::random_device generator;

    float getRand() {
        static std::uniform_real_distribution<float> distribution(-1,1);
        return distribution(generator);
    }


    template<typename T>
    inline T clamp(T v, T m, T M) const { return std::max(m, std::min(v, M)); }

    template<typename T>
    cv::Point2i getInPGIm(cv::Point_<T> p) const {
        float factor = 4; // (px/cm)

        return {int(round(p.x * factor)) + 29, int(round((200 - p.y) * factor)) + 29};
    }

    template<typename T>
    cv::Point2i getInPGIm(cv::Point3_<T> p) const {
        constexpr float factor = 4; // (px/cm)

        return {int(round(p.x * factor)) + 29, int(round((200 - p.y) * factor)) + 29};
    }

    cv::Point2f getFromPgIm(cv::Point2i p) const {
        constexpr float factor = 4; // (px/cm)

        return {(p.x - 29.f)/factor, 200.f - (p.y - 29.f)/factor};
    }

public:
    ProcAbsPos(Cam* c, const std::string& staticTestPointFile, eVidTypeProc typeProcess);
};

#endif /* PROCESSES_PROCABSPOS_H_ */
