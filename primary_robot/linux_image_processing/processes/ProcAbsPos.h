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

class ProcAbsPos: public Process {
protected:
    std::vector<TestPoint> staticTP;

    std::vector<TestPoint> getPosDependentTP(const Pos& robPos);
    float getEnergy(ProjAcq& pAcq, const Pos& robPos);
    cv::Mat getSimulatedAt(ProjAcq& pAcq, const Pos& robPos) const;
    cv::Mat getSimulatedAt(ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const;
    void addTestPointsAtTo(cv::Mat& im, ProjAcq& pAcq, const Transform2D<float>& tr_pg2rob) const;
    cv::Mat getTestPointsAt(ProjAcq& pAcq, const Transform2D<float>& tr_rob2pg) const;

    cv::Mat pg;

    // prepare random generator
    std::random_device generator;

    float getRand() {
        static std::uniform_real_distribution<float> distribution(-1,1);
        return distribution(generator);
    }

public:
    ProcAbsPos(Cam* c, const std::string& staticTestPointFile);
    virtual ~ProcAbsPos();

    virtual void process(const std::vector<Acq*>& acqList, const Pos& pos, const PosU& posU) override;
};

#endif /* PROCESSES_PROCABSPOS_H_ */
