/**
 * Project Untitled
 */


#ifndef _PROCESSLIDARDATA_H
#define _PROCESSLIDARDATA_H

class ProcessLidarData {
public: 
    
    /**
     * @param points
     * @param lastPos
     */
    void process(vector<PtLidar> points, <Pos2D> lastPos);
    
    Pos2D getPos();
    
    vector<Pos2D> getAdversaires();
private: 
    vector<Group> groups;
    vector<Object*> objects;
    Pos2D position;
};

#endif //_PROCESSLIDARDATA_H