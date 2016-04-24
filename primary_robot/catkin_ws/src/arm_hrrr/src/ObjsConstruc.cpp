#include "ObjsConstruc.h"
#include "ToolGeometryMsg.h"

ObjsConstruc::ObjsConstruc(eTypeConstruc type, geometry_msgs::Pose pose,
                            std::vector<moveit_msgs::CollisionObject*> objs){
    _type = type;
    _pose = pose;
    _objs.resize(objs.size());
    for(int i=0; i<(int)objs.size(); i++){
        _objs[i].first = false;
        _objs[i].second = objs[i];
    }
    std::cout<<"Construction of type "<<type<<" created\n";
    makePlanConstruc();
    printPlanConstruc();
}

void ObjsConstruc::makePlanConstruc(){
    std::vector<int> idxObjs;
    shape_msgs::SolidPrimitive prim;
    geometry_msgs::Pose poseObj;
    switch(_type){
    case tower11:  // Construct a tower 1 cylinder first and next 1 cone above it
        idxObjs.push_back(findObj(prim.CYLINDER, false));
        if(idxObjs[0] >= 0){
            idxObjs.push_back(findObj(prim.CONE, false));
            if(idxObjs[1] >= 0){
                //Build planConstruc for the first piece
                _planConstruc.push_back(std::make_tuple(grabObj, vertical, _objs[idxObjs[0]].second->id,
                            _objs[idxObjs[0]].second->primitive_poses[0]));
                copyGeometryMsg(_pose, poseObj);
                poseObj.position.z = DIMOBJ/2.;
                _planConstruc.push_back(std::make_tuple(putObj, vertical, _objs[idxObjs[0]].second->id,
                            poseObj));
                //Build planConstruc for the second piece
                _planConstruc.push_back(std::make_tuple(grabObj, vertical, _objs[idxObjs[1]].second->id,
                            _objs[idxObjs[1]].second->primitive_poses[0]));
                poseObj.position.z += DIMOBJ;
                _planConstruc.push_back(std::make_tuple(putObj, vertical, _objs[idxObjs[1]].second->id,
                            poseObj));
            }
            else{
                ROS_WARN("Could not find cone obj to build tower11");
            }
        }
        else{
            ROS_WARN("Could not find cylinder obj to build tower11");
        }

        break;
    case tower21:
        idxObjs.push_back(findObj(prim.CYLINDER, false));
        if(idxObjs[0] >= 0){
             _objs[idxObjs[0]].first = true;  // Is reserved for the construction
            idxObjs.push_back(findObj(prim.CYLINDER, false));
            if(idxObjs[1] >= 0){
                _objs[idxObjs[1]].first = true;  // Is reserved for the construction
                idxObjs.push_back(findObj(prim.CONE, false));
                if(idxObjs[2] >= 0){
                    _objs[idxObjs[2]].first = true;  // Is reserved for the construction

                    //Build planConstruc for the first piece
                    _planConstruc.push_back(std::make_tuple(grabObj, vertical, _objs[idxObjs[0]].second->id,
                                _objs[idxObjs[0]].second->primitive_poses[0]));
                    copyGeometryMsg(_pose, poseObj);
                    poseObj.position.z = DIMOBJ/2.;
                    _planConstruc.push_back(std::make_tuple(putObj, vertical, _objs[idxObjs[0]].second->id,
                                poseObj));
                    //Build planConstruc for the second piece
                    _planConstruc.push_back(std::make_tuple(grabObj, vertical, _objs[idxObjs[1]].second->id,
                                _objs[idxObjs[1]].second->primitive_poses[0]));
                    poseObj.position.z += DIMOBJ;
                    _planConstruc.push_back(std::make_tuple(putObj, vertical, _objs[idxObjs[1]].second->id,
                                poseObj));

                    //Build planConstruc for the third piece
                    _planConstruc.push_back(std::make_tuple(grabObj, vertical, _objs[idxObjs[2]].second->id,
                                _objs[idxObjs[2]].second->primitive_poses[0]));
                    poseObj.position.z += DIMOBJ;
                    _planConstruc.push_back(std::make_tuple(putObj, vertical, _objs[idxObjs[2]].second->id,
                                poseObj));
                }
                else{
                    ROS_WARN("Could not find cone obj to build tower11");
                }
            }
        }
        else{
            ROS_WARN("Could not find cylinder obj to build tower11");
        }
        break;
    case wall222:

        break;
    case wall322:

        break;
    default:
        ROS_WARN("Unknown type of construction %d", _type);
    }
}

const planConstruc_t ObjsConstruc::getPlanConstruc() const{
    return _planConstruc;
}

int ObjsConstruc::findObj(uint8_t shape, bool isInConstruc){
    int i;
    for(i=0; i<(int)_objs.size(); i++){
        uint8_t type = _objs[i].second->primitives[0].type;
        if(type == shape  &&  _objs[i].first == isInConstruc){
            return i;
        }
    }

    if(i == (int)_objs.size())
        return -1;
}

void ObjsConstruc::printStepConstruc(int i) const{
    std::cout<<"Step "<<i<<" : "<<eActionManip2str(std::get<0>(_planConstruc[i]))<<", "<<
                eOrientationTool2str(std::get<1>(_planConstruc[i]))<<", "<<
                std::get<2>(_planConstruc[i])<<", "<<
                "pose = ["<<std::get<3>(_planConstruc[i]).position.x<<", "<<
                            std::get<3>(_planConstruc[i]).position.y<<", "<<
                            std::get<3>(_planConstruc[i]).position.z<<", "<<
                            std::get<3>(_planConstruc[i]).orientation.w<<", "<<
                            std::get<3>(_planConstruc[i]).orientation.x<<", "<<
                            std::get<3>(_planConstruc[i]).orientation.y<<", "<<
                            std::get<3>(_planConstruc[i]).orientation.z<<"]\n";

}

void ObjsConstruc::printPlanConstruc() const{
    int n = (int)_planConstruc.size();

    for(int i=0; i<n; i++){
        printStepConstruc(i);
    }
}

std::string eActionManip2str(eActionManip act){
    switch(act){
    case grabObj:
        return std::string("grabObj");
    case putObj:
        return std::string("putObj");
    case eActionManipMax:
        return std::string("eActionManipMax");
    default:
        return std::string("");
    }
}

std::string eOrientationTool2str(eOrienTool orien){
    switch(orien){
    case vertical:
        return std::string("vertical");
    case horizontal:
        return std::string("horizontal");
    case eOrienToolMax:
        return std::string("eOrienToolMax");
    default:
        return std::string("");
    }
}

std::string eTypeConstruc2str(eTypeConstruc type){
    switch(type){
    case tower11:
        return std::string("tower11");
    case eTypeConstrucMax:
        return std::string("eTypeConstrucMax");
    default:
        return std::string("");
    }
}

std::string eShape2str(eShape shape){
    switch(shape){
    case cub:
        return std::string("cube");
    case cyl:
        return std::string("cylindre");
    case con:
        return std::string("cone");
    case eShapeMax:
        return std::string("eShapeMax");
    default:
        return std::string("");
    }
}


