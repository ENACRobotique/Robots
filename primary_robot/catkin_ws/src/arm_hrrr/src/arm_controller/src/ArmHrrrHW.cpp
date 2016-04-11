#include "ArmHrrrHW.h"

ArmHrrrHW::ArmHrrrHW(std::vector<std::string> jntNames) : _N(jntNames.size()),
    _name(jntNames), _pos(_N, 0.0), _cmd(_N, 0.0), _vel(_N, 0.0), _eff(_N, 0.0),
    _stateHandles(_N), _posHandles(_N){

    // Connect and register the joint state and position interfaces
    for (int i=0; i<_N; ++i){
        _stateHandles[i] = new hardware_interface::JointStateHandle(_name[i],
            &_pos[i] , &_vel[i], &_eff[i]);
        _jointStateInterface.registerHandle(*_stateHandles[i]);
        _posHandles[i] = new hardware_interface::JointHandle(
            _jointStateInterface.getHandle(_name[i]), &_cmd[i]);
        _jointPosInterface.registerHandle(*_posHandles[i]);
    }
    registerInterface(&_jointStateInterface);
    registerInterface(&_jointPosInterface);
}


double ArmHrrrHW::getCmd(const int &i) const{
    if ( i >= 0 && i < _N )
        return _cmd[i];
    else
        return 0.0;
}


void ArmHrrrHW::setCmd(const int& i, const double& val){
    if ( i >= 0 && i < _N )
        _cmd[i] = val;
    else
        ROS_INFO("ArmHrrrHW::setCmd(): invalid index %i", i);
}


double ArmHrrrHW::getPos(const int &i) const{
    if ( i >= 0 && i < _N )
        return _pos[i];
    else
        return 0.0;
}


void ArmHrrrHW::setPos(const int& i, const double& val){
    if ( i >= 0 && i < _N )
        _pos[i] = val;
    else
        ROS_INFO("ArmHrrrHW::setCmd(): invalid index %i", i);
}
