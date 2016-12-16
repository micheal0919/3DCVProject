#pragma once
#include <map>
#include "MarkerRigidBody.h"
#include "opencv\cxcore.h"

class MarkerRigidBodyPoseMap :
    public std::map<const MarkerRigidBody *, PoseMatrix>
{
public:
    MarkerRigidBodyPoseMap(void);
    ~MarkerRigidBodyPoseMap(void);
};
