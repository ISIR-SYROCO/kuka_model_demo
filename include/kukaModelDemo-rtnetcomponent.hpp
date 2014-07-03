// Filename:  kukaModelDemo-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr) 
// Description: 

#ifndef KUKA_MODEL_DEMO_RTNET_COMPONENT_HPP
#define KUKA_MODEL_DEMO_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include "kukafixed.h"
#include <Eigen/Dense>

#include <orc/control/Feature.h>
#include <orc/control/ControlFrame.h>
#include <orc/control/ControlEnum.h>

class KukaModelDemoRTNET : public FriRTNetExampleAbstract{
    public:
        KukaModelDemoRTNET(std::string const& name);
        kukafixed* model;

      	geometry_msgs::Pose X;

        void updateHook();
};

#endif
