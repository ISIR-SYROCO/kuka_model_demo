// Filename:  kukaSimpleDemo-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr) 
// Description: Orocos component to command the kuka using simple controllers

#ifndef KUKA_SIMPLE_DEMO_RTNET_COMPONENT_HPP
#define KUKA_SIMPLE_DEMO_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include <Eigen/Dense>

class KukaModelDemoRTNET : public FriRTNetExampleAbstract{
    public:
        KukaModelDemoRTNET(std::string const& name);

        void updateHook();
};

#endif
