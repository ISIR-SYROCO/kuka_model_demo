// Filename:  kukaSimpleDemo-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr) 
// Description: Orocos component to command the kuka using simple controllers

#ifndef KUKA_SIMPLE_DEMO_RTNET_COMPONENT_HPP
#define KUKA_SIMPLE_DEMO_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include <Eigen/Dense>

class KukaSimpleDemoRTNET : public FriRTNetExampleAbstract{
    public:
        KukaSimpleDemoRTNET(std::string const& name);

        void updateHook();
};

#endif
