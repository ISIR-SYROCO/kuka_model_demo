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
        std::vector<double> pose_des;
        Eigen::Displacementd posEndEffMes;

        Eigen::VectorXd tau;
        std::vector<double> joint_position_command;

        double kp;
        double kd;
        double vmax;
        double omegamax;

        //x, y, z, rx, ry, rz
        void setDesiredPos(std::vector<double> &pdes);
        void initDesiredPos();

        void updateHook();

    private:
        void computeTranslationError(Eigen::Displacementd& delta, Eigen::Vector3d& t_err);
        void computeOrientationError(Eigen::Displacementd& delta, Eigen::Vector3d& o_err);
};

#endif
