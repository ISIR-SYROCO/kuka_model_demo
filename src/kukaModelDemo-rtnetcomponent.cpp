// Filename:  kukaModelDemo-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr) 
// Description:  

#include "kukaModelDemo-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaModelDemoRTNET::KukaModelDemoRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    model = new kukafixed("kuka");
}

void KukaModelDemoRTNET::updateHook(){

    std::string fri_mode("e_fri_unkown_mode");
    bool fri_cmd_mode = false;
    RTT::FlowStatus fs_event = iport_events.read(fri_mode);
    if (fri_mode == "e_fri_cmd_mode")
        fri_cmd_mode = true;
    else if (fri_mode == "e_fri_mon_mode")
        fri_cmd_mode = false;
        
    std::vector<double> JState(LWRDOF);
    std::vector<double> JVel(LWRDOF);
    RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
    RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel);

    if(joint_state_fs == RTT::NewData){        
        Eigen::VectorXd joint_pos(LWRDOF);
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos[i] = JState[i];
        }
        model->setJointPositions(joint_pos);
        
    }
    
    if(joint_vel_fs == RTT::NewData){
        Eigen::VectorXd joint_vel(LWRDOF);
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_vel[i] = JVel[i];
        }
        model->setJointVelocities(joint_vel);
    }

    Eigen::Displacementd posEndEffMes;
    RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
    if(cartPos_fs==RTT::NewData){
        double x = (double)X.position.x;
        double y = (double)X.position.y;
        double z = (double)X.position.z;
        double qx = (double)X.quaternion.x;
        double qy = (double)X.quaternion.y;
        double qz = (double)X.quaternion.z;
        double qw = (double)X.quaternion.w;
        posEndEffMes.x()=x;
        posEndEffMes.y()=y;
        posEndEffMes.z()=z;
        posEndEffMes.qx()=qx;
        posEndEffMes.qy()=qy;
        posEndEffMes.qz()=qz;
        posEndEffMes.qw()=qw;
    }
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(kuka_simple_demo)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(KukaModelDemoRTNET)
