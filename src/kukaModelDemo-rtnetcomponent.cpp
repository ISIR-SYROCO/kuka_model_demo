// Filename:  kukaModelDemo-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author:  Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr) 
// Description:  

#include "kukaModelDemo-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaModelDemoRTNET::KukaModelDemoRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
    this->addOperation("setJointImpedance", &KukaModelDemoRTNET::setJointImpedance, this, RTT::OwnThread);
    this->addOperation("initDesiredPos", &KukaModelDemoRTNET::initDesiredPos, this, RTT::OwnThread);
    this->addOperation("setXcons", &KukaModelDemoRTNET::setDesiredPos, this, RTT::OwnThread);
    this->addOperation("setGains", &KukaModelDemoRTNET::setGains, this, RTT::OwnThread);
    this->addOperation("getJacobianModel", &KukaModelDemoRTNET::getJacobianModel, this, RTT::OwnThread);

    pose_des.resize(6);
    kp = 1;
    kd = 1;
    vmax = 0.1;
    tau.resize(LWRDOF);
	joint_position_command.assign(LWRDOF, 0.0);
}

bool KukaModelDemoRTNET::configureHook(){
    setPeer("lwr");
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }

    connectPorts();	
    return true;
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
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_position_command[i] = JState[i];
        }
        model.setJointPosition(JState);
    }
    
    Eigen::VectorXd joint_vel(LWRDOF);
    if(joint_vel_fs == RTT::NewData){    
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_vel[i] = JVel[i];
        }
        model.setJointVelocity(JVel);
    }

    RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
    if(cartPos_fs==RTT::NewData){
        double x = (double)X.position.x;
        double y = (double)X.position.y;
        double z = (double)X.position.z;
        double qx = (double)X.orientation.x;
        double qy = (double)X.orientation.y;
        double qz = (double)X.orientation.z;
        double qw = (double)X.orientation.w;
        posEndEffMes.x()=x;
        posEndEffMes.y()=y;
        posEndEffMes.z()=z;
        posEndEffMes.qx()=qx;
        posEndEffMes.qy()=qy;
        posEndEffMes.qz()=qz;
        posEndEffMes.qw()=qw;
    }

    Eigen::Displacementd posEndEffDes;
    posEndEffDes.x() = pose_des[0];
    posEndEffDes.y() = pose_des[1];
    posEndEffDes.z() = pose_des[2];
    KDL::Rotation desired_rotation = KDL::Rotation::RPY(pose_des[3], pose_des[4], pose_des[5]);
    desired_rotation.GetQuaternion(posEndEffDes.qx(), posEndEffDes.qy(), posEndEffDes.qz(), posEndEffDes.qw());

    Eigen::Displacementd delta;
    delta = posEndEffMes.inverse() * posEndEffDes;
    std::cout << "Delta " << delta.getTranslation() << std::endl;

    Eigen::Vector3d t_err;
    computeTranslationError(delta, t_err);
	t_err[0] = pose_des[0] - posEndEffMes.x();
	t_err[1] = pose_des[1] - posEndEffMes.y();
    t_err[2] = pose_des[2] - posEndEffMes.z();

    Eigen::MatrixXd J(3,7);
    //block(start_row, start_col, block_row, block_col)
    //only translation component (first 3 rows)
    //model.computeJacobian();
    //model.jacobian.changeBase(model.getSegmentPosition(8).M.Inverse()); 
    J = model.getSegmentJacobian(7).data.block(0,0, 3,7);

    Eigen::Vector3d f = kp * t_err;
    std::cout << "Error " << t_err.transpose() << std::endl;
    Eigen::VectorXd tau(LWRDOF);
    tau = J.transpose() * f - kd * joint_vel;
    std::cout << "Tau " << tau.transpose() << std::endl;
    
    //Send tau
	
    if (fri_cmd_mode){
        if(requiresControlMode(30)){
            std::vector<double> joint_eff_command;
            joint_eff_command.assign(LWRDOF, 0.0);
            for(unsigned int i=0; i<LWRDOF; ++i){
                joint_eff_command[i] = tau[i];
            }
            oport_add_joint_trq.write(joint_eff_command);
        }
        oport_joint_position.write(joint_position_command);
    }
	
}

void KukaModelDemoRTNET::setDesiredPos(std::vector<double>& pdes){
    for(unsigned int i = 0; i < 3; i++){
        pose_des[i] = pdes[i];
    }
    for(unsigned int i = 3; i < 6; i++){
        pose_des[i] = 0.0;
    }
}

void KukaModelDemoRTNET::initDesiredPos(){
    RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(X);
    if(cartPos_fs==RTT::NewData){
        double x = (double)X.position.x;
        double y = (double)X.position.y;
        double z = (double)X.position.z;
        double qx = (double)X.orientation.x;
        double qy = (double)X.orientation.y;
        double qz = (double)X.orientation.z;
        double qw = (double)X.orientation.w;
        posEndEffMes.x()=x;
        posEndEffMes.y()=y;
        posEndEffMes.z()=z;
        posEndEffMes.qx()=qx;
        posEndEffMes.qy()=qy;
        posEndEffMes.qz()=qz;
        posEndEffMes.qw()=qw;
        KDL::Rotation r;
        r.Quaternion(posEndEffMes.qx(), posEndEffMes.qy(), posEndEffMes.qz(), posEndEffMes.qw());
        pose_des[0] = posEndEffMes.x();
        pose_des[1] = posEndEffMes.y();
        pose_des[2] = posEndEffMes.z();
        r.GetRPY(pose_des[3], pose_des[4], pose_des[5]);
    }
    else{
        std::cout << "Fail to init desired pose" << std::endl;
    }

}

void KukaModelDemoRTNET::computeTranslationError(Eigen::Displacementd& delta, Eigen::Vector3d& t_err){
    double t = getPeriod();
    t_err = delta.getTranslation();
    //std::cout << "t norm " << t_err.norm() << " t : " << t << " vmax " << vmax << std::endl;
    if (t_err.norm() > 0.1){
        //t_err.normalize();
        t_err = t_err * vmax  / t_err.norm();
    }
    return;
}

void KukaModelDemoRTNET::computeOrientationError(Eigen::Displacementd& delta, Eigen::Vector3d& o_err){
}

void KukaModelDemoRTNET::setGains(double KP, double KD){
	kp=KP;
	kd=KD;
}

void KukaModelDemoRTNET::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
	if(stiffness.size() != LWRDOF || damping.size() != LWRDOF){
		std::cout << "Wrong vector size, should be " << LWRDOF << ", " << LWRDOF << std::endl;
		return;
	}else{
		lwr_fri::FriJointImpedance joint_impedance_command;
		for(unsigned int i = 0; i < LWRDOF; i++){
			joint_impedance_command.stiffness[i] = stiffness[i];
			joint_impedance_command.damping[i] = damping[i];
		}

		oport_joint_impedance.write(joint_impedance_command);
	}
}

std::vector<double> KukaModelDemoRTNET::getCartPos(){
    std::vector<double> cart_pos(12);
    cart_pos[0] = posEndEffMes.x();
    cart_pos[1] = posEndEffMes.y();
    cart_pos[2] = posEndEffMes.z();
    Eigen::MatrixXd cart_pos_mat(4,4);
    cart_pos_mat = posEndEffMes.toHomogeneousMatrix();
    for(int i=0; i<3; ++i){
        for(int j=0; j<3; ++j){
            cart_pos[3+3*i+j] = cart_pos_mat(i,j);
        }
    }
    return cart_pos;
}

std::vector<double> KukaModelDemoRTNET::getJacobianModel(int segmentIndex){
    std::vector<double> jac_model(42);
    //double q_1[] = {0.0, -0.0523626, 0.0, 1.51845, 0.0, -0.959863, 0.0};
    //std::vector<double> q1(q_1, q_1+7);
    //model.setJointPosition(q1);
        
    KDL::Jacobian jac = model.getSegmentJacobian(segmentIndex);
    for(int i=0; i<6; ++i){
        for(int j=0; j<7; ++j){
            jac_model[7*i+j] = jac.data(i, j);
        }
    }
    return jac_model;
}

void KukaModelDemoRTNET::connectPorts(){
	connectOJointPosition();
	connectOJointTorque();
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
