//
// Created by jemin on 3/27/19.
// Modified by Inkyu on 22/July/2019
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/* Convention
*
*   observation space = [ theta (tilted angle),              n =  1, si =  0
*                         theta dot (tilted angle velocity,  n =  1, si =  1
*                         x (position),                      n =  1, si =  2
*                         x dot (body Linear velocities),    n =  1, si =  3] total 4
*/


#include <stdlib.h>
#include <cstdint>
#include <set>
#include <raisim/OgreVis.hpp>
#include "RaisimGymEnv.hpp"
#include "visSetupCallback.hpp"

#include "visualizer/raisimKeyboardCallback.hpp"
#include "visualizer/helper.hpp"
#include "visualizer/guiState.hpp"
#include "visualizer/raisimBasicImguiPanel.hpp"

using namespace std;
namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const YAML::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), distribution_(0.0, 0.2), visualizable_(visualizable) {

    /// add objects
    cout<<"hello adsfadfss"<<endl;
    cout<<resourceDir<<endl;
    cartpole_ = world_->addArticulatedSystem(resourceDir+"/cartpole.urdf");
    //cartpole_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    cartpole_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    auto ground = world_->addGround();
    world_->setERP(0,0);
    cout<<"hello2"<<endl;
    /// get robot data
    gcDim_ = cartpole_->getGeneralizedCoordinateDim();
    gvDim_ = cartpole_->getDOF();
    nJoints_ = 1;
    cout<<"gcDim_="<<gcDim_<<"gvDim_="<<gvDim_<<endl;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    torque_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of anymal
    //gc_init_ << 0, 0, 0.44, 0.7071067, 0.70710678, 0.0, 0.0, 0.0, 0.0, -0.7, 0, 0, -0.7, 0.00, 0, -0.7, 0, 0, -0.7;

    /// set pd gains
    Eigen::VectorXd jointPgain(gcDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(40.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(1.0);
    cartpole_->setPdGains(jointPgain, jointDgain);
    cartpole_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 4; /// theta, theta_dot, x, x_dot
    actionDim_ = nJoints_;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obMean_.setZero(obDim_); obStd_.setZero(obDim_);

    /// action & observation scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.6);

    obMean_.setZero();
    obStd_ << 2*M_PI,10,3,0.1;
    // obMean_ << 0.44, /// average height
    //     0.0, 0.0, 0.0, /// gravity axis 3
    //     gc_init_.tail(12), /// joint position 12
    //     Eigen::VectorXd::Constant(6, 0.0), /// body lin/ang vel 6
    //     Eigen::VectorXd::Constant(12, 0.0); /// joint vel history

    // obStd_ << 0.12, /// average height
    //     Eigen::VectorXd::Constant(3, 0.7), /// gravity axes angles
    //     Eigen::VectorXd::Constant(12, 1.0 / 1.0), /// joint angles
    //     Eigen::VectorXd::Constant(3, 2.0), /// linear velocity
    //     Eigen::VectorXd::Constant(3, 4.0), /// angular velocities
    //     Eigen::VectorXd::Constant(12, 10.0); /// joint velocities

    /// Reward coefficients
    forwardVelRewardCoeff_ = cfg["forwardVelRewardCoeff"].as<double>();
    torqueRewardCoeff_ = cfg["torqueRewardCoeff"].as<double>();
    gui::rewardLogger.init({"forwardVelReward", "torqueReward"});

    /// indices of links that should not make contact with ground
    // footIndices_.insert(cartpole_->getBodyIdx("FR_lower_leg"));
    // footIndices_.insert(cartpole_->getBodyIdx("FL_lower_leg"));
    // footIndices_.insert(cartpole_->getBodyIdx("RR_lower_leg"));
    // footIndices_.insert(cartpole_->getBodyIdx("RL_lower_leg"));

    /// visualize if it is the first environment
    if (visualizable_) {
      auto vis = raisim::OgreVis::get();

      /// these method must be called before initApp
      vis->setWorld(world_.get());
      vis->setWindowSize(1800, 1200);
      vis->setImguiSetupCallback(imguiSetupCallback);
      vis->setImguiRenderCallback(imguiRenderCallBack);
      vis->setKeyboardCallback(raisimKeyboardCallback);
      vis->setSetUpCallback(setupCallback);
      vis->setAntiAliasing(2);

      /// starts visualizer thread
      vis->initApp();

      cartpoleVisual_ = vis->createGraphicalObject(cartpole_, "Cartpole");
      vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");
      desired_fps_ = 50.;
      vis->setDesiredFPS(desired_fps_);
    }
  }

  ~ENVIRONMENT() final = default;

  void init() final { }

  void reset() final {
    cartpole_->setState(gc_init_, gv_init_);
    updateObservation();
    if(visualizable_)
      gui::rewardLogger.clean();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    actionScaled_ = action.cast<double>();
    actionScaled_ = actionScaled_.cwiseProduct(actionStd_);
    actionScaled_ += actionMean_;

    /// action scaling
    // pTarget12_ = action.cast<double>();
    // pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    // pTarget12_ += actionMean_;
    // pTarget_.tail(nJoints_) = pTarget12_;
    cartpole_->setGeneralizedForce(actionScaled_);

    //cartpole_->setPdTarget(pTarget_, vTarget_);
    auto loopCount = int(control_dt_ / simulation_dt_ + 1e-10);
    auto visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);

    for(int i=0; i<loopCount; i++) {
      world_->integrate();

      if (visualizable_ && visualizeThisStep_ && visualizationCounter_ % visDecimation == 0)
        raisim::OgreVis::get()->renderOneFrame();

      visualizationCounter_++;
    }

    updateObservation();

    torqueReward_ = torqueRewardCoeff_ * cartpole_->getGeneralizedForce().squaredNorm();
    forwardVelReward_ = forwardVelRewardCoeff_ * bodyLinearVel_[2];

    if(visualizeThisStep_) {
      gui::rewardLogger.log("torqueReward", torqueReward_);
      gui::rewardLogger.log("forwardVelReward", forwardVelReward_);

      /// set camera
      auto vis = raisim::OgreVis::get();
      vis->select(cartpoleVisual_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(-1.57), Ogre::Radian(-1.0), 3);
    }

    return torqueReward_ + forwardVelReward_;
  }

  void updateExtraInfo() final {
    extraInfo_["forward vel reward"] = forwardVelReward_;
    extraInfo_["base height"] = gc_[2];
  }

  void updateObservation() {
    cartpole_->getState(gc_, gv_);
    obDouble_.setZero(obDim_); obScaled_.setZero(obDim_);
    cout<<"gc_="<<gc_<<endl;
    cout<<"gv_="<<gv_<<endl;
    // Articulated sysstem (Slidebar->cart->pole)
    // auto cartFrame = cartpole_->getFrameByIdx("slider_to_cart"); //prismatic joint
    // raisim::Vec<3> cartPosition;
    // cartpole_->getFramePosition_W(cartFrame,cartPosition);

    // auto poleFrame = cartpole_->getFrameByIdx("cart_to_pole");
    // raisim::Mat<3,3> poleOrientationR;
    // raisim::Vec<4> quat;
    // cartpole_->getFramePosition_W(poleFrame,poleOrientationR);
    //raisim::rotMatToQuat(quat,poleOrientationR);


    
    obDouble_ << gc_[1],gv_[1],gc_[0],gv_[0]; //theta, theta_dot, x, x_dot

    // obDouble_[0] = gc_[0]; //theta
    // obDouble_[1] = gv_[0]; //theta_dot
    // obDouble_[2] = gc_[1]; //x, get cart position
    // obDouble_[3] = gv_[1]; //x_dot, get cart velocity
    obScaled_ = (obDouble_-obMean_).cwiseQuotient(obStd_);

    // /// body orientation
    // raisim::Vec<4> quat;
    // raisim::Mat<3,3> rot;
    // quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    // raisim::quatToRotMat(quat, rot);
    // obDouble_.segment(1, 3) = rot.e().row(2);

    // /// joint angles
    // obDouble_.segment(4, 12) = gc_.tail(12);

    // /// body velocities
    // bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    // bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    // obDouble_.segment(16, 3) = bodyLinearVel_;
    // obDouble_.segment(19, 3) = bodyAngularVel_;

    // /// joint velocities
    // obDouble_.tail(12) = gv_.tail(12);
    // obScaled_ = (obDouble_-obMean_).cwiseQuotient(obStd_);
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obScaled_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: cartpole_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end()) {
        return true;
      }

    terminalReward = 0.f;
    return false;
  }

  void setSeed(int seed) final {
    std::srand(seed);
  }

  void close() final {
  }

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  std::normal_distribution<double> distribution_;
  raisim::ArticulatedSystem* cartpole_;
  std::vector<GraphicObject> * cartpoleVisual_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, actionScaled_,pTarget_, pTarget12_, vTarget_, torque_;
  double terminalRewardCoeff_ = -10.;
  double forwardVelRewardCoeff_ = 0., forwardVelReward_ = 0.;
  double torqueRewardCoeff_ = 0., torqueReward_ = 0.;
  double desired_fps_ = 60.;
  int visualizationCounter_=0;
  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;
  Eigen::VectorXd obDouble_, obScaled_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
};

}

