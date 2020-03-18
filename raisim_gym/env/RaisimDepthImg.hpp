//
// Created by Inkyu on 12/Mar/20.
// MIT License
//
// Copyright (c) 2020 CSIRO, Brisbane
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

#ifndef RAISIM_RAISIMDEPTHIMG_HPP
#define RAISIM_RAISIMDEPTHIMG_HPP

#ifdef __linux__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#elif WINDOWS

#endif

#include <mutex>
#include <thread>
#include <string>
#include <fstream>
#include <atomic>
#include <sys/wait.h>
#include "raisim/World.hpp"

namespace raisim {

class RaisimDepthImg {

// ======================================================
// public
// ======================================================
  
 public:
  RaisimDepthImg()
  {
    collisiongroup_=1;
    ray_length_=5; //in metre
  }

  void init(raisim::ArticulatedSystem *as)
  {
    as_=as;
  }
  //~RaisimDepthImg(); //causing undefined symbol: _ZN6raisim14RaisimDepthImgD1Ev why??

  //taken from https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html
  static constexpr double H_FOV = 1.51844; //in rad, 87 deg, D435
  static constexpr double V_FOV = 1.01229; //in rad, 58
  //static constexpr int H_RES = 640; //in pixel
  //static constexpr int V_RES = 480; //in pixel
  static constexpr int H_RES = 2; //in pixel
  static constexpr int V_RES = 2; //in pixel
  
  static constexpr double SENSOR_SIZE = 0.1; //in metre
  static constexpr double FOCAL_LENGTH = 0.2; //in metre
  static constexpr double NEAR_DISTANCE = 0.1; //in metre
  static constexpr double FAR_DISTANCE = 10; //in metre
  static constexpr double OFFSET = 0.05; //in metre
//Function definition

  void setWorld(World* world) {
    world_ = world;
  }

  Mat<H_RES,V_RES> getDepth()
  {
    //===========================================
    //  1. obtain the current camera pose
    //===========================================
    getCameraPose(as_,"base_link_to_realsense_link",camPos_W_,camOri_W_);
    std::cout<<"camPos_W_="<<camPos_W_<<std::endl;

    //===========================================
    //  2. calc ray direction
    //===========================================

    std::vector<Vec<3>> dirs;
    Vec<3> di;
    di.e()<<camPos_W_[0], camPos_W_[1]+OFFSET, camPos_W_[2]+OFFSET;
    dirs.push_back(di);
    di.e()<<camPos_W_[0],camPos_W_[1]+OFFSET,camPos_W_[2]-OFFSET;
    dirs.push_back(di);
    di.e()<<camPos_W_[0],camPos_W_[1]-OFFSET,camPos_W_[2]+OFFSET;
    dirs.push_back(di);
    di.e()<<camPos_W_[0],camPos_W_[1]-OFFSET,camPos_W_[2]-OFFSET;
    dirs.push_back(di);
    dirs_=dirs;


    //===========================================
    //  3. performing ray casting
    //===========================================

    std::vector<RayCollisionList> rays;

    //std::cout<<"performing ray casting"<<std::endl;
    for(auto dir:dirs)
    {
      //std::cout<<i<<std::endl;
      rays.push_back(world_->rayTest(camPos_W_.e(),dir.e(),ray_length_,collisiongroup_, CollisionGroup(-1)));
    }

    //std::cout<<"rays.size()="<<rays.size()<<std::endl;
    // for(auto ray:rays)
    // {
    //   depth_<<ray.geoms[0].depth
    //   for(int i=0;i<ray.size;i++)
    //   {
    //     std::cout<<ray.geoms[i].depth<<std::endl;
    //   }
    // }

    //===========================================
    //  4. Constructing a depth image
    //===========================================
    for (int i=0;i<H_RES;i++) //rows
    {
      for (int j=0;j<V_RES;j++) //
        depth_(i,j) = rays[i*2+j].geoms[0].depth; // [0] is the cloest geoms 
    }
    return depth_;
  }
  Vec<3> getCameraPosition(void) { return camPos_W_; }
  Mat<3,3> getCameraOrientation(void) { return camOri_W_; }
  std::vector<Vec<3>> getDirs(void) {return dirs_; }

//Variables definition


// ======================================================
// private
// ======================================================

 private:


//===========================================
//  Function definition
//===========================================

  void getCameraPose(raisim::ArticulatedSystem* as, std::string jointName, Vec<3> &pos, Mat<3,3> &ori)
  {
    auto frameIndex = as->getFrameByName(jointName); 
    as->getFramePosition(frameIndex, pos);
    as->getFrameOrientation(frameIndex, ori);
  }

//===========================================
//  Variable definition
//===========================================

  raisim::ArticulatedSystem* as_;
  Vec<3> camPos_W_; // w.r.t the world frame
  Mat<3,3> camOri_W_; // w.r.t 
  Mat<H_RES,V_RES> depth_;
  std::vector<Vec<3>> dirs_;
  
  raisim::World* world_;
  CollisionGroup collisiongroup_;
  double ray_length_;

};

}

#endif //RAISIM_RAISIMDEPTHIMG_HPP
