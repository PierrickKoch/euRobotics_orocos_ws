/******************************************************************************
*                    OROCOS Youbot controller component                       *
*                                                                             *
*                         (C) 2011 Steven Bellens                             *
*                     steven.bellens@mech.kuleuven.be                         *
*                    Department of Mechanical Engineering,                    *
*                   Katholieke Universiteit Leuven, Belgium.                  *
*                                                                             *
*       You may redistribute this software and/or modify it under either the  *
*       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
*       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
*       discretion) of the Modified BSD License:                              *
*       Redistribution and use in source and binary forms, with or without    *
*       modification, are permitted provided that the following conditions    *
*       are met:                                                              *
*       1. Redistributions of source code must retain the above copyright     *
*       notice, this list of conditions and the following disclaimer.         *
*       2. Redistributions in binary form must reproduce the above copyright  *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*       3. The name of the author may not be used to endorse or promote       *
*       products derived from this software without specific prior written    *
*       permission.                                                           *
*       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
*       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
*       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
*       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
*       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
*       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
*       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
*       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
*       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
*       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
*       POSSIBILITY OF SUCH DAMAGE.                                           *
*                                                                             *
*******************************************************************************/

#include "controller.hpp"

ORO_CREATE_COMPONENT(youbot::Controller)

namespace youbot{
  Controller::Controller(std::string name) : TaskContext(name,PreOperational)
  ,m_goal_tolerance(3,0.0)
  ,m_velocity(3,0.0)
  ,m_time_passed(0.0)
  {
    this->addPort("current_pose",current_pose_port).doc("Youbot pose");
    this->addPort("goal_pose",goal_pose_port).doc("Youbot goal pose");
    this->addPort("ctrl",ctrl_port).doc("Youbot control input");
    this->addProperty("goal_tolerance",m_goal_tolerance).doc("Tolerance on goal pose [x y yaw]");
    this->addProperty("control_velocity",m_velocity).doc("Control velocity");
    this->addProperty("current_pose",m_current_pose).doc("Current pose");
  }

  Controller::~Controller(){}

  bool Controller::configureHook(){
    m_delta_pose.resize(3);
    return true;
  }

  bool Controller::startHook(){
    m_time_begin = RTT::os::TimeService::Instance()->getTicks();
    return true;
  }

  void Controller::updateHook(){
    /// get the time since the last update
    m_time_passed = RTT::os::TimeService::Instance()->secondsSince(m_time_begin);
    /// read in current pose and goal pose
    goal_pose_port_status = goal_pose_port.read(m_goal_pose);
    current_pose_port_status = current_pose_port.read(m_current_pose);

    if(goal_pose_port_status != NoData && current_pose_port_status != NoData){
      /// calculate the pose difference
      calcPoseDiff();
      // generate control inputs
      // don't do anything for those DOFs that are within the specified tolerance
      if(abs(m_delta_pose[0]) > m_goal_tolerance[0]){
        if(m_delta_pose[0] > 0) m_ctrl.linear.x = -abs(m_velocity[0]);
        else m_ctrl.linear.x = abs(m_velocity[0]);
      }
      else{
        m_ctrl.linear.x = 0.0;
      }
      if(abs(m_delta_pose[1]) > m_goal_tolerance[1]){
        if(m_delta_pose[1] > 0) m_ctrl.linear.y = -abs(m_velocity[1]);
        else m_ctrl.linear.y = abs(m_velocity[1]);
      }
      else{
        m_ctrl.linear.y = 0.0;
      }
      if(abs(m_delta_pose[2]) > m_goal_tolerance[2]){
        if(m_delta_pose[2] > 0) m_ctrl.angular.z = abs(m_velocity[2]);
        else m_ctrl.angular.z = -abs(m_velocity[2]);
      }
      else{
        m_ctrl.angular.z = 0.0;
      }
    }
    else{
      if(goal_pose_port_status == NoData){
        log(Debug) << "No goal pose received" << endlog();
      }
      if(current_pose_port_status == NoData){
        log(Debug) << "No pose estimate received" << endlog();
      }
      m_ctrl.linear.x = 0.0;
      m_ctrl.linear.y = 0.0;
      m_ctrl.linear.z = 0.0;
      m_ctrl.angular.z = 0.0;
    }
    // write out
    ctrl_port.write(m_ctrl);
    m_time_begin = RTT::os::TimeService::Instance()->getTicks();
  }

  void Controller::calcPoseDiff(){
    // position difference expressed in the world frame
    KDL::Vector diff_world(m_goal_pose.x - m_current_pose[0], m_goal_pose.y - m_current_pose[1], 0.0);
    // youbot to world orientation matrix
    KDL::Rotation rot = KDL::Rotation::RotZ(m_current_pose[2]);
    // transform to position difference expressed in the Youbot frame
    KDL::Vector diff_youbot = rot.Inverse(diff_world);
    m_delta_pose[0] = diff_youbot[0];
    m_delta_pose[1] = diff_youbot[1];
    m_delta_pose[2] = m_goal_pose.theta - m_current_pose[2];
    log(Info) << "orientation matrix " << rot << endlog();
    log(Info) << "delta_pose " << m_delta_pose << endlog();
  }

  void Controller::stopHook(){
  }

  void Controller::cleanupHook(){
  }
}
