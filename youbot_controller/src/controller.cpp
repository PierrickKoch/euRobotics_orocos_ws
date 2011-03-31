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

// This macro is used whenever a new Orocos component is generated. Here we
// generate a Youbot Controller component.
ORO_CREATE_COMPONENT(youbot::Controller)

namespace youbot{
  Controller::Controller(std::string name) : TaskContext(name,PreOperational)
  ,m_goal_tolerance(3,0.0)
  ,m_velocity(3,0.0)
  ,m_goal_reached(false)
  {
    /// Add the input and output ports to the Orocos interface
    this->addPort("current_pose",current_pose_port).doc("Youbot pose");
    this->addOperation("moveTo",&Controller::moveTo,this,RTT::OwnThread).doc("Move to goal pose").arg("goal_pose","Goal pose");
    this->addPort("ctrl",ctrl_port).doc("Youbot control input");
    /// Add property variables to the Orocos interface
    this->addProperty("goal_tolerance",m_goal_tolerance).doc("Tolerance on goal pose [x y yaw]");
    this->addProperty("control_velocity",m_velocity).doc("Control velocity");
    this->addProperty("goal_reached",m_goal_reached).doc("Goal reached");
  }

  Controller::~Controller(){}

  bool Controller::configureHook(){
    return true;
  }

  bool Controller::startHook(){
    return true;
  }

  void Controller::updateHook(){
    /// read in current pose and goal pose - notice that whenever one of the
    //ports does not have any data on it, the controller will tell the YouBot
    //not to move.
    if(current_pose_port.read(m_current_pose) != NoData){
      /// calculate the pose difference
      calcPoseDiff();
      m_goal_reached = false;
      // Generate control inputs, but don't do anything for those DOFs that are within the specified tolerance
      // X
      if(abs(m_delta_pose[0]) > m_goal_tolerance[0]){
        if(m_delta_pose[0] > 0) m_ctrl.linear.x = abs(m_velocity[0]);
        else m_ctrl.linear.x = -abs(m_velocity[0]);
      }
      else{
        m_ctrl.linear.x = 0.0;
      }
      // Y
      if(abs(m_delta_pose[1]) > m_goal_tolerance[1]){
        if(m_delta_pose[1] > 0) m_ctrl.linear.y = abs(m_velocity[1]);
        else m_ctrl.linear.y = -abs(m_velocity[1]);
      }
      else{
        m_ctrl.linear.y = 0.0;
      }
      // Theta
      if(abs(m_delta_pose[2]) > m_goal_tolerance[2]){
        if(m_delta_pose[2] > 0) m_ctrl.angular.z = abs(m_velocity[2]);
        else m_ctrl.angular.z = -abs(m_velocity[2]);
      }
      else{
        m_ctrl.angular.z = 0.0;
      }
      if (m_ctrl.linear.x == 0 && m_ctrl.linear.y == 0 && m_ctrl.angular.z ==
        0){
        m_goal_reached = true;
      }
    }
    // One of the input ports did not contain any data, find out which one and
    // signal this to the user
    else{
      log(Debug) << "No current pose received" << endlog();
      // Tell the YouBot not to move
      m_ctrl.linear.x = 0.0;
      m_ctrl.linear.y = 0.0;
      m_ctrl.angular.z = 0.0;
    }
    // Write the control values to the ctrl output port
    ctrl_port.write(m_ctrl);
  }

  void Controller::calcPoseDiff(){
    // Calculate the position difference expressed in the world frame
    KDL::Vector diff_world(m_goal_pose.x - m_current_pose[0], m_goal_pose.y - m_current_pose[1], 0.0);
    // Calculate the YouBot-to-world orientation matrix, based on the angle
    // estimation.
    KDL::Rotation rot = KDL::Rotation::RotZ(m_current_pose[2]);
    // Express the position difference in the YouBot frame
    m_delta_pose = rot.Inverse(diff_world);
    m_delta_pose[2] = m_goal_pose.theta - m_current_pose[2];
  }

  void Controller::moveTo(geometry_msgs::Pose2D goal_pose){
    m_goal_pose = goal_pose;
    m_goal_reached = false;
  }

  void Controller::stopHook(){
  }

  void Controller::cleanupHook(){
  }
}
