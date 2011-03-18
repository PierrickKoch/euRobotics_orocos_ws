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
/* @Description:
 * @brief Youbot controller - OROCOS component
 * @Author: Steven Bellens
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Component.hpp>

#include <kdl/frames.hpp>

#include <bfl/wrappers/rng/rng.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/wrappers/matrix/vector_wrapper.h>

#include <bfl/bfl_constants.h>
#include <bfl/filter/extendedkalmanfilter.h>

#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/discreteconditionalpdf.h>
#include <bfl/pdf/discretepdf.h>
#include <bfl/pdf/gaussian.h>

#include <geometry_msgs/Twist.h>

namespace youbot{

  using namespace std;
  using namespace RTT;
  using namespace BFL;

  class Controller : public TaskContext{
    protected:
      /// @name Ports
      //@{
      /// Youbot current pose
      InputPort<ColumnVector> current_pose_port;
      /// Youbot goal pose
      InputPort<ColumnVector> goal_pose_port;
      /// Youbot control input
      OutputPort<geometry_msgs::Twist> ctrl_port;
      //@}
      /// @name Properties
      //@{
      /// Goal tolerance
      std::vector<double> m_goal_tolerance;
      /// Youbot velocity
      std::vector<double> m_velocity;
      //@}

    public:
      /**
       * \brief Constructor
       *
       * Constructor building a Youbot controller component
       * \param name The component name
       */
      Controller(std::string name);
      //! Destructor
      ~Controller();

      /// @name Public methods
      //@{
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();

      /// Calculate the pose difference (and express it in the Youbot frame)
      void calcPoseDiff();
      /// Update the goal pose
      void goalUpdate(RTT::base::PortInterface* portInterface);
      /// Update the current pose
      void measUpdate(RTT::base::PortInterface* portInterface);
      //@}

    private:
      /// Youbot goal pose
      ColumnVector m_goal_pose;
      /// Youbot current pose
      ColumnVector m_current_pose;
      /// Youbot pose difference
      ColumnVector m_delta_pose;
      /// Youbot ctrl input
      geometry_msgs::Twist m_ctrl;
      /// timer
      RTT::os::TimeService::ticks m_time_begin;
      /// time since last update
      double m_time_passed;
      /// port statuses
      FlowStatus goal_pose_port_status;
      FlowStatus current_pose_port_status;
  };
}
