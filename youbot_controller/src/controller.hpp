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

 /*
  * The YouBot Controller component receives current pose estimates and a
  * desired goal pose. It will generate control signals in order to steer the YouBot to this goal
  * pose.
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Component.hpp>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

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
#include <geometry_msgs/Pose2D.h>

namespace youbot{

  using namespace std;
  using namespace RTT;
  using namespace BFL;

  class Controller : public TaskContext{
    protected:
      /// @name Ports
      //@{
      /// Youbot current pose - the controller get's this one from the Extended Kalman Filter estimator component
      InputPort<ColumnVector> current_pose_port;
      /// Youbot goal pose - the state machine will send goal poses through this port
      InputPort<geometry_msgs::Pose2D> goal_pose_port;
      /// Youbot control input - generated by the controller, these control signals are send to the YouBot, or a YouBot simulator
      OutputPort<geometry_msgs::Twist> ctrl_port;
      //@}
      /// @name Properties
      //@{
      /// Goal tolerance - whenever the YouBot is closer to it's goal than this offset, it consideres the goal as 'reached'
      std::vector<double> m_goal_tolerance;
      /// Youbot velocity - the YouBot velocity while reaching the goal pose
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

      /**
       * \brief Calculate the pose difference (and express it in the YouBot
       * frame)
       *
       * Calculates the difference between the current pose and the goal pose
       * and expresses this difference in the YouBot frame (X-axis pointing
       * forwards, Y-axis to the left). This transformation uses the estimated
       * angle.
       */
      void calcPoseDiff();
      //@}

    private:
      /// YouBot goal pose [x, y, theta]
      geometry_msgs::Pose2D m_goal_pose;
      /// Youbot current pose [x, y, theta]
      ColumnVector m_current_pose;
      /// Youbot ctrl input
      geometry_msgs::Twist m_ctrl;
      /// Youbot pose difference - expressed in the YouBot frame
      KDL::Vector m_delta_pose;
  };
}
