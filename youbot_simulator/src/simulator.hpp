/******************************************************************************
*                    OROCOS Youbot simulator component                        *
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
 * @brief Youbot simulator - OROCOS component
 * @Author: Steven Bellens
 */

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/base/PortInterface.hpp>
#include <rtt/Component.hpp>

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

#include "nonlinearanalyticconditionalgaussianmobile.h"

#include <geometry_msgs/Twist.h>

namespace youbot{

  using namespace std;
  using namespace RTT;
  using namespace BFL;

  class Simulator : public TaskContext{
    protected:
      /// @name Ports
      //@{
      /// Youbot control input
      InputPort<geometry_msgs::Twist> ctrl_port;
      /// Youbot current pose
      OutputPort<ColumnVector> measurement_port;
      //@}
      /// @name Properties
      //@{
      // The level of continuity of the system model
      int m_level;
      /// The mean of the white noise on the system model
      double m_sysNoiseMean;
      /// The covariance of the white noise on the system model
      double m_sysNoiseCovariance;
      /// Matrix for linear measurementModel
      Matrix m_measModelMatrix;
      /// Covariance matrix of additive Gaussian noise on measurement model
      SymmetricMatrix m_measModelCovariance;
      /// The mean of the white noise on the measurement model
      ColumnVector m_measNoiseMean;
      /// The covariance of the white noise on the measurement model
      SymmetricMatrix m_measNoiseCovariance;
      /// The dimension of the state space, only at position level
      int m_posStateDimension;
      /// The dimension of the measurement space
      int m_measDimension;
      /// Period at which the system model gets updated
      double m_period;
      /// The system state: (x,y,theta) for level = 0, ...
      ColumnVector m_state;
      //@}

    public:
      /**
       * \brief Constructor
       *
       * Constructor building a Youbot controller component
       * \param name The component name
       */
      Simulator(std::string name);
      //! Destructor
      ~Simulator();

      /// @name Public methods
      //@{
      bool configureHook();
      bool startHook();
      void updateHook();
      void stopHook();
      void cleanupHook();
      //@}

    private:
      /// The dimension of the state space
      unsigned int m_dimension;
      /// The linear conditional Gaussian underlying the system model
      NonLinearAnalyticConditionalGaussianMobile* m_sysPdf;
      /// The linear system model
      AnalyticSystemModelGaussianUncertainty* m_sysModel;
      /// The linear conditional Gaussian underlying the measurement model
      LinearAnalyticConditionalGaussian* m_measPdf;
      /// The analytic measurement model with addtive Gaussian noise
      AnalyticMeasurementModelGaussianUncertainty* m_measModel;
      /// The control input
      geometry_msgs::Twist m_ctrl_input;
      /// The system state covariance matrix
      SymmetricMatrix m_poseCovariance;
      /// Measurement
      ColumnVector m_measurement;
      /// System inputs
      ColumnVector m_inputs;
      /*!
      * helper function calculating the factorial of an int
      * @param the integer of which to calculate the factorial
      * @return the factorial
      */
      int factorial(int);
  };
}
