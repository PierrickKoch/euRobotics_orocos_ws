/****************************************************************************** 
* OROCOS Extended Kalman Filter component for estimating the position         *
* off a mobile robot by measuring its distance to a wall at position x=0      * 
*                                                                             *
*                           (C) 2011 Tinne De Laet                            *
*                       tinne.delaet@mech.kuleuven.be                         *                              
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
/* @Description: OROCOS component which estimates the position of a robot by measuring the distance to a wall (x=0)
 * using BFL (Bayesian Filter Library) Extended Kalman Filter.
 *
 * @Author: Tinne De Laet
 */
#ifndef _EKF_COMPONENT_ROBOT_
#define _EKF_COMPONENT_ROBOT_

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

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/base/PortInterface.hpp>
#include <rtt/os/Timer.hpp>

#include <ocl/Component.hpp>

#include <geometry_msgs/Twist.h>

#include "nonlinearanalyticconditionalgaussianmobile.h"
#include "youbotLaserPdf.h"


using namespace std;
using namespace BFL;
using namespace OCL;
using namespace RTT;

class ExtendedKalmanFilterComponentRobot : public TaskContext
  {
    protected:
      /*********
      PORTS
      *********/
      /// The sysUpdate method gets triggered each time the OCL::TimerComponent
      //fires an event on this port
      InputPort< RTT::os::Timer::TimerId >      _timerId;
      /// The input (velocity send to te robot)
      //InputPort< ColumnVector >                 _inputPort;
      InputPort<geometry_msgs::Twist>           _inputPort;
      /// The measurement
      InputPort< ColumnVector >                 _measurementPort;
      /// The estimated state
      OutputPort< ColumnVector >                _estimatedStatePort;
      /// The covariance on the estimated state
      OutputPort< SymmetricMatrix>              _covarianceStatePort;

      /*********
      PROPERTIES
      *********/
      /// The estimated initial state
      ColumnVector            _priorMean;
      /// ColumnVector containting the diagonal values of the initial covariances : it is assumed that the initial estimates are uncoupled (i.e. the initial covariance matrix is diagonal) 
      ColumnVector            _priorCovariance;
      // The level of continuity of the system model 
      int                     _level;
      /// The mean of the white noise on the system model
      double                  _sysNoiseMean;
      /// The covariance of the white noise on the system model
      double                  _sysNoiseCovariance;
      /// Matrix for linear measurementModel
      //Matrix                   _measModelMatrix;
      /// Covariance matrix of additive Gaussian noise on measurement model
      SymmetricMatrix          _measModelCovariance;
      /// Mean of additive Gaussian noise on measurement model
      ColumnVector              _measNoiseMean;

    public:
      /*!
       * \brief Constructor
       * 
       * Constructor building a ExtendedKalmanFilterComponentRobot component
       * \param name the component name
      */
      ExtendedKalmanFilterComponentRobot(std::string name);
    
      //! Destructor
      ~ExtendedKalmanFilterComponentRobot();
      
      bool      configureHook();
      bool      startHook();
      void      updateHook();
      void      stopHook();
      void      cleanUpHook();
    
    private:
      /// The dimension of the state space
      int                                                     _dimension;
      /// The dimension of the state space, only at position level 
      int                                                     _posStateDimension;
      /// The dimension of the measurement space
      int                                                     _measDimension;
      /// Gaussian to represent the initial state
      Gaussian                                                _priorCont; 
      /// EKF to estimate the state
      ExtendedKalmanFilter*                                   _extendedKalmanFilter; 
      /// The conditional Gaussian underlying the system model
      NonLinearAnalyticConditionalGaussianMobile*             _sysPdf;
      /// The system model 
      AnalyticSystemModelGaussianUncertainty*                 _sysModel;
      /// The linear conditional Gaussian underlying the measurement model
      //LinearAnalyticConditionalGaussian*                      _measPdf; 
      AnalyticConditionalGaussian*                              _measPdf; 
      /// The analytic measurement model with addtive Gaussian noise
      AnalyticMeasurementModelGaussianUncertainty*            _measModel; 
      /// The system state: (Fx,Fy,Fz,wz,wy,wz) for level = 0, ...
      ColumnVector                                            _systemState; 
      /// The system state covariance matrix
      SymmetricMatrix                                         _stateCovariance; 
      /// Measurement 
      ColumnVector                                            _measurement;
      /// Matrix to store the covariance
      SymmetricMatrix                                         _mat;
      /// ColumnVector to store the state
      ColumnVector                                            _state;
      /// Period at which the system model gets updated
      double                                                  _period;
      /// helper variable to store input
      geometry_msgs::Twist                                    _input;
      /// helper variable to store input
      ColumnVector                                            _inputColumnVector;
      
      /*!
      * helper function calculating the factorial of an int 
      * @param the integer of which to calculate the factorial
      * @return the factorial
      */
      int factorial(int);

      /*!
       * update the measurement model each time new data arrives
       */
      void measUpdate(RTT::base::PortInterface*);

      /*!
       * update the system model
       */
      void sysUpdate(RTT::base::PortInterface*);
  };
#endif // _EKF_COMPONENT_ROBOT_

