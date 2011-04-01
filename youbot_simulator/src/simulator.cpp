/******************************************************************************
*                    OROCOS Youbot simulator component                        *
*                                                                             *
*                         (C) 2011 Steven Bellens, Tinne De Laet              *
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
#include "simulator.hpp"

ORO_CREATE_COMPONENT(youbot::Simulator)

namespace youbot{
  Simulator::Simulator(std::string name) : TaskContext(name,PreOperational)
    ,_timerId("TimerId")
    ,m_level(0)
    ,m_posStateDimension(0)
    ,m_measDimension(0)
    ,prop_timer_state(10)
    ,prop_timer_meas(11)
  {
    this->addPort("ctrl",ctrl_port).doc("Youbot control input");
    this->addPort("measurement",measurement_port).doc("Laser measurement output");
    this->addPort("simulatedState",simulatedState_port).doc("Simulated state");
    this->addEventPort(_timerId,boost::bind(&Simulator::triggerTimer,this,_1)).doc("Triggers simulateMeas() when new data arrives");
    this->addProperty("Level", m_level).doc("The level of continuity of the system model: 0 = cte position, 1= cte velocity ,... ");
    this->addProperty("SysNoiseMean", m_sysNoiseMean).doc("The mean of the noise on the marker system model");
    this->addProperty("SysNoiseCovariance", m_sysNoiseCovariance).doc("The covariance of the noise on the marker system model");
    this->addProperty("MeasNoiseMean", m_measNoiseMean).doc("The mean of the noise on the marker measurement model");
    this->addProperty("MeasNoiseCovariance", m_measNoiseCovariance).doc("Covariance matrix of additive Gaussian noise on measurement model");
    this->addProperty("PosStateDimension", m_posStateDimension).doc("The dimension of the state space, only at position level");
    this->addProperty("MeasDimension", m_measDimension).doc("The dimension of the measurement space");
    this->addProperty("Period", m_period).doc("Period at which the system model gets updated");
    this->addProperty("State", m_state).doc("The system state: (x,y,theta) for level = 0, ...");
    this->addProperty("idTimerState", prop_timer_state).doc("The timer id for trigger the state update ");
    this->addProperty("idTimerMeas", prop_timer_meas).doc("The timer id for trigger the meas update ");
  }

  Simulator::~Simulator(){}

  bool Simulator::configureHook(){
#ifndef NDEBUG
    log(Debug) << "(Simulator) ConfigureHook entered" << endlog();
#endif
    if(m_posStateDimension == 0)
    {
      log(Error) << "The dimension of the state space at position level cannot be zero" << endlog();
      return false;
    }
    if(m_measDimension == 0 )
    {
      log(Error) << "The dimension of the measurement space cannot be zero" << endlog();
      return false;
    }
    // dimension of the state
    m_dimension = m_posStateDimension * (m_level+1);
#ifndef NDEBUG
    log(Debug) << "(Simulator) resizing initial mean and covariances" << endlog();
#endif
    m_measNoiseMean.resize(m_measDimension);

    /// Make sure the class variables are of the correct size
    m_poseCovariance.resize(m_dimension);
    m_measurement.resize(m_measDimension);
    m_inputs.resize(4);

     /// make system model: a constant level'th derivative model
     /// state [x, y, theta]
     /// assumption: the 3 components of the youbot state [x,y,theta] are assumed to evolve independently
    ColumnVector sysNoiseVector = ColumnVector(m_level+1);
    sysNoiseVector = 0.0;
    SymmetricMatrix sysNoiseMatrixOne = SymmetricMatrix(m_level +1);
    sysNoiseMatrixOne = 0.0;
    Matrix sysNoiseMatrixNonSymOne = Matrix(m_level+1,m_level+1);
    sysNoiseMatrixNonSymOne = 0.0;
    for(unsigned int i =0 ; i<=m_level; i++)
    {
      sysNoiseVector(i+1) = pow(m_period,m_level-i+1)/double(factorial(m_level-i+1));
    }
    sysNoiseMatrixNonSymOne = (sysNoiseVector * sysNoiseVector.transpose()) * m_sysNoiseCovariance;
    sysNoiseMatrixNonSymOne.convertToSymmetricMatrix(sysNoiseMatrixOne);

    SymmetricMatrix sysNoiseMatrix = SymmetricMatrix(m_dimension);
    sysNoiseMatrix = 0.0;
    for(unsigned int i =0 ; i<=m_level; i++)
    {
      for(unsigned int j =0 ; j<=m_level; j++)
      {
        for (unsigned int k=1 ; k <=m_posStateDimension; k++)
        {
          sysNoiseMatrix(i*m_posStateDimension+k,j*m_posStateDimension+k)=sysNoiseMatrixOne(i+1,j+1);
        }
      }
    }

    ColumnVector sysNoiseMean = ColumnVector(m_dimension);
    sysNoiseMean = m_sysNoiseMean;

    Gaussian system_Uncertainty(sysNoiseMean, sysNoiseMatrix);
    m_sysPdf = new NonLinearAnalyticConditionalGaussianMobile(system_Uncertainty);
    m_sysModel = new AnalyticSystemModelGaussianUncertainty(m_sysPdf);

    if(m_measDimension != m_measNoiseMean.rows() )
    {
        log(Error) << "(Simulator) The size of the measurement (" << m_measDimension << ") does not fit the size of the mean of te mesurement noise (" <<m_measNoiseMean.rows() <<"), measurement model not constructed " << endlog();
        log(Error) << "(Simulator) The size of the measurement does not fit the size of the mean of te mesurement noise, measurement model not constructed " << endlog();
        return false;
    }
    if(m_measDimension != m_measNoiseCovariance.rows() )
    {
        log(Error) << "(Simulator) The size of the measurement (" << m_measDimension << ") does not fit the size of the covariance of te mesurement noise (" <<m_measNoiseCovariance.rows() <<"), measurement model not constructed " << endlog();
        return false;
    }
    Gaussian measurement_Uncertainty(m_measNoiseMean, m_measNoiseCovariance);
    /// The measurement model is non-linear and uses the custom YoubotLaserPdf class
    m_measPdf = new YoubotLaserPdf(measurement_Uncertainty);
    m_measModel = new AnalyticMeasurementModelGaussianUncertainty(m_measPdf);

    simulatedState_port.setDataSample(ColumnVector(m_dimension));
    measurement_port.setDataSample(0.0);
    return true;
  }

  bool Simulator::startHook(){
    return true;
  }

  void Simulator::updateHook(){
  }

  void Simulator::triggerTimer(RTT::base::PortInterface* port){
    int timer_id;
    // Check which timer triggered the port and act accordingly
    _timerId.read(timer_id);
    if( timer_id == prop_timer_state){
      simulateState();
    }
    else if( timer_id == prop_timer_meas ){
      simulateMeas();
    }
  }

  void Simulator::simulateMeas(){
    // Simulate a new measurement and write it out
    m_measurement = m_measModel->Simulate(m_state);
    measurement_port.write(m_measurement(1));
  }

  void Simulator::simulateState(){
    // Read in the current control signals
    ctrl_port.read(m_ctrl_input);
    m_inputs(1) = m_ctrl_input.linear.x;
    m_inputs(2) = m_ctrl_input.linear.y;
    m_inputs(3) = m_ctrl_input.angular.z;
    m_inputs(4) = m_period;
    // Simulate the system one time step
    m_state = m_sysModel->Simulate(m_state,m_inputs);
    // Write out the estimated system state
    simulatedState_port.write(m_state);
  }

  int Simulator::factorial (int num)
  {
    if (num==1)
      return 1;
    return factorial(num-1)*num; // recursive call
  }

  void Simulator::stopHook(){
  }

  void Simulator::cleanupHook(){
  }
}
