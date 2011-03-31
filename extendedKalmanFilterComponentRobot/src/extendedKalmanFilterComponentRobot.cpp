/****************************************************************************** 
* OROCOS Extended Kalman Filter component for estimating the position         *
* off a mobile robot by measuring its distance to a wall at position x=0      * 
*                                                                             *
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
#include "extendedKalmanFilterComponentRobot.hpp"

ORO_CREATE_COMPONENT(ExtendedKalmanFilterComponentRobot)

ExtendedKalmanFilterComponentRobot::ExtendedKalmanFilterComponentRobot(std::string name)
  : TaskContext(name,PreOperational)
  ,_timerId("TimerId")
  ,_inputPort("Input")
  ,_measurementPort("Measurement")
  ,_estimatedStatePort("EstimatedState")
  ,_covarianceStatePort("CovarianceState")
  ,_level(0)
  ,_posStateDimension(0)
  ,_measDimension(0)
  ,_inputColumnVector(4)
{ 
  this->addEventPort(_timerId,boost::bind(&ExtendedKalmanFilterComponentRobot::sysUpdate,this,_1)).doc("Triggers sysUpdate() when new data arrives");
  this->addEventPort(_measurementPort,boost::bind(&ExtendedKalmanFilterComponentRobot::measUpdate,this,_1)).doc("Measurement - this port triggers measUpdate() when new data arrives");
  this->addPort(_inputPort).doc("Input (twist) send to robot ");
  this->addPort(_estimatedStatePort).doc("Estimated state");
  this->addPort(_covarianceStatePort).doc("Covariance of state ");
  this->addProperty("PriorMean", _priorMean).doc("The mean of the prior distribution on the marker state");
  this->addProperty("PriorCovariance", _priorCovariance).doc("The diagonal values of the covariance of the prior distribution: it is assumed that the initial state uncertainties are not coupled");
  this->addProperty("Level", _level).doc("The level of continuity of the system model: 0 = cte position, 1= cte velocity ,... ");
  this->addProperty("SysNoiseMean", _sysNoiseMean).doc("The mean of the noise on the marker system model");
  this->addProperty("SysNoiseCovariance", _sysNoiseCovariance).doc("The covariance of the noise on the marker system model");
  this->addProperty("PosStateDimension", _posStateDimension).doc("The dimension of the state space, only at position level");
  this->addProperty("MeasDimension", _measDimension).doc("The dimension of the measurement space, only for pre-allocation");
  //this->addProperty("MeasModelMatrix", _measModelMatrix).doc("Matrix for linear measurement model");
  this->addProperty("MeasModelCovariance", _measModelCovariance).doc("Covariance matrix of additive Gaussian noise on measurement model");
  this->addProperty("MeasNoiseMean", _measNoiseMean).doc("Mean of additive Gaussian noise on measurement model");
  this->addProperty("Period", _period).doc("Period at which the system model gets updated");
  this->addProperty("TimerIdSystemUpdate", _timerIdSystemUpdate).doc("timerId for the system update");
}

ExtendedKalmanFilterComponentRobot::~ExtendedKalmanFilterComponentRobot(){}

bool ExtendedKalmanFilterComponentRobot::configureHook()
{
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) ConfigureHook entered" << endlog();
#endif
  // dimension of the state
  _dimension = _posStateDimension * (_level+1);
  
  /************************
  * Resize class variables
  ************************/
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) resizing class variables" << endlog();
#endif
  //_priorMean.resize(_dimension);
  //_priorCovariance.resize(_dimension);
  //_measNoiseMean.resize(_measDimension);
  //_measModelCovariance.resize(_measDimension);
  
  _systemState.resize(_dimension);
  _stateCovariance.resize(_dimension);
  _measurement.resize(_measDimension);
  _inputColumnVector=0.0;
  
  /************************
  * Create prior distribution for pose state
  ************************/
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) create prior distribution" << endlog();
#endif
  if((_dimension != _priorMean.rows()) )
  {
      log(Error) << "The size of the prior mean does not fit the dimension of the state " << endlog();
      log(Error) << "The size of the prior mean: " << _priorMean.rows() << endlog();
      log(Error) << "The _dimension : " << _dimension << endlog();
      return false;
  }
  _priorCont.DimensionSet(_dimension); 
  _priorCont.ExpectedValueSet(_priorMean); 
  
  if((_dimension != _priorCovariance.rows())  )
  {
      log(Error) << "The size of the prior covariance does not fit the dimension of the state " << endlog();
      return false;
  }
  SymmetricMatrix _priorCovarianceMatrix(_dimension);
  _priorCovarianceMatrix = 0.0;
  for(int i=1 ; i<=_dimension; i++)
    _priorCovarianceMatrix(i,i) = _priorCovariance(i);
  
  _priorCont.CovarianceSet(_priorCovarianceMatrix); 
  
  /************************
  * Create extended kalman filter  
  ************************/
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) create extended Kalman filter" << endlog();
#endif
  _extendedKalmanFilter = new ExtendedKalmanFilter(&_priorCont);
  
  /************************
  * Make system model: a constant level'th derivative model
  * assumption: the position level components of the state are assumed to evolve independently
  ************************/
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) make system model " << endlog();
#endif
  ColumnVector sysNoiseVector = ColumnVector(_level+1);
  sysNoiseVector = 0.0;
  ColumnVector sysNoiseMean(_dimension);
  sysNoiseMean = _sysNoiseMean;
  SymmetricMatrix sysNoiseMatrixOne = SymmetricMatrix(_level +1);
  sysNoiseMatrixOne = 0.0;
  Matrix sysNoiseMatrixNonSymOne = Matrix(_level+1,_level+1);  
  sysNoiseMatrixNonSymOne = 0.0;
  for(int i =0 ; i<=_level; i++) 
  {
    sysNoiseVector(i+1) = pow(_period,_level-i+1)/double(factorial(_level-i+1));
  }
  sysNoiseMatrixNonSymOne =  (sysNoiseVector* sysNoiseVector.transpose()) * _sysNoiseCovariance ;
  sysNoiseMatrixNonSymOne.convertToSymmetricMatrix(sysNoiseMatrixOne);
  
  SymmetricMatrix sysNoiseMatrix = SymmetricMatrix(_dimension);
  sysNoiseMatrix = 0.0;
  for(int i =0 ; i<=_level; i++) 
  {
    for(int j =0 ; j<=_level; j++)
    {
      for (int k=1 ; k <=_posStateDimension; k++)
      {
        sysNoiseMatrix(i*_posStateDimension+k,j*_posStateDimension+k)=sysNoiseMatrixOne(i+1,j+1);
      }
    }
  }
  
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) create system_Uncertainty " << endlog();
#endif
  Gaussian*  system_Uncertainty = new Gaussian(sysNoiseMean, sysNoiseMatrix);
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) create NonLinearAnalyticConditionalGaussianMobile " << endlog();
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) system_Uncertainty.ExpectedValueGet()"  << system_Uncertainty->ExpectedValueGet()<< endlog();
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) system_Uncertainty.CovarianceGet()"  << system_Uncertainty->CovarianceGet()<< endlog();
#endif
  _sysPdf = new NonLinearAnalyticConditionalGaussianMobile(*system_Uncertainty);
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) create AnalyticSystemModelGaussianUncertainty " << endlog();
#endif
  _sysModel = new AnalyticSystemModelGaussianUncertainty(_sysPdf);
  
  /************************
  * Make measurement model
  ************************/
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) make measurement model " << endlog();
#endif
  //_measModelMatrix.resize(_measDimension,_dimension);
  //_measModelMatrix = 0.0;
  //_measModelMatrix(1,2) = 1.0; // the y position of the robot is the expected distance measurement to the wall
  //_measNoiseMean.resize(_measDimension);
  //if(_measDimension != _measModelMatrix.rows() )
  //{
  //    log(Error) << "The size of the measurement does not fit the size of the measurement model matrix, measurement update not executed " << endlog();
  //    return false;
  //}
  //if(_dimension != _measModelMatrix.columns() )
  //{
  //    log(Error) << "The size of the measurement model matrix does not fit the state dimension, measurement update not executed " << endlog();
  //    return false;
  //}
  if(_measDimension != _measNoiseMean.rows() )
  {
      log(Error) << "The size of the measurement does not fit the size of the mean of te mesurement noise, measurement update not executed " << endlog();
      return false;
  }
  if(_measDimension != _measModelCovariance.rows() )
  {
      log(Error) << "The size of the measurement covariance matrix  does not fit the size of the mean of te mesurement noise, measurement update not executed " << endlog();
      return false;
  }

  Gaussian measurement_Uncertainty(_measNoiseMean, _measModelCovariance);
  //_measPdf = new LinearAnalyticConditionalGaussian(_measModelMatrix,measurement_Uncertainty);
  //_measModel = new LinearAnalyticMeasurementModelGaussianUncertainty(_measPdf);
  _measPdf= new YoubotLaserPdf(measurement_Uncertainty);
  _measModel = new AnalyticMeasurementModelGaussianUncertainty(_measPdf);

#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) startHook() entered" << endlog();
#endif
  // Get estimated position and covariance
  _systemState = _extendedKalmanFilter->PostGet()->ExpectedValueGet();
  _stateCovariance = _extendedKalmanFilter->PostGet()->CovarianceGet();
#ifndef NDEBUG    
  log(Debug) << "_systemState " << _systemState << endlog();
  log(Debug) << "_stateCovariance " << _stateCovariance << endlog();
#endif

#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) configureHook finished " << endlog();
#endif
  return true;
}

bool ExtendedKalmanFilterComponentRobot::startHook()
{
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) startHook() entered" << endlog();
#endif
  // Get estimated position and covariance
  _systemState = _extendedKalmanFilter->PostGet()->ExpectedValueGet();
  _stateCovariance = _extendedKalmanFilter->PostGet()->CovarianceGet();
  _estimatedStatePort.write(_systemState);
  _covarianceStatePort.write(_stateCovariance);
#ifndef NDEBUG    
  log(Debug) << "_systemState " << _systemState << endlog();
  log(Debug) << "_stateCovariance " << _stateCovariance << endlog();
#endif
  
  // start a TimerComponent
#ifndef NDEBUG    
  log(Debug) << "start timer component " <<  endlog();
#endif
  //OperationCaller<bool(RTT::os::Timer::TimerId,RTT::Seconds)> startTimer = this->getPeer("Timer")->provides()->getOperation("startTimer");
  //startTimer(1,_period);
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) startHook() ended" << endlog();
#endif
  
  return true;
}

void ExtendedKalmanFilterComponentRobot::updateHook()
{
}

void ExtendedKalmanFilterComponentRobot::sysUpdate(RTT::base::PortInterface* portInterface)
{
   int timer_id;                                                                                                                                                                                             
   _timerId.read(timer_id);  
   if( timer_id == _timerIdSystemUpdate){ 
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) sysUpdate() entered" << endlog();
#endif
  // Do system update
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) read input" << endlog();
#endif
  _inputPort.read(_input);
  _inputColumnVector(1) = _input.linear.x;
  _inputColumnVector(2) = _input.linear.y;
  _inputColumnVector(3) = _input.angular.z;
  _inputColumnVector(4) = _period;
  // apply current input
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) update filter " << endlog();
#endif
  _extendedKalmanFilter->Update(_sysModel,_inputColumnVector);
  
  // Get estimated pose and covariance
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) get estimated pose and covariance " << endlog();
#endif
  _systemState = _extendedKalmanFilter->PostGet()->ExpectedValueGet();
  _stateCovariance = _extendedKalmanFilter->PostGet()->CovarianceGet();
  
  // write results to port
#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) write results to port" << endlog();
#endif
  _estimatedStatePort.write(_systemState);
  _covarianceStatePort.write(_stateCovariance);

#ifndef NDEBUG    
  log(Debug) << "(ExtendedKalmanFilterComponentRobot) sysUpdate() finished" << endlog();
#endif
   }
}

void ExtendedKalmanFilterComponentRobot::measUpdate(RTT::base::PortInterface* portInterface)
{
#ifndef NDEBUG    
    log(Debug) << "(ExtendedKalmanFilterComponentRobot) measUpdate() entered" << endlog();
#endif
    _measurementPort.read(_measurement);
    //_measModelMatrixPort.read(_measModelMatrix);
    //_measModelCovariancePort.read(_measModelCovariance);
    //_measModelMeanNoisePort.read(_measNoiseMean);
    if(_measurement.rows() != _measDimension )
    {
        log(Error) << "The size of the measurement does not fit the size of the measurement model matrix, measurement update not executed " << endlog();
        log(Error) << "_measurement " << _measurement << endlog();
        //log(Error) << "_measModelMatrix " << _measModelMatrix << endlog();
        return;
    }
    //if(_dimension != _measModelMatrix.rows() )
    //{
    //    log(Error) << "The size of the measurement model matrix does not fit the state dimension, measurement update not executed " << endlog();
    //    return;
    //}
    //if(_measurement.columns() != _measNoiseMean.columns() )
    //{
    //    log(Error) << "The size of the measurement does not fit the size of the mean of te mesurement noise, measurement update not executed " << endlog();
    //    return;
    //}

#ifndef NDEBUG    
    log(Debug) << "(ExtendedKalmanFilterComponentRobot) _measurement: " << _measurement << endlog();
    //log(Debug) << "(ExtendedKalmanFilterComponentRobot) _measModelMatrix: " << _measModelMatrix << endlog();
    log(Debug) << "(ExtendedKalmanFilterComponentRobot) _measModelCovariance: " << _measModelCovariance << endlog();
    log(Debug) << "(ExtendedKalmanFilterComponentRobot) _measNoiseMean: " << _measNoiseMean << endlog();
#endif
    // set the measurement model matrix
    //_measPdf->MatrixSet(0,_measModelMatrix);
    // set the measurement model pdf mean
    //_measPdf->AdditiveNoiseMuSet( _measNoiseMean);
    // set the measurement model pdf covariance
    //_measPdf->AdditiveNoiseSigmaSet( _measModelCovariance);
    // update the pdf of the measurement model
    //_measModel->MeasurementPdfSet( _measPdf );
    // update the filter with the measurement
    bool result = _extendedKalmanFilter->Update(_measModel,_measurement);
    if(!result)
    {
#ifndef NDEBUG    
      log(Error) << "(ExtendedKalmanFilterComponentRobot) updating Kalman Filter failed" << endlog();
#endif
    }
    // Get estimated pose and covariance
    _systemState = _extendedKalmanFilter->PostGet()->ExpectedValueGet();
    _stateCovariance = _extendedKalmanFilter->PostGet()->CovarianceGet();
#ifndef NDEBUG    
    log(Debug) << "_systemState " << _systemState << endlog();
    log(Debug) << "_stateCovariance " << _stateCovariance << endlog();
#endif
  // write results to port
  _estimatedStatePort.write(_systemState);
  _covarianceStatePort.write(_stateCovariance);
  
#ifndef NDEBUG    
    log(Debug) << "(ExtendedKalmanFilterComponentRobot) measUpdate() entered" << endlog();
#endif
}

void ExtendedKalmanFilterComponentRobot::stopHook()
{
}

int ExtendedKalmanFilterComponentRobot::factorial (int num)
{
  if (num==1)
      return 1;
  return factorial(num-1)*num; // recursive call
}

void ExtendedKalmanFilterComponentRobot::cleanUpHook()
{
  delete _extendedKalmanFilter;
  delete _sysPdf;
  delete _sysModel;
  delete _measPdf;
  delete _measModel;
}
