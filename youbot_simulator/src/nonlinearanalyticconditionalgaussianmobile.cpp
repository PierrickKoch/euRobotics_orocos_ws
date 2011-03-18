// $Id: nonlinearanalyticconditionalgaussianmobile.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
// Copyright (C) 2006  Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#include "nonlinearanalyticconditionalgaussianmobile.h"
#include <bfl/wrappers/rng/rng.h> // Wrapper around several rng
                                 // libraries
#define NUMCONDARGUMENTS_MOBILE 2

namespace BFL
{
  using namespace MatrixWrapper;


  NonLinearAnalyticConditionalGaussianMobile::NonLinearAnalyticConditionalGaussianMobile(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_MOBILE)
      , _AB(2)
      , _dimension(additiveNoise.DimensionGet())
      , _period(0.01)
      , _state(_dimension,0.0)
      , _vel(4,0.0)
      , _sysModelInputMatrix(_dimension,4) // vx,vy,omega,period
      , _sysModelMatrix(_dimension,_dimension)
      , _df(_dimension,_dimension)
      , _posStateDimension(3)
      , _level( (_dimension/_posStateDimension) -1.0 )
  {
#ifndef NDEBUG    
  cout << "(NonLinearAnalyticConditionalGaussianMobile) additiveNoise.ExpectedValueGet() " << additiveNoise.ExpectedValueGet() << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) additiveNoise.CovarianceGet() " << additiveNoise.CovarianceGet() << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) enter constructor " << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _dimension " << _dimension << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _posStateDimension " << _posStateDimension << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _level " << _level << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _sysModelInputMatrix " << _sysModelInputMatrix << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _sysModelMatrix " << _sysModelMatrix << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _df " << _df << endl;
#endif
        _df = 0.0;
#ifndef NDEBUG    
  cout << "(NonLinearAnalyticConditionalGaussianMobile) create sysModelMatrix" << endl;
#endif
        _sysModelMatrix = 0.0;
        _sysModelMatrix(1,1) = 1.0;
        _sysModelMatrix(2,2) = 1.0;
        _sysModelMatrix(3,3) = 1.0;
        // only start from acceleration level
        for(int i =2 ; i<=_level; i++) 
        {
          for (int k=1 ; k <=_posStateDimension; k++)
          {
            _sysModelMatrix(i*_posStateDimension + k,i*_posStateDimension + k)= 1.0;
          }
          for(int j =i+1 ; j<=_level; j++)
          {
            for (int k=1 ; k <=_posStateDimension; k++)
            {
              _sysModelMatrix(i*_posStateDimension + k,j*_posStateDimension + k)= pow(_period,j-i)/double(factorial(j-i));
            }
          }
        }
        _AB[0]=_sysModelMatrix;

#ifndef NDEBUG    
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _sysModelMatrix " << _sysModelMatrix << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) create sysModelInputMatrix" << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _state " << _state << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _period " << _period << endl;
#endif
        _sysModelInputMatrix = 0.0;
        //x
        _sysModelInputMatrix(1,1) = cos(_state(3)) * _period;
        _sysModelInputMatrix(1,2) = -sin(_state(3)) * _period;
        //y
        _sysModelInputMatrix(2,1) = sin(_state(3)) * _period;
        _sysModelInputMatrix(2,2) = cos(_state(3)) * _period;
        //omega
        _sysModelInputMatrix(3,3) = _period;
        if(_level>0)
        {
            // vx
            _sysModelInputMatrix(4,1) = cos(_state(3));
            _sysModelInputMatrix(4,2) = -sin(_state(3));
            // vy
            _sysModelInputMatrix(5,1) = sin(_state(3));
            _sysModelInputMatrix(5,2) = cos(_state(3));
            //omega
            _sysModelInputMatrix(6,3) = 1.0;
        }
        _AB[1]=_sysModelInputMatrix;
#ifndef NDEBUG    
  cout << "(NonLinearAnalyticConditionalGaussianMobile) _sysModelInputMatrix " << _sysModelInputMatrix << endl;
  cout << "(NonLinearAnalyticConditionalGaussianMobile) finished constructor " << endl;
#endif
  }


  NonLinearAnalyticConditionalGaussianMobile::~NonLinearAnalyticConditionalGaussianMobile(){}

  ColumnVector NonLinearAnalyticConditionalGaussianMobile::ExpectedValueGet() const
  {
#ifndef NDEBUG    
    cout << "(NonLinearAnalyticConditionalGaussianMobile) ExpectedValueGet() " << endl;
#endif
    _state = ConditionalArgumentGet(0);
    _vel  = ConditionalArgumentGet(1);
    _period = _vel(4);
#ifndef NDEBUG    
    cout << "(NonLinearAnalyticConditionalGaussianMobile) _state " << _state << endl;
    cout << "(NonLinearAnalyticConditionalGaussianMobile) _vel " << _vel << endl;
    cout << "(NonLinearAnalyticConditionalGaussianMobile) _period " << _period << endl;
    cout << "(NonLinearAnalyticConditionalGaussianMobile) fill in _sysModelInputMatrix " <<  endl;
#endif

    //x
    _sysModelInputMatrix(1,1) = cos(_state(3)) * _period;
    _sysModelInputMatrix(1,2) = -sin(_state(3)) * _period;
    //y
    _sysModelInputMatrix(1,1) = sin(_state(3)) * _period;
    _sysModelInputMatrix(1,2) = cos(_state(3)) * _period;
    //omega
    _sysModelInputMatrix(3,3) = _period;
    if(_level>0)
    {
        // vx
        _sysModelInputMatrix(4,1) = cos(_state(3));
        _sysModelInputMatrix(4,2) = -sin(_state(3));
        // vy
        _sysModelInputMatrix(5,1) = sin(_state(3));
        _sysModelInputMatrix(5,2) = cos(_state(3));
        //omega
        _sysModelInputMatrix(6,3) = 1.0;
    }
    _AB[1] = _sysModelInputMatrix;

#ifndef NDEBUG    
    cout << "(NonLinearAnalyticConditionalGaussianMobile) _state before applying system model" << _state << endl;
#endif
    _state = _AB[0] * _state + _AB[1] * _vel;
#ifndef NDEBUG    
    cout << "(NonLinearAnalyticConditionalGaussianMobile) _state after applying system model " << _state << endl;
    cout << "(NonLinearAnalyticConditionalGaussianMobile) AdditiveNoiseMuGet() " << AdditiveNoiseMuGet() << endl;
#endif
    return _state + AdditiveNoiseMuGet();
  }

  Matrix NonLinearAnalyticConditionalGaussianMobile::dfGet(unsigned int i) const
  {
#ifndef NDEBUG    
    cout << "(NonLinearAnalyticConditionalGaussianMobile) dfGet() " << endl;
#endif
    if (i==0)//derivative to the first conditional argument (x)
      {
	    _state = ConditionalArgumentGet(0);
	    _vel = ConditionalArgumentGet(1);
        _period = _vel(4);
#ifndef NDEBUG    
        cout << "(NonLinearAnalyticConditionalGaussianMobile) _state " << _state << endl;
        cout << "(NonLinearAnalyticConditionalGaussianMobile) _vel " << _vel << endl;
        cout << "(NonLinearAnalyticConditionalGaussianMobile) _period " << _period << endl;
        cout << "(NonLinearAnalyticConditionalGaussianMobile) fill in _sysModelInputMatrix " <<  endl;
#endif
        //x
	    _df(1,1)=1;
	    _df(1,2)=0;
	    _df(1,3)=-_vel(1)*sin(_state(3))* _period - _vel(2) * cos(_state(3)) * _period;
        //y
	    _df(2,1)=0;
	    _df(2,2)=1;
	    _df(2,3)=_vel(1)*cos(_state(3)) * _period - _vel(2) * sin(_state(3)) * _period;
        //theta
	    _df(3,1)=0;
	    _df(3,2)=0;
	    _df(3,3)=1;
        if(_level>0)
        {
            //vx
	        _df(4,3)=-_vel(1)*sin(_state(3))- _vel(2) * cos(_state(3)) ;
            //vy
	        _df(5,3)=_vel(1)*cos(_state(3)) - _vel(2) * sin(_state(3)) ;
            //omega
        }
        for(int i =2 ; i<=_level; i++) 
        {
          for (int k=1 ; k <=_posStateDimension; k++)
          {
            _df(i*_posStateDimension + k,i*_posStateDimension + k)= 1.0;
          }
          for(int j =i+1 ; j<=_level; j++)
          {
            for (int k=1 ; k <=_posStateDimension; k++)
            {
              _resultFac = factorial(j-i);
              _df(i*_posStateDimension + k,j*_posStateDimension + k)= pow(_period,j-i)/double(_resultFac);
            }
          }
        }
#ifndef NDEBUG    
        cout << "(NonLinearAnalyticConditionalGaussianMobile) _df" << _df<< endl;
#endif
	return _df;
      }
    else
      {
	if (i >= NumConditionalArgumentsGet())
	  {
	    cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
	    exit(-BFL_ERRMISUSE);
	  }
	else{
	  cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
	  exit(-BFL_ERRMISUSE);
	}
      }
  }
  int NonLinearAnalyticConditionalGaussianMobile::factorial (int num) const
  {
    if (num==1)
        return 1;
    return factorial(num-1)*num; // recursive call
  }

}//namespace BFL

