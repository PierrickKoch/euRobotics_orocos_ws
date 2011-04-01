/****************************************************************************** 
* OROCOS component for calculating the distance to a wall                     *
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
#include "calculateDistanceToWall.hpp"

ORO_CREATE_COMPONENT(CalculateDistanceToWall)

CalculateDistanceToWall::CalculateDistanceToWall(std::string name)
  : TaskContext(name,PreOperational)
  ,_laserScanPort("LaserScan")
  ,_distanceToWallPort("DistanceToWall")
{ 
  this->addEventPort(_laserScanPort,boost::bind(&CalculateDistanceToWall::calculateDistance,this,_1)).doc("Triggers calcualteDistanceToWall() when new laser scan data arrives");
  this->addPort(_distanceToWallPort).doc("Calculated distance to wall");
}

CalculateDistanceToWall::~CalculateDistanceToWall(){}

bool CalculateDistanceToWall::configureHook()
{
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) ConfigureHook entered" << endlog();
#endif
  if(!this->hasPeer("rtt_tf"))
  {
    log(Error) << "(CalculateDistanceToWall) component has no peer rtt_tf " << endlog();
    return false;
  }
  if(!(this->getPeer("rtt_tf")->operations()->hasMember("lookupTransform")) )
  {
    log(Error) << "(CalculateDistanceToWall) component peer rtt_tf has no operation lookupTransform " << endlog();
    return false;
  }
  lookupTransform = this->getPeer("rtt_tf")->provides()->getOperation("lookupTransform");
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) configureHook finished " << endlog();
#endif
  return true;
}

bool CalculateDistanceToWall::startHook()
{
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) startHook() entered" << endlog();
#endif
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) startHook() ended" << endlog();
#endif
  
  return true;
}

void CalculateDistanceToWall::updateHook()
{
}


void CalculateDistanceToWall::stopHook()
{
}

void CalculateDistanceToWall::calculateDistance(RTT::base::PortInterface* portInterface)
{
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) calculateDistance() entered " << endlog();
#endif
   _laserScanPort.read(_laserScan); 
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) _laserScan "<< _laserScan << endlog();
  log(Debug) << "(CalculateDistanceToWall) _laserScan "<< _laserScan.angle_min << endlog();
  log(Debug) << "(CalculateDistanceToWall) _laserScan "<< _laserScan.angle_max << endlog();
  log(Debug) << "(CalculateDistanceToWall) _laserScan "<< _laserScan.angle_increment << endlog();
  log(Debug) << "(CalculateDistanceToWall) lookupTransform of laser wrt world" << endlog();
#endif
  _transformLaserWorld = lookupTransform("/laser","/world");
  double yaw = tf::getYaw(_transformLaserWorld.transform.rotation);
  int laser_number = (int)((yaw-_laserScan.angle_min)/_laserScan.angle_increment);
  double distance_measurement=_laserScan.ranges[laser_number];
  _distanceToWall=distance_measurement;
  _distanceToWallPort.write(_distanceToWall);
#ifndef NDEBUG    
  log(Debug) << "(CalculateDistanceToWall) Transform of laser wrt world " << _transformLaserWorld << endlog();
  log(Debug) << "(CalculateDistanceToWall) yaw of laser wrt world (degrees)" << yaw*180/3.14<< endlog();
  log(Debug) << "(CalculateDistanceToWall) laser_number " << laser_number<< endlog();
  log(Debug) << "(CalculateDistanceToWall) _distanceToWall " << _distanceToWall<< endlog();
  log(Debug) << "(CalculateDistanceToWall) calculateDistance() finished " << endlog();
#endif
}

void CalculateDistanceToWall::cleanUpHook()
{
}
