/****************************************************************************** 
* OROCOS component for calculating the                                        *
* distance to a wall from a laser scan                                        *  
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
/* @Description: OROCOS component which calculates the distance to a wall from a laser scan
 *
 * @Author: Tinne De Laet
 */
#ifndef _CALCULATE_DISTANCE_TO_WALL_
#define _CALCULATE_DISTANCE_TO_WALL_

#include <rtt/RTT.hpp>                                                                                                                                                                                         
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>
#include <rtt/base/PortInterface.hpp>
#include <rtt/os/Timer.hpp>

#include <ocl/Component.hpp>

#include <bfl/wrappers/rng/rng.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/wrappers/matrix/vector_wrapper.h>

#include <tf/tf.h>                                                                                                                                                                                             
#include <tf/transform_datatypes.h>                                                                                                                                                                                             
#include <tf/tfMessage.h>    
#include <sensor_msgs/LaserScan.h>    
#include <std_msgs/Float64.h>    

using namespace std;
using namespace BFL;
using namespace OCL;
using namespace RTT;

class CalculateDistanceToWall : public TaskContext
  {
    protected:
      /*********
      PORTS
      *********/
      /// The measured laser scan 
      InputPort< sensor_msgs::LaserScan >       _laserScanPort;
      /// The calculated distance to the wall
      OutputPort< std_msgs::Float64>            _distanceToWallPort;

      OperationCaller<geometry_msgs::TransformStamped(const std::string&,const std::string&)> lookupTransform;

      /*********
      PROPERTIES
      *********/

    public:
      /*!
       * \brief Constructor
       * 
       * Constructor building a ExtendedKalmanFilterComponentRobot component
       * \param name the component name
      */
      CalculateDistanceToWall(std::string name);
    
      //! Destructor
      ~CalculateDistanceToWall();
      
      bool      configureHook();
      bool      startHook();
      void      updateHook();
      void      stopHook();
      void      cleanUpHook();
    
    private:
      geometry_msgs::TransformStamped   _transformLaserWorld; 
      sensor_msgs::LaserScan            _laserScan;
      std_msgs::Float64                 _distanceToWall;
      /*!
       * calculate distance to wall
       */
      void      calculateDistance(RTT::base::PortInterface*);

  };
#endif // _CALCULATE_DISTANCE_TO_WALL_

