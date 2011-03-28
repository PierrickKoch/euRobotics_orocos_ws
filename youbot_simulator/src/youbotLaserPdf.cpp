#include "youbotLaserPdf.h"
#include <bfl/wrappers/rng/rng.h>

#define YOUBOT_BASE_LASER_DISTANCE 0.25
#define NUMCONDARGUMENTS_YOUBOT 2

namespace BFL
{
  using namespace MatrixWrapper;

  YoubotLaserPdf::YoubotLaserPdf(const Gaussian& additiveNoise)
    : AnalyticConditionalGaussianAdditiveNoise(additiveNoise,NUMCONDARGUMENTS_YOUBOT)
    ,m_state(additiveNoise.DimensionGet(),0.0)
    ,m_input(4,0.0)
    ,m_df(additiveNoise.DimensionGet(),additiveNoise.DimensionGet())
  {
    m_additiveNoise = additiveNoise;
  }


  YoubotLaserPdf::~YoubotLaserPdf(){}

  Probability YoubotLaserPdf::ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
  {
    m_state = ConditionalArgumentGet(0);
    m_input  = ConditionalArgumentGet(1);
    ColumnVector expected_measurement(1);
    expected_measurement(1) = m_state(2) + YOUBOT_BASE_LASER_DISTANCE * sin(m_state(3));
    return m_additiveNoise.ProbabilityGet(expected_measurement - measurement);
  }

  ColumnVector YoubotLaserPdf::ExpectedValueGet() const
  {
    m_state = ConditionalArgumentGet(0);
    m_input  = ConditionalArgumentGet(1);
    return m_state(2) + YOUBOT_BASE_LASER_DISTANCE * sin(m_state(3));
  }

  Matrix YoubotLaserPdf::dfGet(unsigned int i) const
  {
    m_state = ConditionalArgumentGet(0);
    m_input  = ConditionalArgumentGet(1);
    m_df(1,1) = 0.0;
    m_df(1,2) = 1.0;
    m_df(1,3) = YOUBOT_BASE_LASER_DISTANCE * cos(m_state(3));
    return m_df;
  }
}//namespace BFL

