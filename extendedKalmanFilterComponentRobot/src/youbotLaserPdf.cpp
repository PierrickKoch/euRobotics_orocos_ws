#include "youbotLaserPdf.h"
#include <bfl/wrappers/rng/rng.h>

#define YOUBOT_BASE_LASER_DISTANCE 0.25
#define NUMCONDARGUMENTS_YOUBOT 1

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
    m_expected_measurement(1);
  }


  YoubotLaserPdf::~YoubotLaserPdf(){}

  Probability YoubotLaserPdf::ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const
  {
    m_state = ConditionalArgumentGet(0);
#ifndef NDEBUG    
    cout << "(YoubotLaserPdf - ProbabilityGet()) m_state: " << m_state << endl;
#endif
    m_expected_measurement(1) = m_state(2) + YOUBOT_BASE_LASER_DISTANCE * sin(m_state(3));
#ifndef NDEBUG    
    cout << "(YoubotLaserPdf - ProbabilityGet()) m_expected_measurement: " << m_expected_measurement << endl;
#endif
    return m_additiveNoise.ProbabilityGet(m_expected_measurement - measurement);
  }

  ColumnVector YoubotLaserPdf::ExpectedValueGet() const
  {
    m_state = ConditionalArgumentGet(0);
#ifndef NDEBUG    
     cout << "(YoubotLaserPdf - ExpectedValueGet()) m_state: " << m_state << endl;
#endif
    ColumnVector measurement(1);
    measurement(1) = m_state(2) + YOUBOT_BASE_LASER_DISTANCE * sin(m_state(3));
#ifndef NDEBUG    
     cout << "(YoubotLaserPdf - ExpectedValueGet()) expected measurement: " << measurement << endl;
#endif
    return measurement;
  }

  Matrix YoubotLaserPdf::dfGet(unsigned int i) const
  {
    m_state = ConditionalArgumentGet(0);
#ifndef NDEBUG    
    cout << "(YoubotLaserPdf - dfGet()) m_state: " << m_state << endl;
#endif
    m_df.resize(1,m_state.rows() );
    m_df(1,1) = 0.0;
    m_df(1,2) = 1.0;
    m_df(1,3) = YOUBOT_BASE_LASER_DISTANCE * cos(m_state(3));
#ifndef NDEBUG    
    cout << "(YoubotLaserPdf - dfGet()) m_df: " << m_df << endl;
#endif
    return m_df;
  }
}//namespace BFL

