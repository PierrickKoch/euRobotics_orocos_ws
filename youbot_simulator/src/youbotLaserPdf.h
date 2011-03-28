#ifndef __YOUBOTLASERPDF__
#define __YOUBOTLASERPDF__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
  class YoubotLaserPdf : public AnalyticConditionalGaussianAdditiveNoise
    {
    public:
      YoubotLaserPdf (const Gaussian& additiveNoise);
      virtual ~YoubotLaserPdf();

      // redefine virtual functions
      virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;
      virtual ColumnVector ExpectedValueGet() const;
      virtual Matrix dfGet(unsigned int i) const;

    protected:
      mutable ColumnVector m_state;
      mutable ColumnVector m_input;
      mutable Matrix m_df;
      Gaussian m_additiveNoise;
    };
} // End namespace BFL
#endif //
