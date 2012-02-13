#ifndef __itkDirectionalGradientImageFilter_txx
#define __itkDirectionalGradientImageFilter_txx

#include "itkDirectionalMorphologyImageFilter.h"
#include "itkProgressAccumulator.h"
#include "itkCropImageFilter.h"
#include "itkConstantPadImageFilter.h"

namespace itk
{
template<class TInputImage, class TMaskImage, class TOutputImage>
DirectionalGradientImageFilter<TInputImage, TMaskImage, TOutputImage>
::DirectionalGradientImageFilter()
{
  m_Length = -1.0;
}


template<class TInputImage, class TMaskImage, class TOutputImage>
void
DirectionalGradientImageFilter<TInputImage, TMaskImage, TOutputImage>
::GenerateData()
{

  typename InputImageType::ConstPointer input  = this->GetInput();
  typename VectorImageType::ConstPointer vec = this->GetVectorImage();

  // construct mini pipeline
  if (m_Pad)
    {
      typename InputImageType::SizeType Pad;
      Pad.Fill(1);
      m_Padder->SetPadLowerBound( Pad);
      m_Padder->SetPadUpperBound( Pad );
      m_Padder->SetConstant( 0 );
      m_Padder->SetInput( mask );

      m_DT->SetInput(m_Padder->GetOutput());
      m_DT->SetOutsideValue(m_OutsideValue);

      m_Cropper->SetInput( m_DT->GetOutput() );
      m_Cropper->SetUpperBoundaryCropSize( Pad );
      m_Cropper->SetLowerBoundaryCropSize( Pad );

      m_GradDT->SetInput(m_Cropper->GetOutput());

    }
  else
    {
    m_DT->SetInput(mask);
    m_DT->SetOutsideValue(m_OutsideValue);
    m_GradDT->SetInput(m_DT->GetOutput());
    }

  m_RawGrad->SetInput(input);
  m_RawGrad->SetSigma(m_Sigma);

  m_Innerprod->SetInput(m_GradDT->GetOutput());
  m_Innerprod->SetInput2(m_RawGrad->GetOutput());
  m_Innerprod->SetScale(m_Scale);

  progress->RegisterInternalFilter(m_DT, 0.2f);
  progress->RegisterInternalFilter(m_GradDT, 0.2f);
  progress->RegisterInternalFilter(m_RawGrad, 0.4f);
  progress->RegisterInternalFilter(m_Innerprod, 0.2f);
  
  m_Innerprod->GraftOutput(this->GetOutput());
  m_Innerprod->Update();
  this->GraftOutput(m_Innerprod->GetOutput());
}


}

#endif
