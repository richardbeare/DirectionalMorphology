#include "ioutils.h"
#include <itkSignedMaurerDistanceMapImageFilter.h>
#include <itkGradientImageFilter.h>
#include <itkCovariantVector.h>
#include <itkGradientRecursiveGaussianImageFilter.h>

#include "itkDirectionalErodeImageFilter.h"
#include "itkDirectionalDilateImageFilter.h"

int main(int argc, char * argv[])
{
  const unsigned dim = 2;

  typedef itk::Image<unsigned char, dim> ImageType;
  typedef itk::Image<float, dim> FloatImageType;
  typedef itk::Image<unsigned char, dim> MaskType;
  typedef itk::Image<itk::CovariantVector<float, ImageType::ImageDimension>, ImageType::ImageDimension  > GradImType;

  ImageType::Pointer raw = readIm<ImageType>(argv[1]);
  MaskType::Pointer  directionmask = readIm<MaskType>(argv[2]);
  MaskType::Pointer  locationmask  = readIm<MaskType>(argv[3]);
  
  // compute distance transform and gradient of direction mask
  typedef itk::SignedMaurerDistanceMapImageFilter<MaskType, FloatImageType> DTType;
  DTType::Pointer DT = DTType::New();
  DT->SetInput(directionmask);
  DT->SquaredDistanceOff();
  DT->InsideIsPositiveOn();

#if 1
  typedef itk::GradientRecursiveGaussianImageFilter<FloatImageType, GradImType> GradFiltType;
  GradFiltType::Pointer gradDT = GradFiltType::New();
  gradDT->SetInput(DT->GetOutput());
  gradDT->SetSigma(5);
#else
  typedef itk::GradientImageFilter<FloatImageType> GradFiltType;
  GradFiltType::Pointer gradDT = GradFiltType::New();
  gradDT->SetInput(DT->GetOutput());
#endif

  typedef itk::DirectionalErodeImageFilter<ImageType, GradImType, MaskType > DirErodeType;
  typedef itk::DirectionalDilateImageFilter<ImageType, GradImType, MaskType > DirDilateType;

  DirErodeType::Pointer DE = DirErodeType::New();
  DE->SetLineLength(150);
  DE->SetFilterLength(20);
  DE->SetUseImageSpacing(true);
  DE->SetInput(raw);
  DE->SetVectorImage(gradDT->GetOutput());
  DE->SetMaskImage(locationmask);

  DirDilateType::Pointer DD = DirDilateType::New();
  DD->SetInput(DE->GetOutput());
  DD->SetVectorImage(gradDT->GetOutput());
  DD->SetMaskImage(locationmask);

  DD->SetLineLength(DE->GetLineLength());
  DD->SetFilterLength(DE->GetFilterLength());
  DD->SetUseImageSpacing(DE->GetUseImageSpacing());
  writeIm<ImageType>(DE->GetOutput(), "/tmp/DE.png");

  writeIm<ImageType>(DD->GetOutput(), argv[4]);
  return EXIT_SUCCESS;
}

