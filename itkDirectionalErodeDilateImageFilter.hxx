#ifndef __itkDirectionalGradientImageFilter_txx
#define __itkDirectionalGradientImageFilter_txx

#include "itkDirectionalMorphologyImageFilter.h"
#include "itkProgressAccumulator.h"
#include "itkImageRegionConstIterator.h"

namespace itk
{
template<class TInputImage, class TVectorImage, class TMaskImage, class TOutputImage>
DirectionalErodeDilateImageFilter<TInputImage, TVectorImage, TMaskImage, TOutputImage>
::DirectionalErodeDilateImageFilter()
{
  m_LineLength = -1.0;
  m_FilterLength = -1.0;
  m_UseImageSpacing = true;
}

template<class TInputImage, class TVectorImage, class TMaskImage, class TOutputImage>
DirectionalErodeDilateImageFilter<TInputImage, TVectorImage, TMaskImage, TOutputImage>
::GenerateInputRequestedRegion()
{
  // call the superclass' implementation of this method
  Superclass::GenerateInputRequestedRegion();
  
  // get pointers to the inputs
  typename InputImageType::Pointer input  = const_cast< InputImageType * >( this->GetInput());
  typename VectorImageType::Pointer vec = this->GetVectorImage();
  typename MaskImageType::Pointer mask = this->GetMaskImage();

  
  if ( !mask || !input || !vec )
    { return; }

  // We need to
  // configure the inputs such that all the data is available.
  //
  mask->SetRequestedRegion(mask->GetLargestPossibleRegion());
  input->SetRequestedRegion(input->GetLargestPossibleRegion());
  vec->SetRequestedRegion(vec->GetLargestPossibleRegion());
}

template<class TInputImage, class TVectorImage, class TMaskImage, class TOutputImage>
DirectionalErodeDilateImageFilter<TInputImage, TVectorImage, TMaskImage, TOutputImage>
::EnlargeOutputRequestedRegion(DataObject *)
{
  this->GetOutput()->SetRequestedRegion( this->GetOutput()->GetLargestPossibleRegion() );
}

template<class TInputImage, class TVectorImage, class TMaskImage, class TOutputImage>
void
DirectionalErodeDilateImageFilter<TInputImage, TVectorImage, TMaskImage, TOutputImage>
::GenerateData()
{

  typename InputImageType::ConstPointer input  = this->GetInput();
  typename VectorImageType::ConstPointer vec = this->GetVectorImage();
  typename MaskImageType::ConstPointer mask = this->GetMaskImage();

  typename InputImageType::RegionType IReg = input->GetLargestPossibleRegion();

  // iterate over the mask image to find operating points
  typedef typename itk::ImageRegionConstIterator<MaskImageType> MaskIterType;

  MaskIterType maskIt(mask, mask->GetLargestPossibleRegion());
  
  size_t bufflength = 0;
  // max is sum of image sizes in each dimension
  for ( unsigned i = 0; i < TImage::ImageDimension; i++ )
    {
    bufflength += IReg.GetSize()[i];
    }

  std::vector<InputImagePixelType> buffer(bufflength);
  std::vector<InputImagePixelType> forward(bufflength);
  std::vector<InputImagePixelType> reverse(bufflength);

  for (maskIt.GoToBegin(); !maskIt.IsAtEnd(); ++maskIt)
    {
    if (maskIt.Get())
      {
      // we are going filter along a line starting at this point
      typename MaskImageType::IndexType start = maskIt.GetIndex();
      typename VectorImageType::PixelType orientation = vec->GetIndex(start);
      if (this->GetUseImageSpacing())
	{
	// figure out the line and operation length in voxels
	// normalize the orientation
	}
      
      typename MaskImageType::IndexType finish;

      typename BresType::IndexArray BL = BresType::BuildLine(start, finish);
      // copy the line into buffer
      for (
      }
    }

}


}

#endif
