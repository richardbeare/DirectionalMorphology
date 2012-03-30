#ifndef __itkDirectionalCloseImageFilter_txx
#define __itkDirectionalCloseImageFilter_txx

#include "itkDirectionalCloseImageFilter.h"
#include "itkProgressAccumulator.h"
#include "itkImageRegionConstIterator.h"
#include <itkVanHerkGilWermanUtilities.h>

namespace itk
{
template<class TInputImage, class TMaskImage, class TVectorImage>
DirectionalCloseImageFilter<TInputImage, TMaskImage, TVectorImage>
::DirectionalCloseImageFilter()
{
  m_LineLength = -1.0;
  m_DilFilterLength = -1.0;
  m_EroFilterLength = -1.0;
  m_UseImageSpacing = true;
  this->SetNumberOfRequiredInputs(2);
  m_TargetIndex.Fill(0);
  m_Scale = 1.0;

}

template<class TInputImage, class TMaskImage, class TVectorImage>
void
DirectionalCloseImageFilter<TInputImage, TMaskImage, TVectorImage>
::GenerateInputRequestedRegion()
{
  // call the superclass' implementation of this method
  Superclass::GenerateInputRequestedRegion();
  
  // get pointers to the inputs
  typename InputImageType::Pointer input  = const_cast< InputImageType * >( this->GetInput());
  typename VectorImageType::Pointer vec = this->GetVectorImage();
  typename MaskImageType::Pointer mask = this->GetMaskImage();

  
  if ( !mask || !input )
    { return; }

  // We need to
  // configure the inputs such that all the data is available.
  //
  mask->SetRequestedRegion(mask->GetLargestPossibleRegion());
  input->SetRequestedRegion(input->GetLargestPossibleRegion());
  if (vec)
    {
    vec->SetRequestedRegion(vec->GetLargestPossibleRegion());
    }
}

template<class TInputImage, class TMaskImage, class TVectorImage>
void
DirectionalCloseImageFilter<TInputImage, TMaskImage, TVectorImage>
::EnlargeOutputRequestedRegion(DataObject *)
{
  this->GetOutput()->SetRequestedRegion( this->GetOutput()->GetLargestPossibleRegion() );
}

template<class TInputImage, class TMaskImage, class TVectorImage>
void
DirectionalCloseImageFilter<TInputImage, TMaskImage, TVectorImage>
::GenerateData()
{

  this->AllocateOutputs();

  if ((m_LineLength < 0) || (m_DilFilterLength < 0) || (m_EroFilterLength < 0)) 
    {
    itkGenericExceptionMacro(<< "Must set line length and filter length" );
    }

  typename OutputImageType::Pointer     output = this->GetOutput();
  typename InputImageType::ConstPointer input  = this->GetInput();
  typename VectorImageType::ConstPointer   vec = this->GetVectorImage();
  typename MaskImageType::ConstPointer    mask = this->GetMaskImage();

  m_Boundary = NumericTraits<PixelType>::NonpositiveMin();
  output->FillBuffer(m_Boundary);

  typename InputImageType::RegionType IReg = input->GetLargestPossibleRegion();

  // iterate over the mask image to find operating points
  typedef typename itk::ImageRegionConstIterator<MaskImageType> MaskIterType;
  typename InputImageType::SpacingType spacing = input->GetSpacing();

  MaskIterType maskIt(mask, mask->GetLargestPossibleRegion());
  
  size_t maxbufflength = 0;
  // max is sum of image sizes in each dimension
  for ( unsigned i = 0; i < InputImageType::ImageDimension; i++ )
    {
    maxbufflength += IReg.GetSize()[i];
    }

  std::vector<PixelType> buffer(maxbufflength);
  std::vector<PixelType> forward(maxbufflength);
  std::vector<PixelType> reverse(maxbufflength);

  BresType BresLine;
  //TFunction1 m_TF;

  for (maskIt.GoToBegin(); !maskIt.IsAtEnd(); ++maskIt)
    {
    if (maskIt.Get())
      {
      // we are going filter along a line starting at this point
      typename MaskImageType::IndexType start = maskIt.GetIndex();
      typename VectorImageType::PixelType orientation;
      if (vec)
	{
	orientation = vec->GetPixel(start);
	}
      else
	{
	// we are using target index
	for (unsigned k = 0; k < MaskImageType::ImageDimension; k++)
	  {
	  orientation[k] = m_TargetIndex[k] - start[k];
	  }
	}
      orientation = m_Scale * orientation;
      orientation.Normalize();
      typename MaskImageType::OffsetType off;
      unsigned DilOpLength = m_DilFilterLength;
      unsigned EroOpLength = m_EroFilterLength;
      if (this->GetUseImageSpacing())
	{
	// figure out the line and operation length in voxels
	// normalize the orientation
	float DilOpLengthFl = 0.;
	float EroOpLengthFl = 0.;
	for (unsigned g = 0; g < InputImageType::ImageDimension ; ++g)
	  {
	  off[g] = static_cast<typename MaskImageType::OffsetType::OffsetValueType>(orientation[g] * m_LineLength / spacing[g]);
	  // figure out the length of the operation, in voxels
	  DilOpLengthFl += vcl_abs(orientation[g]) * m_DilFilterLength / spacing[g];
	  EroOpLengthFl += vcl_abs(orientation[g]) * m_EroFilterLength / spacing[g];
	  }
	DilOpLength = static_cast<unsigned>(DilOpLengthFl);
	EroOpLength = static_cast<unsigned>(EroOpLengthFl);
	// make sure it is an odd number
	if (!(DilOpLength % 2))
	  {
	  ++DilOpLength;
	  }
	if (!(EroOpLength % 2))
	  {
	  ++EroOpLength;
	  }

	}
      else
	{
	for (unsigned g = 0; g < InputImageType::ImageDimension ; ++g)
	  {
	  off[g] = static_cast<typename MaskImageType::OffsetType::OffsetValueType>(orientation[g] * m_LineLength);
	  }

	}
      
      typename MaskImageType::IndexType finish = start + off;
      
      typename BresType::IndexArray BL = BresLine.BuildLine(start, finish);

      // copy the line into buffer
      size_t bufflength = BL.size();

      for (unsigned g = 0; g < BL.size(); ++g)
	{
	if (IReg.IsInside(BL[g])) 
	  {
	  buffer[g+1] = input->GetPixel(BL[g]);
	  }
	else
	  {
	  bufflength=static_cast<size_t>(g);
	  break;
	  }
	}
      // setup the boundary conditions. Starting point is anchored -
      // don't do anything unusual. Far end needs to be "safe", so
      // include a buffer
      unsigned padsize = std::max(DilOpLength, EroOpLength);

      buffer[0] = m_Boundary;
      
      for (unsigned k = bufflength+1; k < bufflength+padsize; k++)
	{
	buffer[k] = m_Boundary;
	}
      unsigned int bsize = bufflength + 1 + padsize;
      FillForwardExt<PixelType , DirectionalClose::MaxFunctor<PixelType> >(buffer, forward, DilOpLength, bsize);
      FillReverseExt<PixelType , DirectionalClose::MaxFunctor<PixelType> >(buffer, reverse, DilOpLength, bsize);

      OneLine<DirectionalClose::MaxFunctor<PixelType> >(bsize, DilOpLength, buffer, forward, reverse, MxFunc);

      buffer[0]=NumericTraits<PixelType>::max();

      // now do the reverse
      FillForwardExt<PixelType , DirectionalClose::MinFunctor<PixelType> >(buffer, forward, EroOpLength, bsize);
      FillReverseExt<PixelType , DirectionalClose::MinFunctor<PixelType> >(buffer, reverse, EroOpLength, bsize);

      OneLine<DirectionalClose::MinFunctor<PixelType> >(bsize, EroOpLength, buffer, forward, reverse, MnFunc);

      // copy buffer to line - need to mask current contents because
      // lines may overlap
      for (unsigned g = 0; g < bufflength; ++g)
	{
       	PixelType current = output->GetPixel(BL[g]);
       	output->SetPixel(BL[g], MxFunc(current, buffer[g+1]));
       	}

      }
    }

}

template<class TInputImage, class TMaskImage, class TVectorImage>
template <class TFunction1> 
void
DirectionalCloseImageFilter<TInputImage, TMaskImage, TVectorImage>
::OneLine(unsigned bsize, unsigned OpLength, std::vector<PixelType> &buffer,
	  std::vector<PixelType> &forward, std::vector<PixelType> &reverse,
	  TFunction1 &TF)
{
      if ( bsize <= OpLength / 2 )
        {
        for ( unsigned j = 0; j < bsize; j++ )
          {
          buffer[j] = forward[bsize - 1];
          }
        }
      else if ( bsize <= OpLength )
        {
        for ( unsigned j = 0; j < bsize - OpLength / 2; j++ )
          {
          buffer[j] = forward[j + OpLength / 2];
          }
        for ( unsigned j =  bsize - OpLength / 2; j <= OpLength / 2; j++ )
          {
          buffer[j] = forward[bsize - 1];
          }
        for ( unsigned j =  OpLength / 2 + 1; j < bsize; j++ )
          {
          buffer[j] = reverse[j - OpLength / 2];
          }
        }
      else
        {
        // line beginning
        for ( unsigned j = 0; j < OpLength / 2; j++ )
          {
          buffer[j] = forward[j + OpLength / 2];
          }
        for ( unsigned j = OpLength / 2, k = OpLength / 2 + OpLength / 2, l = OpLength / 2 - OpLength / 2;
              j < bsize - OpLength / 2; j++, k++, l++ )
          {

	  PixelType V1 = forward[k];
          PixelType V2 = reverse[l];
          buffer[j] = TF(V1, V2);
          }
        // line end -- involves reseting the end of the reverse
        // extreme array
        for ( unsigned j = bsize - 2; ( j > 0 ) && ( j >= ( bsize - OpLength - 1 ) ); j-- )
          {
          reverse[j] = TF(reverse[j + 1], reverse[j]);
          }
        for ( unsigned j = bsize - OpLength / 2; j < bsize; j++ )
          {
          buffer[j] = reverse[j - OpLength / 2];
          }
        }
}

template<class TInputImage, class TMaskImage, class TVectorImage>
void
DirectionalCloseImageFilter<TInputImage, TMaskImage, TVectorImage>
::PrintSelf(std::ostream& os, Indent indent) const
{
  Superclass::PrintSelf(os,indent);
  if (m_UseImageSpacing)
    {
    os << "Line length in world units: " << m_LineLength << std::endl;
    }
  else
    {
    os << "Line length in voxels: " << m_LineLength << std::endl;
    }
}

}

#endif
