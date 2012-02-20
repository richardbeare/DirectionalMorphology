#ifndef __itkDirectionalErodeImageFilter_h
#define __itkDirectionalErodeImageFilter_h

#include "itkDirectionalErodeDilateImageFilter.h"
#include "itkNumericTraits.h"

namespace itk
{
/**
 * \class DirectionalErodeImageFilter
 * \brief Parent class for morphological operations along lines defined
 * by a vector field. Operations applied whereever the mask image is
 * non zero.
 *
 *
 * \author Richard Beare, Department of Medicine, Monash University,
 * Australia.  <Richard.Beare@monash.edu>
 *
**/

template< class TPixel >
class MinFunctor
{
public:
  MinFunctor(){}
  ~MinFunctor(){}
  inline TPixel operator()(const TPixel & A, const TPixel & B) const
  {
    return vnl_math_min(A, B);
  }
};

template <typename TInputImage,
	  typename TVectorImage,
	  typename TMaskImage
	   >
class ITK_EXPORT DirectionalErodeImageFilter:
    public DirectionalErodeDilateImageFilter<TInputImage,TVectorImage,TMaskImage, MinFunctor<typename TInputImage::PixelType> >
{

public:
  /** Standard class typedefs. */
  typedef DirectionalErodeImageFilter  Self;
  typedef DirectionalErodeDilateImageFilter<TInputImage,TVectorImage,TMaskImage, MinFunctor<typename TInputImage::PixelType> > Superclass;
  typedef SmartPointer<Self>                   Pointer;
  typedef SmartPointer<const Self>        ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Runtime information support. */
  itkTypeMacro(DirectionalErodeImageFilter, DirectionalErodeDilateImageFilter);

  /** Pixel Type of the input image */
  typedef TInputImage                                    InputImageType;
  typedef TInputImage                                   OutputImageType;
  typedef TVectorImage                                   VectorImageType;
  typedef TMaskImage                                     MaskImageType;

  typedef typename TInputImage::PixelType                PixelType;
  typedef typename NumericTraits<PixelType>::RealType    RealType;
  typedef typename NumericTraits<PixelType>::ScalarRealType ScalarRealType;
  typedef typename OutputImageType::PixelType  OutputPixelType;

  /** Smart pointer typedef support.  */
  typedef typename TInputImage::Pointer        InputImagePointer;
  typedef typename TInputImage::ConstPointer   InputImageConstPointer;
  typedef typename TInputImage::SizeType       InputSizeType;
  typedef typename OutputImageType::SizeType   OutputSizeType;

  typedef typename OutputImageType::IndexType  OutputIndexType;

  /** Image dimension. */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);



#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(SameDimension,
                  (Concept::SameDimension<itkGetStaticConstMacro(InputImageDimension),itkGetStaticConstMacro(OutputImageDimension)>));

  itkConceptMacro(Comparable,
		  (Concept::Comparable<PixelType>));

  /** End concept checking */
#endif

protected:
  DirectionalErodeImageFilter()
    {
    this->m_Boundary = NumericTraits<PixelType>::max();
    }
  virtual ~DirectionalErodeImageFilter() {};
  void PrintSelf(std::ostream& os, Indent indent) const
  {
    Superclass::PrintSelf(os, indent);
    os << "Line Erosion" << std::endl;
  }
  

private:
  DirectionalErodeImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented


};

} // end namespace itk



#endif
