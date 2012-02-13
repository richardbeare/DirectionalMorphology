#ifndef __itkDirectionalErodeDilateImageFilter_h
#define __itkDirectionalErodeDilateImageFilter_h

#include "itkImageToImageFilter.h"
#include "itkNumericTraits.h"
#include "itkProgressReporter.h"

namespace itk
{
/**
 * \class DirectionalErodeDilateImageFilter
 * \brief Parent class for morphological operations along lines defined
 * by a vector field. Operations applied whereever the vector field is
 * non zero.
 *
 *
 * \author Richard Beare, Department of Medicine, Monash University,
 * Australia.  <Richard.Beare@monash.edu>
 *
**/


template <typename TInputImage,
	  typename TVectorImage,
	  bool doDilate,
          typename TOutputImage= TInputImage >
class ITK_EXPORT DirectionalErodeDilateImageFilter:
    public ImageToImageFilter<TInputImage,TOutputImage>
{

public:
  /** Standard class typedefs. */
  typedef DirectionalErodeDilateImageFilter  Self;
  typedef ImageToImageFilter<TInputImage,TOutputImage> Superclass;
  typedef SmartPointer<Self>                   Pointer;
  typedef SmartPointer<const Self>        ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Runtime information support. */
  itkTypeMacro(DirectionalErodeDilateImageFilter, ImageToImageFilter);

  /** Pixel Type of the input image */
  typedef TInputImage                                    InputImageType;
  typedef TOutputImage                                   OutputImageType;
  typedef TVectorImage                                   VectorImageType;

  typedef typename TInputImage::PixelType                PixelType;
  typedef typename NumericTraits<PixelType>::RealType    RealType;
  typedef typename NumericTraits<PixelType>::ScalarRealType ScalarRealType;
  typedef typename TOutputImage::PixelType  OutputPixelType;

  /** Smart pointer typedef support.  */
  typedef typename TInputImage::Pointer  InputImagePointer;
  typedef typename TInputImage::ConstPointer  InputImageConstPointer;
  typedef typename TInputImage::SizeType    InputSizeType;
  typedef typename TOutputImage::SizeType   OutputSizeType;

  typedef typename OutputImageType::IndexType       OutputIndexType;

  /** a type to represent the "kernel radius" */
  typedef typename itk::FixedArray<ScalarRealType, TInputImage::ImageDimension> RadiusType;

  /** Image dimension. */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TOutputImage::ImageDimension);
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);


  typedef typename OutputImageType::RegionType OutputImageRegionType;
  /** Define the image type for internal computations
      RealType is usually 'double' in NumericTraits.
      Here we prefer float in order to save memory.  */

  typedef typename NumericTraits< PixelType >::FloatType   InternalRealType;
  //typedef typename Image<InternalRealType, itkGetStaticConstMacro(ImageDimension) >   RealImageType;
  void SetVectorImage(TVectorImage *input)
  {
    // Process object is not const-correct so the const casting is required.
    this->SetNthInput( 1, const_cast<TMaskImage *>(input) );
  }
  VectorImageType * GetVectorImage()
  {
    return static_cast<VectorImageType*>(const_cast<DataObject *>(this->ProcessObject::GetInput(1)));

  }



  /**
   * Set/Get whether the scale refers to pixels or world units -
   * default is false
   */
  itkSetMacro(UseImageSpacing, bool);
  itkGetConstReferenceMacro(UseImageSpacing, bool);
  itkBooleanMacro(UseImageSpacing);
  /** Image related typedefs. */
 
#ifdef ITK_USE_CONCEPT_CHECKING
  /** Begin concept checking */
  itkConceptMacro(SameDimension,
                  (Concept::SameDimension<itkGetStaticConstMacro(InputImageDimension),itkGetStaticConstMacro(OutputImageDimension)>));

  itkConceptMacro(Comparable,
		  (Concept::Comparable<PixelType>));

  /** End concept checking */
#endif

protected:
  DirectionalErodeDilateImageFilter();
  virtual ~DirectionalErodeDilateImageFilter() {};
  void PrintSelf(std::ostream& os, Indent indent) const;
  
  /** Generate Data */
  void GenerateData( void );
  unsigned int SplitRequestedRegion(unsigned int i, unsigned int num,
    OutputImageRegionType & splitRegion);

  void ThreadedGenerateData(const OutputImageRegionType& outputRegionForThread, ThreadIdType threadId );

  void GenerateInputRequestedRegion() throw(InvalidRequestedRegionError);
  // Override since the filter produces the entire dataset.
  void EnlargeOutputRequestedRegion(DataObject *output);

  float m_Length;
  
private:
  DirectionalErodeDilateImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  int m_MagnitudeSign;
  int m_CurrentDimension;
};

} // end namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkDirectionalErodeDilateImageFilter.hxx"
#endif


#endif
