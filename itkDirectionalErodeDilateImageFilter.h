#ifndef __itkDirectionalErodeDilateImageFilter_h
#define __itkDirectionalErodeDilateImageFilter_h

#include "itkImageToImageFilter.h"
#include "itkNumericTraits.h"
#include "itkProgressReporter.h"
#include "itkBresenhamLine.h"

namespace itk
{
/**
 * \class DirectionalErodeDilateImageFilter
 * \brief Parent class for morphological operations along lines defined
 * by a vector field. Operations applied whereever the mask image is
 * non zero.
 *
 *
 * \author Richard Beare, Department of Medicine, Monash University,
 * Australia.  <Richard.Beare@monash.edu>
 *
**/


template <typename TInputImage,
	  typename TVectorImage,
	  typename TMaskImage,
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
  typedef TMaskImage                                     MaskImageType;

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

  /** Image dimension. */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TOutputImage::ImageDimension);
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);


  void SetVectorImage(TVectorImage *input)
  {
    // Process object is not const-correct so the const casting is required.
    this->SetNthInput( 1, const_cast<TVectorImage *>(input) );
  }
  VectorImageType * GetVectorImage()
  {
    return static_cast<VectorImageType*>(const_cast<DataObject *>(this->ProcessObject::GetInput(1)));

  }

  void SetMaskImage(TMaskImage *input)
  {
    // Process object is not const-correct so the const casting is required.
    this->SetNthInput( 2, const_cast<TMaskImage *>(input) );
  }
  MaskImageType * GetMaskImage()
  {
    return static_cast<MaskImageType*>(const_cast<DataObject *>(this->ProcessObject::GetInput(2)));

  }

  /**
   * Set/Get whether the scale refers to pixels or world units -
   * default is true
   */
  itkSetMacro(UseImageSpacing, bool);
  itkGetConstReferenceMacro(UseImageSpacing, bool);
  itkBooleanMacro(UseImageSpacing);

  /**
   * Set/Get whether the scale refers to pixels or world units -
   * default is true
   */
  itkSetMacro(LineLength, float);
  itkGetConstReferenceMacro(LineLength, float);

  itkSetMacro(FilterLength, float);
  itkGetConstReferenceMacro(FilterLength, float);


  /**
   * Set/Get whether the scale refers to pixels or world units -
   * default is true
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
  
  void GenerateInputRequestedRegion();

  void EnlargeOutputRequestedRegion(DataObject *itkNotUsed(output));


  /** Generate Data */
  void GenerateData( void );

  float m_LineLength;
  float m_FilterLength;
  bool m_UseImageSpacing;
private:
  DirectionalErodeDilateImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  typedef BresenhamLine< itkGetStaticConstMacro(InputImageDimension) > BresType;


};

} // end namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkDirectionalErodeDilateImageFilter.hxx"
#endif


#endif
