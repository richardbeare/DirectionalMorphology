#ifndef __itkDirectionalCloseImageFilter_h
#define __itkDirectionalCloseImageFilter_h

#include "itkImageToImageFilter.h"
#include "itkNumericTraits.h"
#include "itkProgressReporter.h"
#include "itkBresenhamLine.h"
#include "itkCovariantVector.h"
namespace itk
{
/**
 * \class DirectionalCloseImageFilter
 * \brief Performs a morphological dilation followed by an erosion along lines defined
 * by a vector field. Operations applied whereever the mask image is
 * non zero. This does both operations along the line without writing
 * back to the image in between. Allows sensible safe border options
 *
 *
 * \author Richard Beare, Department of Medicine, Monash University,
 * Australia.  <Richard.Beare@monash.edu>
 *
**/

namespace DirectionalClose
{
template< class TPixel >
class MaxFunctor
{
public:
  MaxFunctor(){}
  ~MaxFunctor(){}
  inline TPixel operator()(const TPixel & A, const TPixel & B) const
  {
    return vnl_math_max(A, B);
  }
};

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

}

template <typename TInputImage,
	  typename TMaskImage,
	  typename TVectorImage=Image<CovariantVector<float, TInputImage::ImageDimension>, TInputImage::ImageDimension >
	  >
class ITK_EXPORT DirectionalCloseImageFilter:
    public ImageToImageFilter<TInputImage,TInputImage>
{

public:
  /** Standard class typedefs. */
  typedef DirectionalCloseImageFilter  Self;
  typedef ImageToImageFilter<TInputImage,TInputImage> Superclass;
  typedef SmartPointer<Self>                   Pointer;
  typedef SmartPointer<const Self>        ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Runtime information support. */
  itkTypeMacro(DirectionalCloseImageFilter, ImageToImageFilter);

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
  typedef typename TInputImage::Pointer  InputImagePointer;
  typedef typename TInputImage::ConstPointer  InputImageConstPointer;
  typedef typename TInputImage::SizeType    InputSizeType;
  typedef typename TInputImage::IndexType   InputIndexType;
  typedef typename OutputImageType::SizeType   OutputSizeType;

  typedef typename OutputImageType::IndexType       OutputIndexType;

  /** Image dimension. */
  itkStaticConstMacro(ImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(OutputImageDimension, unsigned int,
                      TInputImage::ImageDimension);
  itkStaticConstMacro(InputImageDimension, unsigned int,
                      TInputImage::ImageDimension);


  void SetVectorImage(TVectorImage *input)
  {
    // Process object is not const-correct so the const casting is required.
    this->SetNthInput( 2, const_cast<TVectorImage *>(input) );
  }
  VectorImageType * GetVectorImage()
  {
    return static_cast<VectorImageType*>(const_cast<DataObject *>(this->ProcessObject::GetInput(2)));

  }

  void SetMaskImage(TMaskImage *input)
  {
    // Process object is not const-correct so the const casting is required.
    this->SetNthInput( 1, const_cast<TMaskImage *>(input) );
  }
  MaskImageType * GetMaskImage()
  {
    return static_cast<MaskImageType*>(const_cast<DataObject *>(this->ProcessObject::GetInput(1)));

  }
  
  /** 
   * Set/Get the target index. This is used if the VectorImage is not
   * set. Default is [0,0,...]
   */
  itkSetMacro(TargetIndex, InputIndexType);
  itkGetConstReferenceMacro(TargetIndex, InputIndexType);

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

  itkSetMacro(DilFilterLength, float);
  itkGetConstReferenceMacro(DilFilterLength, float);

  itkSetMacro(EroFilterLength, float);
  itkGetConstReferenceMacro(EroFilterLength, float);

  /** 
   * Set/Get the scale - used to reverse the direction of the
   * line. Default = 1.0
   */
  itkSetMacro(Scale, float);
  itkGetConstReferenceMacro(Scale, float);

  /** Set/Get the boundary value. */
  itkSetMacro(Boundary, PixelType);
  itkGetConstMacro(Boundary, PixelType);

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
  DirectionalCloseImageFilter();
  virtual ~DirectionalCloseImageFilter() {};
  void PrintSelf(std::ostream& os, Indent indent) const;
  
  void GenerateInputRequestedRegion();

  void EnlargeOutputRequestedRegion(DataObject *itkNotUsed(output));


  /** Generate Data */
  void GenerateData( void );

  InputIndexType  m_TargetIndex;

  float m_Scale;
  float m_LineLength;
  float m_DilFilterLength;
  float m_EroFilterLength;
  bool m_UseImageSpacing;
  PixelType m_Boundary;

  DirectionalClose::MaxFunctor<PixelType> MxFunc;
  DirectionalClose::MinFunctor<PixelType> MnFunc;

  //MxFunctionType m_MxFunc;

  template <class TFunction1>
  void OneLine(unsigned bsize, unsigned OpLength, std::vector<PixelType> &buffer,
   	       std::vector<PixelType> &forward, std::vector<PixelType> &reverse,
   	       TFunction1 &TF);
private:
  DirectionalCloseImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  typedef BresenhamLine< itkGetStaticConstMacro(InputImageDimension) > BresType;


};

} // end namespace itk

#ifndef ITK_MANUAL_INSTANTIATION
#include "itkDirectionalCloseImageFilter.hxx"
#endif


#endif
