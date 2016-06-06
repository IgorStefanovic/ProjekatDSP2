
#include "ImageProcessing.h"
#include "ImageInterpolation.h"

#include <QDebug>

void imageProcessingFun(const QString& progName, QImage* const outImgs, const QImage* const inImgs, const QVector<double>& params) 
{
	int X_SIZE = inImgs->width();
	int Y_SIZE = inImgs->height();
	int newXSize, newYSize, help;
	/* NOTE: Calculate output image resolution and construct output image object */

	if(progName == "Sample and hold") 
	{	
		/* Input image data in RGB format can be obtained with inImgs->bits() */
		/* Vertical scale factor is params[0] */
		/* Horizontal scale factor is params[1] */

		/* TO DO: Calculate output image resolution and construct output image object */
		newXSize = (int)(params[0] + 0.5) * X_SIZE;
		newYSize = (int)(params[1] + 0.5) * Y_SIZE;

		*outImgs = *(new QImage(newXSize, newYSize, inImgs->format()));

		/* TO DO: Sample and Hold interpolation  */
		sampleAndHold(inImgs->bits(), X_SIZE, Y_SIZE, outImgs->bits(), newXSize, newYSize);
	}
	else if (progName == "Bilinear") 
	{
		/* Input image data in RGB format can be obtained with inImgs->bits() */
		/* Vertical scale factor is params[0] */
		/* Horizontal scale factor is params[1] */

		/* TO DO: Calculate output image resolution and construct output image object */
		newXSize = (int)(params[0] + 0.5) * X_SIZE;
		newYSize = (int)(params[1] + 0.5) * Y_SIZE;

		*outImgs = *(new QImage(newXSize, newYSize, inImgs->format()));

		/* TO DO: Perform Bilinear interpolation  */
		bilinearInterpolate(inImgs->bits(), X_SIZE, Y_SIZE, outImgs->bits(), newXSize, newYSize);
	}
	else if(progName == "Rotation") 
	{	
		/* Input image data in RGB format can be obtained with inImgs->bits() */
		/* Rotation angle in degrees is params[0]*/
		/* Center of rotation coordinates are (XSIZE/2, YSIZE/2) */

		/* TO DO: Construct output image object */
		*outImgs = *(new QImage(X_SIZE, Y_SIZE, inImgs->format()));

		/* TO DO: Perform image rotation */
		imageRotate(inImgs->bits(), X_SIZE, Y_SIZE, outImgs->bits(), X_SIZE/2, Y_SIZE/2, params[0]);
	}
	else if (progName == "Rotation Bilinear") 
	{
		/* Input image data in RGB format can be obtained with inImgs->bits() */
		/* Rotation angle in degrees is params[0]*/
		/* Center of rotation coordinates are (XSIZE/2, YSIZE/2) */

		/* TO DO: Construct output image object */
         *outImgs = *(new QImage(X_SIZE, Y_SIZE, inImgs->format()));
        
		/* TO DO: Perform image rotation with bilinear interpolation */
		 imageRotateBilinear(inImgs->bits(), X_SIZE, Y_SIZE, outImgs->bits(), X_SIZE / 2, Y_SIZE / 2, params[0]);
	}

}

