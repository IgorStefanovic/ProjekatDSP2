#include "ImageInterpolation.h"
#include "ColorSpaces.h"

#define _USE_MATH_DEFINES
#include <math.h>


void sampleAndHold(const uchar input[], int xSize, int ySize, uchar output[], int newXSize, int newYSize)
{
	int i, j, p ,q;
	double fx, fy;

	uchar* yBuff = new uchar[xSize * ySize];
	char* uBuff = new char[xSize * ySize / 4];
	char* vBuff = new char[xSize * ySize / 4];

	uchar* yBuffO = new uchar[newXSize * newYSize];
	char* uBuffO = new char[newXSize * newYSize / 4];
	char* vBuffO = new char[newXSize * newYSize / 4];

	fx = newXSize / xSize;
	fy = newYSize / ySize;

	RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
	for(j = 0; j < newYSize; j++) {
		for(i = 0; i < newXSize; i++) {
			p = (j-1)/fy + 1;
			q = (i-1)/fx + 1;
			yBuffO[j * newXSize + i] = yBuff[p * xSize + q];
			uBuffO[(j/2) * newXSize/2 + i/2] = uBuff[(p/2) * (xSize/2) + q/2];
			vBuffO[(j/2) * newXSize/2 + i/2] = vBuff[(p/2) * (xSize/2) + q/2];
		}
	}
	YUV420toRGB(yBuffO, uBuffO, vBuffO, newXSize, newYSize, output);

	delete[] yBuff;
	delete[] uBuff;
	delete[] vBuff;

	delete[] yBuffO;
	delete[] uBuffO;
	delete[] vBuffO;
}

void bilinearInterpolate(const uchar input[], int xSize, int ySize, uchar output[], int newXSize, int newYSize)
{
	int i, j, m, n;
	double a ,b, fx, fy;

	uchar* yBuff = new uchar[xSize * ySize];
	char* uBuff = new char[xSize * ySize / 4];
	char* vBuff = new char[xSize * ySize / 4];

	uchar* yBuffO = new uchar[newXSize * newYSize];
	char* uBuffO = new char[newXSize * newYSize / 4];
	char* vBuffO = new char[newXSize * newYSize / 4];
	
	fx = newXSize / xSize;
	fy = newYSize / ySize;

	RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
	for(j = 0; j < newYSize; j++) {
		for(i = 0; i < newXSize; i++) {
			a = i / fy - floor(i / fy);
			b = j / fx - floor(j / fx);

			n = i / fx;
			m = j / fy;

			yBuffO[j * newXSize + i] = (1 - a) * (1 - b) * yBuff[m * xSize + n] + 
									   (1 - a) * b * yBuff[(m + 1) * xSize + n] +
									   a * (1 - b) * yBuff[m * xSize + (n + 1)] +
									   a * b *	yBuff[(m + 1) * xSize + (n + 1)];

			uBuffO[(j/2) * (newXSize/2) + i/2] = (1 - a) * (1 - b) * uBuff[(m/2) * (xSize/2) + n/2] +
												 (1 - a) * b * uBuff[((m/2) + 1) * (xSize/2) + n/2] +
												 a * (1 - b) * uBuff[(m/2) * (xSize/2) + n/2 + 1] +
												 a * b * uBuff[((m/2) + 1) * (xSize/2) + n/2 + 1];

			vBuffO[(j/2) * (newXSize/2) + i/2] = (1 - a) * (1 - b) * vBuff[(m/2) * (xSize/2) + n/2] +
												 (1 - a) * b * vBuff[((m/2) + 1) * (xSize/2) + n/2] +
												 a * (1 - b) * vBuff[(m/2) * (xSize/2) + n/2 + 1] +
											     a * b * vBuff[((m/2) + 1) * (xSize/2) + n/2 + 1];
		}
	}
	YUV420toRGB(yBuffO, uBuffO, vBuffO, newXSize, newYSize, output);

	delete[] yBuff;
	delete[] uBuff;
	delete[] vBuff;

	delete[] yBuffO;
	delete[] uBuffO;
	delete[] vBuffO;
}

void imageRotate(const uchar input[], int xSize, int ySize, uchar output[], int m, int n, double angle)
{
	int i, j, indexX, indexY;

	uchar* yBuff = new uchar[xSize * ySize];
	char* uBuff = new char[xSize * ySize / 4];
	char* vBuff = new char[xSize * ySize / 4];

	uchar* yBuffO = new uchar[xSize * ySize];
	char* uBuffO = new char[xSize * ySize / 4];
	char* vBuffO = new char[xSize * ySize / 4];
	
	angle *= M_PI / 180;

	RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
	for(j = 0; j < ySize; j++) {
		for(i = 0; i < xSize; i++) {
			indexX = i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m;
			indexY = j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n;

			if(indexX >= 0 && indexX < xSize && indexY >= 0 && indexY < ySize) {			
				yBuffO[j * xSize + i] = yBuff[indexY * xSize + indexX];
				uBuffO[(j/2)* xSize/2 + i/2] = uBuff[(indexY/2) * xSize/2 + indexX/2];
				vBuffO[(j/2) * xSize/2 + i/2] = vBuff[(indexY/2) * xSize/2 + indexX/2];
			} else {
				yBuffO[j * xSize + i] = 0;
				uBuffO[(j/2) * xSize/2 + i/2] = 0;
				vBuffO[(j/2) * xSize/2 + i/2] = 0;
			}
		}
	}
	YUV420toRGB(yBuffO, uBuffO, vBuffO, xSize, ySize, output); 

	delete[] yBuff;
	delete[] uBuff;
	delete[] vBuff;

	delete[] yBuffO;
	delete[] uBuffO;
	delete[] vBuffO;
}

void imageRotateBilinear(const uchar input[], int xSize, int ySize, uchar output[], int m, int n, double angle)
{
	int i, j, p, q;
	double a, b, indexX, indexY;

    uchar* yBuff = new uchar[xSize * ySize];
    char* uBuff = new char[xSize * ySize / 4];
    char* vBuff = new char[xSize * ySize / 4];

    uchar* yBuffO = new uchar[xSize * ySize];
    char* uBuffO = new char[xSize * ySize / 4];
    char* vBuffO = new char[xSize * ySize / 4];

	angle *= M_PI / 180;

    RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
    for (j = 0; j < ySize; j++) {
        for (i = 0; i < xSize; i++) {
            indexX = i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m;
			indexY = j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n;

			a = indexX - floor(indexX);
			b = indexY - floor(indexY);

			p = indexX;
			q = indexY;

            if (indexX < 0 || indexX > xSize || indexY < 0 || indexY > ySize) {
                yBuffO[j * xSize + i] = 0;
                uBuffO[(j / 2) * xSize / 2 + i / 2] = 0;
                vBuffO[(j / 2) * xSize / 2 + i / 2] = 0;
            } else {
				yBuffO[j * xSize + i] = (1 - a) * (1 - b) *  yBuff[q * xSize + p] +
											(1 - a) * b * yBuff[(q + 1) * xSize + p] +
											a * (1 - b) * yBuff[q * xSize + p + 1] +
											a * b * yBuff[(q + 1) * xSize + p + 1];
   
				uBuffO[(j/2) * (xSize/2) + i/2] = (1 - a) * (1 - b) * uBuff[(q/2) * (xSize/2) + p/2] +
												 (1 - a) * b * uBuff[((q/2) + 1) * (xSize/2) + p/2] +
												 a * (1 - b) * uBuff[(q/2) * (xSize/2) + p/2 + 1] +
												 a * b * uBuff[((q/2) + 1) * (xSize/2) + p/2 + 1];

				vBuffO[(j/2) * (xSize/2) + i/2] = (1 - a) * (1 - b) * vBuff[(q/2) * (xSize/2) + p/2] +
												 (1 - a) * b * vBuff[((q/2) + 1) * (xSize/2) + p/2] +
												 a * (1 - b) * vBuff[(q/2) * (xSize/2) + p/2 + 1] +
											     a * b * vBuff[((q/2) + 1) * (xSize/2) + p/2 + 1];
            }
        }
    }
    YUV420toRGB(yBuffO, uBuffO, vBuffO, xSize, ySize, output); 

	delete[] yBuff;
	delete[] uBuff;
	delete[] vBuff;

	delete[] yBuffO;
	delete[] uBuffO;
	delete[] vBuffO;
}