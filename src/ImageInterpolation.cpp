#include "ImageInterpolation.h"
#include "ColorSpaces.h"
#include <math.h>

#define M_PI 3.14151926535


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

	fx = (double)newXSize / xSize;
	fy = (double)newYSize / ySize;

	RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
	for(j = 0; j < newYSize; j++) {
		for(i = 0; i < newXSize; i++) {
			p = (j-1)/fy + 1;
			q = (i-1)/fx + 1;
			yBuffO[j * newXSize + i] = yBuff[p * xSize + q];
		}
	}
	for(j = 0; j < newYSize; j+=2) {
		for(i = 0; i < newXSize; i+=2) {
			p = (j-1)/fy + 1;
			q = (i-1)/fx + 1;
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
	
	fx = (double)newXSize / xSize;
	fy = (double)newYSize / ySize;

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
		}
	}
	for(j = 0; j < newYSize; j+=2) {
		for(i = 0; i < newXSize; i+=2) {
			a = i / fy - floor(i / fy);
			b = j / fx - floor(j / fx);

			n = i / fx;
			m = j / fy;

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
			indexX = floor(i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m + 0.5);
			indexY = floor(j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n + 0.5);

			if(indexX < 0 || indexX >= xSize || indexY < 0 || indexY >= ySize) {
				yBuffO[j * xSize + i] = 0;
			} else {
				yBuffO[j * xSize + i] = yBuff[indexY * xSize + indexX];
			}
		}
	}
	for(j = 0; j < ySize; j+=2) {
		for(i = 0; i < xSize; i+=2) {
			indexX = floor(i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m + 0.5);
			indexY = floor(j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n + 0.5);

			if(indexX < 0 || indexX >= xSize || indexY < 0 || indexY >= ySize) {
				uBuffO[(j/2) * xSize/2 + i/2] = 0;
				vBuffO[(j/2) * xSize/2 + i/2] = 0;
			} else {
				uBuffO[(j/2)* xSize/2 + i/2] = uBuff[(indexY/2) * xSize/2 + indexX/2];
				vBuffO[(j/2) * xSize/2 + i/2] = vBuff[(indexY/2) * xSize/2 + indexX/2];
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
	int i, j, indexXprim, indexYprim;
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

			indexXprim = floor(indexX + 0.5);
			indexYprim = floor(indexY + 0.5);

            if (indexX < 0 || indexX >= xSize || indexY < 0 || indexY >= ySize) {
                yBuffO[j * xSize + i] = 0;
            } else {
				yBuffO[j * xSize + i] = (1 - a) * (1 - b) *  yBuff[indexYprim * xSize + indexXprim] +
										(1 - a) * b * yBuff[(indexYprim + 1) * xSize + indexXprim] +
										a * (1 - b) * yBuff[indexYprim * xSize + indexXprim + 1] +
										a * b * yBuff[(indexYprim + 1) * xSize + indexXprim + 1];
            }
        }
    }
	for (j = 0; j < ySize; j+=2) {
        for (i = 0; i < xSize; i+=2) {
            indexX = i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m;
			indexY = j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n;

			a = indexX - floor(indexX);
			b = indexY - floor(indexY);

			indexXprim = floor(indexX + 0.5);
			indexYprim = floor(indexY + 0.5);

            if (indexX < 0 || indexX >= xSize || indexY < 0 || indexY >= ySize) {
                uBuffO[(j / 2) * xSize / 2 + i / 2] = 0;
                vBuffO[(j / 2) * xSize / 2 + i / 2] = 0;
            } else {
				uBuffO[(j/2) * (xSize/2) + i/2] = (1 - a) * (1 - b) * uBuff[(indexYprim/2) * (xSize/2) + indexXprim/2] +
												 (1 - a) * b * uBuff[((indexYprim/2) + 1) * (xSize/2) + indexXprim/2] +
												 a * (1 - b) * uBuff[(indexYprim/2) * (xSize/2) + indexXprim/2 + 1] +
												 a * b * uBuff[((indexYprim/2) + 1) * (xSize/2) + indexXprim/2 + 1];

				vBuffO[(j/2) * (xSize/2) + i/2] = (1 - a) * (1 - b) * vBuff[(indexYprim/2) * (xSize/2) + indexXprim/2] +
												 (1 - a) * b * vBuff[((indexYprim/2) + 1) * (xSize/2) + indexXprim/2] +
												 a * (1 - b) * vBuff[(indexYprim/2) * (xSize/2) + indexXprim/2 + 1] +
											     a * b * vBuff[((indexYprim/2) + 1) * (xSize/2) + indexXprim/2 + 1];
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