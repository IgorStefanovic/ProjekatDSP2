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

	fx = (double)newXSize/xSize;
	fy = (double)newYSize/ySize;

	RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
	for(j = 0; j < newYSize; j++) {
		for(i = 0; i < newXSize; i++) {
			p = j/fy;
			q = i/fx;

			if(p >= ySize) {
				p = ySize - 1;
			}
			if(q >= xSize) {
				q = xSize - 1;
			}

			yBuffO[j * newXSize + i] = yBuff[p * xSize + q];
		}
	}
	for(j = 0; j < newYSize/2; j++) {
		for(i = 0; i < newXSize/2; i++) {
			p = j/fy;
			q = i/fx;

			if(p >= ySize/2) {
				p = ySize/2 - 1;
			}
			if(q >= xSize/2) {
				q = xSize/2 - 1;
			}

			uBuffO[j * newXSize/2 + i] = uBuff[p * (xSize/2) + q];
			vBuffO[j * newXSize/2 + i] = vBuff[p * (xSize/2) + q];
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
	int i, j, m, n, newM, newN;
	double a ,b, fx, fy;

	uchar* yBuff = new uchar[xSize * ySize];
	char* uBuff = new char[xSize * ySize / 4];
	char* vBuff = new char[xSize * ySize / 4];

	uchar* yBuffO = new uchar[newXSize * newYSize];
	char* uBuffO = new char[newXSize * newYSize / 4];
	char* vBuffO = new char[newXSize * newYSize / 4];
	
	fx = (double)newXSize/xSize;
	fy = (double)newYSize/ySize;

	RGBtoYUV420(input, xSize, ySize, yBuff, uBuff, vBuff);
	for(j = 0; j < newYSize; j++) {
		for(i = 0; i < newXSize; i++) {
			a = i / fy - floor(i / fy);
			b = j / fx - floor(j / fx);

			n = i / fx;
			m = j / fy;

			newM = m + 1;
			newN = n + 1;
			if(newM >= ySize) {
				newM = ySize - 1;
			}
			if(newN >= xSize) {
				newN = xSize - 1;
			}

			yBuffO[j * newXSize + i] = (1 - a) * (1 - b) * yBuff[m * xSize + n] + 
									   (1 - a) * b * yBuff[newM * xSize + n] +
									   a * (1 - b) * yBuff[m * xSize + newN] +
									   a * b *	yBuff[newM * xSize + newN];
		}
	}
	for(j = 0; j < newYSize/2; j++) {
		for(i = 0; i < newXSize/2; i++) {
			a = i / fy - floor(i / fy);
			b = j / fx - floor(j / fx);

			n = i / fx;
			m = j / fy;

			newM = m + 1;
			newN = n + 1;
			if(newM >= ySize/2) {
				newM = ySize/2 - 1;
			}
			if(newN >= xSize/2) {
				newN = xSize/2 - 1;
			}

			uBuffO[j * (newXSize/2) + i] = (1 - a) * (1 - b) * uBuff[m * (xSize/2) + n] +
												 (1 - a) * b * uBuff[newM * (xSize/2) + n] +
												 a * (1 - b) * uBuff[m * (xSize/2) + newN] +
												 a * b * uBuff[newM * (xSize/2) + newN];

			vBuffO[j * (newXSize/2) + i] = (1 - a) * (1 - b) * vBuff[m * (xSize/2) + n] +
												 (1 - a) * b * vBuff[newM * (xSize/2) + n] +
												 a * (1 - b) * vBuff[m * (xSize/2) + newN] +
											     a * b * vBuff[newM * (xSize/2) + newN];
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
			indexX = round(i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m);
			indexY = round(j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n);

			if(indexX < 0 || indexX >= xSize || indexY < 0 || indexY >= ySize) {
				yBuffO[j * xSize + i] = 0;
			} else {
				yBuffO[j * xSize + i] = yBuff[indexY * xSize + indexX];
			}
		}
	}
	for(j = 0; j < ySize/2; j++) {
		for(i = 0; i < xSize/2; i++) {
			indexX = round(i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m);
			indexY = round(j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n);

			if(indexX < 0 || indexX >= xSize || indexY < 0 || indexY >= ySize) {
				uBuffO[j * xSize/2 + i] = 0;
				vBuffO[j * xSize/2 + i] = 0;
			} else {
				uBuffO[j * xSize/2 + i] = uBuff[indexY * xSize/2 + indexX];
				vBuffO[j * xSize/2 + i] = vBuff[indexY * xSize/2 + indexX];
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

			indexXprim = indexX;
			indexYprim = indexY;

            if (round(indexX) < 0 || round(indexX) >= xSize || round(indexY) < 0 || round(indexY) >= ySize) {
                yBuffO[j * xSize + i] = 0;
            } else {
				yBuffO[j * xSize + i] = (1 - a) * (1 - b) *  yBuff[indexYprim * xSize + indexXprim] +
										(1 - a) * b * yBuff[(indexYprim + 1) * xSize + indexXprim] +
										a * (1 - b) * yBuff[indexYprim * xSize + indexXprim + 1] +
										a * b * yBuff[(indexYprim + 1) * xSize + indexXprim + 1];
            }
        }
    }
	for (j = 0; j < ySize/2; j++) {
        for (i = 0; i < xSize/2; i++) {
            indexX = i * cos(angle) - j * sin(angle) - m * cos(angle) + n * sin(angle) + m;
			indexY = j * cos(angle) + i * sin(angle) - m * sin(angle) - n * cos(angle) + n;

			a = indexX - floor(indexX);
			b = indexY - floor(indexY);

			indexXprim = indexX;
			indexYprim = indexY;

            if (round(indexX) < 0 || round(indexX) >= xSize || round(indexY) < 0 || round(indexY) >= ySize) {
                uBuffO[j * xSize / 2 + i] = 0;
                vBuffO[j * xSize / 2 + i] = 0;
            } else {
				uBuffO[j * (xSize/2) + i] = (1 - a) * (1 - b) * uBuff[indexYprim * (xSize/2) + indexXprim] +
												 (1 - a) * b * uBuff[(indexYprim + 1) * (xSize/2) + indexXprim] +
												 a * (1 - b) * uBuff[indexYprim * (xSize/2) + indexXprim + 1] +
												 a * b * uBuff[(indexYprim + 1) * (xSize/2) + indexXprim + 1];
				vBuffO[j * (xSize/2) + i] = (1 - a) * (1 - b) * vBuff[indexYprim * (xSize/2) + indexXprim] +
												 (1 - a) * b * vBuff[(indexYprim + 1) * (xSize/2) + indexXprim] +
												 a * (1 - b) * vBuff[indexYprim * (xSize/2) + indexXprim + 1] +
											     a * b * vBuff[(indexYprim + 1) * (xSize/2) + indexXprim + 1];
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

int round(double number) {
	if(number >= 0) {
		if(number - (int)number < 0.5) {
			return (int)floor(number);
		} else {
			return (int)ceil(number);
		}
	} else {
		if(-number + (int)number < 0.5) {
			return (int)ceil(number);
		} else {
			return (int)floor(number);
		}
	}
}