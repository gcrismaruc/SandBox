#pragma once

#include <string>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include "stdafx.h"
#include "resource.h"
#include "NuiApi.h"
#include "Aquisition.h"


using namespace std;
using namespace cv;

typedef struct Color {
	BYTE red;
	BYTE green;
	BYTE blue;
} color;

class DepthMap{

public:
	static int				imageNumber;
	color 					palette[256];
	int						paletteSize;
	Mat						paletteForDepth;
	unsigned char           *input;
	Aquisition aquisition;


	DepthMap(Aquisition *aquisition);
	~DepthMap();

	//HRESULT                 CreateFirstConnected();
   // void                    ProcessDepth(int option);
	//void                    Update(int option);
	int                     saveAFrame(BYTE *imageArray);
	void					ColorTheImage(BYTE * imageArray);
	void					createPalette(color palette[256], int &paletteSize);
	static void             onMouse( int event, int x, int y, int, void* );

    
};
