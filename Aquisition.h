#pragma once

#include <string>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\opencv.hpp>
#include "stdafx.h"
#include "resource.h"
#include "NuiApi.h"

using namespace std;
using namespace cv;

class Aquisition {

	
public:

	Aquisition();
	~Aquisition();
	HRESULT                 CreateFirstConnected();
    USHORT*                     ProcessDepth();
	USHORT*                     Update();

    HWND                    m_hWnd;
    bool                    m_bNearMode;

    // Current Kinect
    INuiSensor*             m_pNuiSensor;

 
 
    HANDLE                  m_pDepthStreamHandle;
    HANDLE                  m_hNextDepthFrameEvent;

    USHORT*                   m_depthRGBX;

	static const int        cDepthWidth  = 640;
    static const int        cDepthHeight = 480;
    static const int        cBytesPerPixel = 8;

    static const int        cStatusMessageMaxLen = MAX_PATH*2;

};