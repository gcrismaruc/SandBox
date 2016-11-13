#include "Aquisition.h"


Aquisition::Aquisition() :
	m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE),
	m_pDepthStreamHandle(INVALID_HANDLE_VALUE),
	m_bNearMode(false),
	m_pNuiSensor(NULL)
{
	// create heap storage for depth pixel data in RGBX format
	m_depthRGBX = new USHORT[cDepthWidth*cDepthHeight*cBytesPerPixel];
	//paletteForDepth = cvLoadImage("C:\\Users\\Gheorghe\\Desktop\\palette.bmp", CV_LOAD_IMAGE_COLOR);
	///input = (unsigned char *)(this->paletteForDepth.data);

	//paletteSize = 0;
}

Aquisition::~Aquisition()
{
	if (m_pNuiSensor)
	{
		m_pNuiSensor->NuiShutdown();
	}

	if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
	{
		CloseHandle(m_hNextDepthFrameEvent);
	}

	// clean up Direct2D renderer
	//delete m_pDrawDepth;
	// m_pDrawDepth = NULL;

	// done with depth pixel data
	delete[] m_depthRGBX;

	// clean up Direct2D
	//SafeRelease(m_pD2DFactory);

	SafeRelease(m_pNuiSensor);
}

HRESULT Aquisition::CreateFirstConnected()
{
	INuiSensor * pNuiSensor;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		return hr;
	}

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}

	if (NULL != m_pNuiSensor)
	{

		cout<<"\n Using depth map";
		// Initialize the Kinect and specify that we'll be using depth
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH); 
		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when depth data is available
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a depth image stream to receive depth frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				m_hNextDepthFrameEvent,
				&m_pDepthStreamHandle);
		}
	}

	return hr;
}

USHORT* Aquisition::Update()
{

	//cout<<"Aq in update\n";
	if (NULL == m_pNuiSensor)
	{
		return NULL;
	}
	return ProcessDepth();
}

USHORT* Aquisition::ProcessDepth()
{

	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the depth frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 6000, &imageFrame);
	if (FAILED(hr))
	{
		return NULL;
	}

	BOOL nearMode;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	if (FAILED(hr))
	{
		cout<<"failed";
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		// Get the min and max reliable depth for the current frame
		int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
		int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		USHORT * rgbrun = m_depthRGBX;
		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

		// end pixel is start + width*height - 1
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		while ( pBufferRun < pBufferEnd )
		{
			// discard the portion of the depth that contains only the player index
			USHORT depth = pBufferRun->depth;

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			
			//USHORT value = ((double)depth / maxDepth ) * 256;

			//BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? value  : 0);

			// Write out blue byte
			*(rgbrun++) = depth;

			// Write out green byte
			//*(rgbrun++) = depth;

			// Write out red byte
			//*(rgbrun++) = depth;

			// We're outputting BGR, the last byte in the 32 bits is unused so skip it
			// If we were outputting BGRA, we would write alpha here.
			//++rgbrun;

			// Increment our index into the Kinect's depth buffer
			++pBufferRun;
		}
	}

	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	pTexture->Release();


	
ReleaseFrame:
	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
	
	return m_depthRGBX;
}
