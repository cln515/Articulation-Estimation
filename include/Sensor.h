#if ENABLE_KINECT_V2
#include <kinect.h>
#endif



class RGBD_Sensor {
public:
	void openInitDevice();


#if ENABLE_KINECT_V2
	ComPtr<IKinectSensor> iKinect;

#endif


};