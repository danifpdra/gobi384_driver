// ros package to communicate with thermal camera

#include <ros/console.h>
#include <ros/ros.h>
//-----------------
#include "stdio.h"
#include <algorithm> // std::max
#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>

/* openCv*/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
// #include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//---------Xeneth Camera ------
#include "XCamera.h"

class ThermalCam {
public:
  // publicadores
  ThermalCam();

  // Filter the cloud based on recieved data
  void loop_function();
  XCamera *connectCam();
  void startCap();
  void stopCap();

  XCHANDLE handle = 0;

private:
  // rosnode
  ros::NodeHandle nh;

  // declare variables
  cv::Mat thermal_img;
  ros::Publisher img_pub;
  image_transport::ImageTransport it;
  std_msgs::Header header;
  sensor_msgs::ImagePtr msg_thermal;
  ErrCode errorCode = I_OK;
  dword frameSize = 0; // The size in bytes of the raw image.
  std::vector<word> framebuffer;

  // declare functions here
  void getImage();
  XDeviceInformation *DeviceDescovery();
};

ThermalCam::ThermalCam() : it(nh) {
  img_pub = nh.advertise<sensor_msgs::Image>("thermal_img", 10);
}

void ThermalCam::loop_function() {
  // DeviceDescovery();
  getImage();
  img_pub.publish(msg_thermal);
}

XDeviceInformation *ThermalCam::DeviceDescovery() {
  unsigned int deviceCount = 0;

  if ((errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll)) !=
      I_OK) {
    printf("An error occurred while enumerating the devices. errorCode: %lu\n",
           errorCode);
    return NULL;
  }

  if (deviceCount == 0) {
    printf("Enumeration was a success but no devices were found!\n");
    return NULL;
  }

  /*  At this point we know how much devices are present in the environment.
   *  Now allocate the XDeviceInformation array to accommodate all the
   * discovered devices using the device count to determine the size needed.
   * Once allocated we call the enumerate devices method again but now instead
   * of passing null as the initial argument use the new allocated buffer. For
   * the flags argument we no longer use a protocol enable flag but make use of
   * the XEF_UseCached flag. On discovery devices are cached internally and as
   * such we are able to retrieve the device information structures instantly
   * when calling XCD_EnumerateDevices for a second time. Note that it is not
   * required to first check device count, allocate structure and retrieve
   * cached devices. A user could allocate one or more device structure and
   * immediately pass this with the initial call to XCD_EnumerateDevices.
   *  XCD_EnumerateDevices will not exceed the supplied deviceCount and when
   * less devices were discovered than the initial deviceCount
   *  this argument is updated with the new count. */

  XDeviceInformation *devices = new XDeviceInformation[deviceCount];
  if ((errorCode = XCD_EnumerateDevices(devices, &deviceCount,
                                        XEF_UseCached)) != I_OK) {
    printf("Error while retrieving the cached device information structures. "
           "errorCode: %lu\n",
           errorCode);
    delete[] devices;
    return NULL;
  }

  /*  All discovered devices are now available in our local array and we are now
   * able to iterate the list and output each item in the array */

  for (unsigned int i = 0; i < deviceCount; i++) {
    XDeviceInformation *dev = &devices[i];
    printf("device[%u] %s @ %s (%s) \n", i, dev->name, dev->address,
           dev->transport);
    printf("PID: %4X\n", dev->pid);
    printf("Serial: %u\n", dev->serial);
    printf("URL: %s\n", dev->url);
    printf("State: %s\n\n",
           dev->state == XDS_Available
               ? "Available"
               : dev->state == XDS_Busy ? "Busy" : "Unreachable");
  }
  XDeviceInformation *dev = &devices[0];
  return dev;
}

XCamera *ThermalCam::connectCam() {

  XDeviceInformation *dev = DeviceDescovery();

  XCamera *cam = XCamera::Create(dev->url, NULL, NULL);

  if (cam == NULL) {
    printf("Unable to create API instance");
    return NULL;
  } else if (cam->IsInitialised() == false) {
    printf("Unable to establish a connection");
    delete cam;
    return NULL;
  } else {
    printf("Connected to device");
    return cam;
  }
}

void ThermalCam::startCap() {
  XCamera *cam = connectCam();
  XDeviceInformation *dev = DeviceDescovery();
  ErrCode errorCode =
      0; // Used to store returned errorCodes from the SDK functions.
  handle = XC_OpenCamera(dev->url);

  header.frame_id = "map";
  header.stamp = ros::Time(0);
  const char *newint = "64";

  XC_SetPropertyValue(handle, "IntegrationTime", newint, "");

  printf("Start capturing.\n");
  if ((errorCode = XC_StartCapture(handle)) != I_OK) {
    printf("Could not start capturing, errorCode: %lu\n", errorCode);
  } else {
    printf("Initialization failed\n");
  }
}

void ThermalCam::stopCap() {
  XC_StopCapture(handle);
  XC_CloseCamera(handle);
  std::cout << "Camera closed" << std::endl;
}

// AUTOGAIN

// void XMainFrame::OnAutoGain(wxCommandEvent &evt)
// {
//     static FilterID histoFlt = 0;
//     if(evt.IsChecked())
//     {
//         histoFlt = QueueFilter(m_pCam, "AutoGain", "");
//     }
//     else
//     {
//         m_pCam->RemImageFilter(histoFlt);
//     }
// }

void ThermalCam::getImage() {

  // When the connection is initialised, ...
  if (XC_IsInitialised(handle)) {
    // ... start capturing
    if (XC_IsCapturing(handle)) // When the camera is capturing ...
    {

      // for (;;) {

      // Determine native framesize.
      frameSize = XC_GetFrameSize(handle);

      // Initialize the 16-bit buffer.
      // frameBuffer = new word[frameSize / 2];
      framebuffer.resize(frameSize / 2);

      // ... grab a frame from the camera.
      printf("Grabbing a frame.\n");
      if ((errorCode = XC_GetFrame(handle, FT_16_BPP_GRAY, XGF_Blocking,
                                   framebuffer.data(), frameSize)) != I_OK) {
        printf("Problem while fetching frame, errorCode %lu", errorCode);
      }

      int h = XC_GetHeight(handle);
      int w = XC_GetWidth(handle);
      // Mat(int rows, int cols, int type, void *data, size_t step =
      // AUTO_STEP);
      thermal_img = cv::Mat(h, w,CV_16UC1 , framebuffer.data());

      // std::cout << thermal_img<< std::endl;
      msg_thermal =
          cv_bridge::CvImage{header, "mono16", thermal_img}.toImageMsg();
      // }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ThermalCam");
  ThermalCam reconstruct;

  reconstruct.startCap();

  ros::Rate rate(50);
  while (ros::ok()) {
    reconstruct.loop_function();

    ros::spinOnce();
    rate.sleep();
  }
  reconstruct.stopCap();

  return 0;
}
