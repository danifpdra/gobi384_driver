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
#include "XFilters.h"

using namespace std;
using namespace cv;

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
  XCamera *cam;
  XDeviceInformation *devices, *dev;
  cv::Mat thermal_img, color_img;
  ros::Publisher img_pub;
  image_transport::ImageTransport it;
  std_msgs::Header header;
  sensor_msgs::ImagePtr msg_thermal;
  ErrCode errorCode = I_OK;
  dword frameSize = 0; // The size in bytes of the raw image.
  std::vector<word> framebuffer;
  const char *packname;
  FilterID fltThermography = 0; // Handle to the thermography filter.
  double *tempLUT = 0;          // Temperature lookup table (ADU to temperature)
  const char *newint = "64";
  FilterID histoFlt = 0;

  // declare functions here
  void getImage();
  XDeviceInformation *DeviceDescovery();
};

/**
 * @brief Construct a new Thermal Cam:: Thermal Cam object
 *
 */
ThermalCam::ThermalCam() : it(nh) {
  img_pub = nh.advertise<sensor_msgs::Image>("thermal_img", 10);
}

/**
 * @brief Loop function
 *
 */
void ThermalCam::loop_function() {
  getImage();
  img_pub.publish(msg_thermal);
}

/**
 * @brief Function to detect the connected devices
 *
 * @return XDeviceInformation*
 */
XDeviceInformation *ThermalCam::DeviceDescovery() {
  unsigned int deviceCount = 0;

  errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll);
  if (errorCode != I_OK) {
    printf("An error occurred while enumerating the devices. errorCode: %lu\n",
           errorCode);
    return NULL;
  } else if (deviceCount == 0) {
    printf("Enumeration was a success but no devices were found!\n");
    return NULL;
  }

  devices = new XDeviceInformation[deviceCount];
  errorCode = XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached);

  if (errorCode != I_OK) {
    printf("Error while retrieving the cached device information structures. "
           "ErrorCode: %lu\n",
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
  dev = &devices[0];
  return dev;
}

/**
 * @brief Function to connect to the camera
 *
 * @return XCamera*
 */
XCamera *ThermalCam::connectCam() {

  XDeviceInformation *dev = DeviceDescovery();
  cam = XCamera::Create(dev->url, NULL, NULL);

  if (cam == NULL) {
    printf("Unable to create API instance");
    return NULL;
  } else if (cam->IsInitialised() == false) {
    printf("Unable to establish a connection");
    delete cam;
    return NULL;
  } else {
    printf("Connected to device\n");
    return cam;
  }
}

/**
 * @brief Function to start capturing and activate filters
 *
 */
void ThermalCam::startCap() {
  cam = connectCam();

  packname =
      "/home/daniela/catkin_ws/src/thermal_camera/config/calibration_5449.xca";
  handle = XC_OpenCamera(dev->url);

  header.frame_id = "map";
  header.stamp = ros::Time(0);
  newint = "64";

  XC_SetPropertyValue(handle, "IntegrationTime", newint, "");

  errorCode = XC_LoadCalibration(handle, packname, XLC_StartSoftwareCorrection);
  if (I_OK == errorCode) {
    fltThermography = XC_FLT_Queue(handle, "Thermography", "celsius");
    histoFlt = XC_FLT_Queue(handle, "AutoGain", "");

    if (fltThermography > 0) {
      // Build the look-up table and ..
      dword mv = XC_GetMaxValue(handle);
      tempLUT = new double[mv + 1];

      for (dword x = 0; x < mv + 1; x++) {
        XC_FLT_ADUToTemperature(handle, fltThermography, x, &tempLUT[x]);
      }

      printf("Start capturing.\n");
      errorCode = XC_StartCapture(handle);

      if (errorCode != I_OK) {
        printf("Could not start capturing, errorCode: %lu\n", errorCode);
      } else {
        printf("Initialization completed!\n");
      }
    }
  }
}

/**
 * @brief Function to get frames at each loop
 *
 */
void ThermalCam::getImage() {

  // When the connection is initialised, ...
  if (XC_IsInitialised(handle)) {
    // ... start capturing
    if (XC_IsCapturing(handle)) // When the camera is capturing ...
    {
      // Determine native framesize.
      frameSize = XC_GetFrameSize(handle);

      // Initialize the 16-bit buffer.
      framebuffer.resize(frameSize / 2);

      // ... grab a frame from the camera.
      // printf("Grabbing a frame.\n");
      errorCode = XC_GetFrame(handle, FT_16_BPP_GRAY, XGF_Blocking,
                              framebuffer.data(), frameSize);
      if (errorCode != I_OK) {
        printf("Problem while fetching frame, errorCode %lu", errorCode);
      }

      int h = XC_GetHeight(handle);
      int w = XC_GetWidth(handle);

      thermal_img =
          cv::Mat(h, w, CV_16UC1, framebuffer.data()); /*convert to OpenCV*/

      cv::Mat color_img;

      thermal_img /= 256; // convert pixel values to 8bit

      thermal_img.convertTo(thermal_img, CV_8U); // convert image to 8bit

      /*add colormap to image to visualize information in rgb. Colormaps
        available:
        COLORMAP_AUTUMN
        COLORMAP_BONE
        COLORMAP_COOL
        COLORMAP_HOT
        COLORMAP_HSV
        COLORMAP_JET
        COLORMAP_OCEAN
        COLORMAP_PINK
        COLORMAP_RAINBOW
        COLORMAP_SPRING
        COLORMAP_SUMMER
        COLORMAP_WINTER*/
      cv::applyColorMap(thermal_img, color_img, cv::COLORMAP_AUTUMN);

      msg_thermal = cv_bridge::CvImage{header, "bgr8", color_img}
                        .toImageMsg(); /*convert to ROS msg*/
    }
  }
}

/**
 * @brief Function to stop capturing and close camera handle
 *
 */
void ThermalCam::stopCap() {
  XC_StopCapture(handle);
  XC_CloseCamera(handle);

  delete[] tempLUT;
  delete cam;

  std::cout << "Camera closed" << std::endl;
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
