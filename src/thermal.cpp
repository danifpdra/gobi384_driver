// ros package to communicate with thermal camera : 169.254.104.30

#include <ros/console.h>
#include <ros/ros.h>
#include <rospack/rospack.h>
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
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <thermal_camera/TutorialsConfig.h>

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
  ErrCode errorCode = I_OK;

private:
  // rosnode
  ros::NodeHandle nh;

  // declare variables
  XCamera *cam;
  XDeviceInformation *devices, *dev;

  cv::Mat thermal_img, color_img, undistort_img, camera_matrix, distortion_coef;
  image_transport::Publisher img_pub_mono, img_pub_color, img_pub_undistort;
  image_transport::ImageTransport it;
  std_msgs::Header header;
  sensor_msgs::ImagePtr msg_thermal_color, msg_thermal_mono, msg_undistort;
  dword frameSize = 0; // The size in bytes of the raw image.
  std::vector<word> framebuffer;
  std::string packname, calibration_file;
  FilterID fltThermography = 0; // Handle to the thermography filter.
  std::vector<double> tempLUT;  // Temperature lookup table (ADU to temperature)
  double *tempLUT_debug;
  const char *newint = "64";
  FilterID histoFlt = 0;

  dynamic_reconfigure::Server<thermal_camera::TutorialsConfig> server;
  dynamic_reconfigure::Server<thermal_camera::TutorialsConfig>::CallbackType f;

  // declare functions here
  void getImage();
  XDeviceInformation *DeviceDescovery();
};

int block, C;
void callback(thermal_camera::TutorialsConfig &config, uint32_t level) {
  block = config.block;
  C = config.C;
}

/**
 * @brief Construct a new Thermal Cam:: Thermal Cam object
 *
 */
ThermalCam::ThermalCam() : it(nh) {
  img_pub_mono = it.advertise("thermal_camera/thermal_img_mono", 1);
  img_pub_color = it.advertise("thermal_camera/thermal_img_color", 1);
  img_pub_undistort = it.advertise("thermal_camera/undistorted_img", 1);
}

/**
 * @brief Loop function
 *
 */
void ThermalCam::loop_function() {

  getImage();
  img_pub_mono.publish(msg_thermal_mono);
  img_pub_color.publish(msg_thermal_color);
  img_pub_undistort.publish(msg_undistort);
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
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
    printf("Unable to create API instance! \n");
    return NULL;
  } else if (cam->IsInitialised() == false) {
    printf("Unable to establish a connection! \n");
    delete cam;
    return NULL;
  } else {
    printf("Connected to device!\n");
    return cam;
  }
}

/**
 * @brief Function to start capturing and activate filters
 *
 */
void ThermalCam::startCap() {
  cam = connectCam();

  // std::string path = ros::package::getPath("thermal_camera");
  // std::cout << "Found package at " << path << std::endl;

  // packname = path + std::string("/config/calibration_5449.xca");
  handle = XC_OpenCamera(dev->url);

  header.frame_id = "map";
  header.stamp = ros::Time(0);
  newint = "64";

  XC_SetPropertyValue(handle, "IntegrationTime", newint, "");

  // errorCode =
  //     XC_LoadCalibration(handle, packname.c_str(),
  //     XLC_StartSoftwareCorrection);
  printf("Start capturing.\n");
  errorCode = XC_StartCapture(handle);

  if (errorCode != I_OK) {
    printf("Could not start capturing, errorCode: %lu\n", errorCode);
  } else {
    printf("Initialization completed!\n");
  }

  /*Reading parameters from intrisic calibartion for image correction */
  std::string calibration_file = path + std::string("/config/ost.yaml");
  std::cout << calibration_file << std::endl;
  FileStorage fs(calibration_file, FileStorage::READ);
  fs["camera_matrix"] >> camera_matrix;
  fs["distortion_coefficients"] >> distortion_coef;
  fs.release();

  std::cout << camera_matrix << std::endl;
  std::cout << distortion_coef << std::endl;
}

/**
 * @brief Function to get frames at each loop
 *
 */
void ThermalCam::getImage() {

  // When the connection is initialised, ...
  if (XC_IsInitialised(handle)) {
    // fltThermography = XC_FLT_Queue(handle, "Thermography", "celsius");
    // // histoFlt = XC_FLT_Queue(handle, "AutoGain", "");
    // if (fltThermography > 0) {

    //   // Build the look-up table and ..
    //   dword mv = XC_GetMaxValue(handle);

    //   tempLUT_debug = new double[mv + 1];

    //   tempLUT.resize(mv + 1);

    //   for (dword x = 0; x < mv + 1; x++) {

    //     double *temp;
    //     XC_FLT_ADUToTemperature(handle, fltThermography, x,
    //                             &tempLUT_debug[x]);
    //     tempLUT[x] = tempLUT_debug[x];
    //   }

    // for (double n : tempLUT) {
    //   std::cout << n << ",";
    // }
    // std::cout << "\n" << std::endl;
    // }

    /***/

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

    cv::Mat color_img = cv::Mat(h, w, CV_16UC1, framebuffer.data());

    color_img.convertTo(color_img, CV_8UC1,
                        1 / 256.0); // convert image to 8bit

    equalizeHist(color_img, color_img);

    // color_img.convertTo(color_img, -1, 1.2,
    //                     0); // increase the contrast by 4

    color_img = cv::Scalar::all(255) - color_img;
    thermal_img = cv::Scalar::all(256 * 256) - thermal_img;
    // cv::threshold(color_img, color_img, 0, 255,
    //               cv::THRESH_BINARY + cv::THRESH_OTSU);

    cv::threshold(color_img, color_img, block, 255, cv::THRESH_BINARY);

    // cv::adaptiveThreshold(color_img, color_img, 255,
    //                       cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY,
    //                       block, C);

    // double inverse_gamma = 1.0 / 0.1;

    // cv::Mat lut_matrix(1, 256, CV_8UC1);
    // uchar *ptr = lut_matrix.ptr();
    // for (int i = 0; i < 256; i++) {
    //   ptr[i] = (int)(pow((double)i / 255.0, inverse_gamma) * 255.0);
    // }

    // cv::Mat result;
    // cv::LUT(color_img, lut_matrix, color_img);
    // color_img.convertTo(color_img, CV_32F);
    // cv::pow(color_img, 0.9, color_img);
    // color_img.convertTo(color_img, CV_8U);
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
    // cv::applyColorMap(color_img, color_img, cv::COLORMAP_AUTUMN);
    cv::undistort(thermal_img, undistort_img, camera_matrix, distortion_coef);

    msg_undistort = cv_bridge::CvImage{header, "mono16", undistort_img}
                        .toImageMsg(); /*convert image to ROS msg*/

    msg_thermal_mono = cv_bridge::CvImage{header, "mono16", thermal_img}
                           .toImageMsg(); /*convert image to ROS msg*/

    msg_thermal_color = cv_bridge::CvImage{header, "mono8", color_img}
                            .toImageMsg(); /*convert image to ROS msg*/
  }
}

/**
 * @brief Function to stop capturing and close camera handle
 *
 */
void ThermalCam::stopCap() {
  XC_StopCapture(handle);
  XC_CloseCamera(handle);

  // delete[] tempLUT;
  delete cam;

  std::cout << "Camera closed" << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ThermalCam");
  ThermalCam reconstruct;

  reconstruct.startCap();

  ros::Rate rate(50);
  while (ros::ok() && reconstruct.errorCode == I_OK) {
    reconstruct.loop_function();

    ros::spinOnce();
    rate.sleep();
  }
  reconstruct.stopCap();

  return 0;
}
