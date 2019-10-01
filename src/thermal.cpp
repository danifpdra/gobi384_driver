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
//---------Xeneth Camera ------
#include "XCamera.h"

class ThermalCam {
public:
  // publicadores
  ThermalCam();

  // Filter the cloud based on recieved data
  void loop_function();
  XCamera *connectCam();

private:
  // rosnode
  ros::NodeHandle nh;
  // declare functions here

  void getImage();
  XDeviceInformation *DeviceDescovery();
};

ThermalCam::ThermalCam() {}

void ThermalCam::loop_function() {
  // DeviceDescovery();
  getImage();
}

XDeviceInformation *ThermalCam::DeviceDescovery() {
  ErrCode errorCode = I_OK;
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
  // char cDeviceList[1024] = "";

  // Get a list of available devices.
  // XCamera::GetDeviceList(cDeviceList, sizeof(cDeviceList));

  // ROS_WARN("Device List: %s", cDeviceList);

  // XCameraDlg cameraDlg;
  // cameraDlg.Create(cDeviceList);
  // if (cameraDlg.ShowModal() != wxID_OK) return NULL;

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

void ThermalCam::getImage() {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ThermalCam");
  ThermalCam reconstruct;

  reconstruct.connectCam();

  ros::Rate rate(50);
  while (ros::ok()) {
    reconstruct.loop_function();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
