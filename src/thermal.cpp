//ros package to communicate with thermal camera


#include <ros/ros.h>
#include <ros/console.h>
//-----------------
#include <math.h>
#include <algorithm> // std::max
#include <cmath>
#include <fstream>
#include <iostream>
#include "stdio.h" 
//---------Xeneth Camera ------
#include "XCamera.h"



class ThermalCam
{
public:
  // publicadores
  ThermalCam();

  // Filter the cloud based on recieved data
  void loop_function();
  XCamera* connectCam();

private:

  //rosnode 
  ros::NodeHandle nh;
    //declare functions here
    
    void getImage();
    void DeviceDescovery();
};


ThermalCam::ThermalCam()
{
  
}

void ThermalCam::loop_function()
{
  DeviceDescovery();
  getImage();
}

void ThermalCam::DeviceDescovery()
{
    ErrCode errorCode = I_OK;
    unsigned int deviceCount = 0;

    if ((errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll)) != I_OK) {
        printf("An error occurred while enumerating the devices. errorCode: %i\n", errorCode);
    }

    if (deviceCount == 0) {
        printf("Enumeration was a success but no devices were found!\n");
    }

    /*  At this point we know how much devices are present in the environment.
     *  Now allocate the XDeviceInformation array to accommodate all the discovered devices using
     *  the device count to determine the size needed. Once allocated we call the enumerate
     *  devices method again but now instead of passing null as the initial argument use the new
     *  allocated buffer. For the flags argument we no longer use a protocol enable flag but make use
     *  of the XEF_UseCached flag. On discovery devices are cached internally and as such we are able to 
     *  retrieve the device information structures instantly when calling XCD_EnumerateDevices for a second time.
     *  Note that it is not required to first check device count, allocate structure and retrieve cached devices.
     *  A user could allocate one or more device structure and immediately pass this with the initial call to XCD_EnumerateDevices.
     *  XCD_EnumerateDevices will not exceed the supplied deviceCount and when less devices were discovered than the initial deviceCount 
     *  this argument is updated with the new count. */

    XDeviceInformation *devices = new XDeviceInformation[deviceCount];
    if ((errorCode = XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached)) != I_OK) {
        printf("Error while retrieving the cached device information structures. errorCode: %i\n", errorCode);
        delete [] devices;
    }

    /*  All discovered devices are now available in our local array and we are now able 
     *  to iterate the list and output each item in the array */

    for(unsigned int i = 0; i < deviceCount; i++) {
        XDeviceInformation * dev = &devices[i];
        printf("device[%lu] %s @ %s (%s) \n", i, dev->name, dev->address, dev->transport);
        printf("PID: %4X\n", dev->pid); 
        printf("Serial: %lu\n", dev->serial);
        printf("URL: %s\n", dev->url);
        printf("State: %s\n\n", dev->state == XDS_Available ? "Available" : dev->state == XDS_Busy ? "Busy" : "Unreachable");
    }
}

XCamera* ThermalCam::connectCam()
{
    char cDeviceList[1024]="";

    // Get a list of available devices.
    // XCamera::GetDeviceList(cDeviceList, sizeof(cDeviceList));

    // ROS_WARN("Device List: %s", cDeviceList);

    // XCameraDlg cameraDlg;
    // cameraDlg.Create(cDeviceList);
    // if (cameraDlg.ShowModal() != wxID_OK) return NULL;

    // XCamera* cam = XCamera::Create(cDeviceList, NULL, NULL);

    // if(cam == NULL)
    // {
    //     ROS_DEBUG("Unable to create API instance");
    //     return NULL;
    // }

    // if(cam->IsInitialised() == false)
    // {
    //     ROS_DEBUG("Unable to establish a connection");
    //     delete cam;
    //     return NULL;
    // }

    return NULL;
}

void ThermalCam::getImage()
{

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ThermalCam");
  ThermalCam reconstruct;

  reconstruct.connectCam();

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.loop_function();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
