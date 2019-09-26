//ros package to communicate with thermal camera


#include <ros/ros.h>
#include <ros/console.h>
//-----------------
#include <math.h>
#include <algorithm> // std::max
#include <cmath>
#include <fstream>
#include <iostream>
//---------Xeneth Camera ------
#include "XCamera.h"



class ThermalCam
{
public:
  // publicadores
  ThermalCam();

  // Filter the cloud based on recieved data
  void loop_function();

private:
    //declare functions here
    void connectCam();
    void getImage();
};


ThermalCam::ThermalCam()
{
  
}

void ThermalCam::loop_function()
{
    getImage();
}

XCamera* ThermalCam::connectCam()
{
    char cDeviceList[1024]="";

    // Get a list of available devices.
    XCamera::GetDeviceList(cDeviceList, sizeof(cDeviceList));

    XCameraDlg cameraDlg;
    cameraDlg.Create(cDeviceList);
    if (cameraDlg.ShowModal() != wxID_OK) return NULL;

    XCamera* cam = XCamera::Create(m_deviceSelection, NULL, NULL);

    if(cam == NULL)
    {
        ROS_DEBUG("Unable to create API instance");
        return NULL;
    }

    if(cam->IsInitialised() == false)
    {
        ROS_DEBUG("Unable to establish a connection");
        delete cam;
        return NULL;
    }

    return (cam);
}

void ThermalCam::getImage()
{

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ThermalCam");
  ThermalCam reconstruct;

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.loop_function();

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
