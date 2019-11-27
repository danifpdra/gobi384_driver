#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rospack/rospack.h>

#include <gtkmm.h>

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

using namespace std;
using namespace cv;

class TempCalib {
public:
  // publicadores
  TempCalib();
  void loop_function();

private:
  // rosnode
  ros::NodeHandle nh;
  image_transport::Subscriber sub;
  std_msgs::Header header;
  std::string encoding;
  cv::Mat image;
  cv_bridge::CvImageConstPtr cv_ptr;
};

/**
 * @brief Construct a new Thermal Cam:: Thermal Cam object
 *
 */
TempCalib::TempCalib() : it(nh) {
  sub = it.subscribe("thermal_camera/thermal_img_mono", 1,
                     &TempCalib::imageCallback, this);
}

/**
 * @brief Loop function
 *
 */
void TempCalib::loop_function() {}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16);
}

int TempCalib::main(int argc, char **argv) {
  auto app = Gtk::Application::create(argc, argv, "org.gtkmm.example");

  // Load the GtkBuilder file and instantiate its widgets:
  auto refBuilder = Gtk::Builder::create();
  try {
    refBuilder->add_from_file("basic.glade");
  } catch (const Glib::FileError &ex) {
    std::cerr << "FileError: " << ex.what() << std::endl;
    return 1;
  } catch (const Glib::MarkupError &ex) {
    std::cerr << "MarkupError: " << ex.what() << std::endl;
    return 1;
  } catch (const Gtk::BuilderError &ex) {
    std::cerr << "BuilderError: " << ex.what() << std::endl;
    return 1;
  }

  // Get the GtkBuilder-instantiated Dialog:
  refBuilder->get_widget("DialogBasic", pDialog);
  if (pDialog) {
    // Get the GtkBuilder-instantiated Button, and connect a signal handler:
    Gtk::Button *pButton = nullptr;
    refBuilder->get_widget("quit_button", pButton);
    if (pButton) {
      pButton->signal_clicked().connect(sigc::ptr_fun(on_button_clicked));
    }

    app->run(*pDialog);
  }

  delete pDialog;
  return 0;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "TempCalib");
  TempCalib reconstruct;

  ros::Rate rate(50);
  while (ros::ok()) {
    reconstruct.loop_function();
    ros::spinOnce();
    rate.sleep();
  }
  reconstruct.stopCap();

  return 0;
}
