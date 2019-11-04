# gobi284_driver
ROS package to work with a thermal camera Xenics Gobi 384 on Linux


# Installation of Camera Drivers on Linux

First download and install the Xenics Xeneth package: http://support.xenics.com/Support/Linux_SDK_26.zip.

To be able to compile and use these libraries, an older version of libpng is required. Please install the correct version of the library by downloading the package here: https://packages.ubuntu.com/xenial/amd64/libpng12-0/download (64bits).

To test the compilation, run:

```bash
cd /usr/share/xeneth/Sample/LinuxDemo
```

In order to correctly compile the program replace the code on the Makefile with:

```bash
BINNAME= xeneth-demo

CFLAGS= -I/usr/share/xeneth/Include/ -I/usr/local/include/wx-2.8-x22 -I/usr/local/lib/wx/include/gtk2-ansi-release-2.8/
CFLAGS+= -DGTK_NO_CHECK_CASTS -D__WXGTK__ -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES -DNO_GCC_PRAGMA -D__GXX_ABI_VERSION=1002

LNKFLAGS= -lxeneth -lwx_gtk2_xrc-2.8-x22 -lwx_gtk2_qa-2.8-x22 -lwx_gtk2_html-2.8-x22 \
	-lwx_gtk2_adv-2.8-x22 -lwx_gtk2_core-2.8-x22 -lwx_base_xml-2.8-x22 \
	-lwx_base_net-2.8-x22 -lwx_base-2.8-x22

LNKFLAGS+= -ldl -lm -lstdc++

# Sources
SRCS= app_main.cpp frm_main.cpp dlg_camera.cpp
OBJS= $(shell echo $(SRCS) | sed "s/\.cpp/.o/g" |sed "s/\.c/.o/g")

all: $(SRCS) $(BINNAME)

$(BINNAME): $(OBJS)
	gcc $(OBJS) -o $(BINNAME) $(LNKFLAGS)

.c.o:
	gcc $(CFLAGS) -o $@ -c $?

.cpp.o:
	g++ $(CFLAGS) -o $@ -c $?

clean:
	rm -f $(OBJS) $(BINNAME)
  ```
  
Compile with ```make``` and run with ```xeneth-demo```.

In the combo box, select the connected camera and go to **Image -> Start Caputre** and **Filters -> Auto Gain** to start image acquisition and improve threshold settings.


# Installation of ROS drivers

Download the code with:

```bash
cd catkin_ws/src
git clone https://github.com/danifpdra/thermal_camera
```

Connect the camera to your computer through an Ethernet connection. Compile and run. You should be able to visualize the camera image in Rviz!
