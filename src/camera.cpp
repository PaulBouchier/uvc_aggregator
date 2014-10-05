#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include "uvc_camera/camera.h"

using namespace sensor_msgs;

namespace uvc_camera {

Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) :
      node(_comm_nh), pnode(_param_nh), it(_comm_nh),
      info_mgr(_comm_nh, "camera") {

      /* default config values */
      width = 640;      // width of EACH camera, if aggregated
      height = 480;
      fps = 10;
      skip_frames = 0;
      frames_to_skip = 0;
      device = "/dev/video0";
      num_cameras = 1;
      frame = "camera";
      rotate = false;
      format = "rgb";
      for(int i=0; i<MAX_AGGREGATED_CAMERAS; i++)
        camera_array[i] = 0;

      /* set up information manager */
      std::string url, camera_name;
      pnode.getParam("camera_info_url", url);
      pnode.param<std::string>("camera_name", camera_name, "camera");

      info_mgr.setCameraName(camera_name);
      info_mgr.loadCameraInfo(url);

      /* pull other configuration */
      pnode.getParam("device", device);
      pnode.getParam("num_cameras", num_cameras);

      pnode.getParam("fps", fps);
      pnode.getParam("skip_frames", skip_frames);

      pnode.getParam("width", width);
      pnode.getParam("height", height);

      pnode.getParam("frame_id", frame);

      pnode.getParam("format", format);

      /* advertise image streams and info streams */
      if (format != "jpeg")
        pub = it.advertise("image_raw", 1);
      else
        pubjpeg = node.advertise<CompressedImage>("image_raw/compressed", 1);

      info_pub = node.advertise<CameraInfo>("camera_info", 1);

      /* initialize the cameras */
      uvc_cam::Cam::mode_t mode = uvc_cam::Cam::MODE_RGB;
      if (format == "jpeg")
        mode = uvc_cam::Cam::MODE_MJPG;
      
      uvc_cam::Cam *cam;

      if(num_cameras > 1) {
        ROS_INFO_STREAM("Aggregating " << num_cameras << " cameras");
      }

      for(int i=0; i<num_cameras; i++, device[10]++) {
        ROS_INFO_STREAM("Initializing camera: " << device);
        cam = camera_array[i] = new uvc_cam::Cam(device.c_str(), mode, width, height, fps);
        cam->set_motion_thresholds(100, -1);

        bool auto_focus;
        if (pnode.getParam("auto_focus", auto_focus)) {
          cam->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
        }

        int focus_absolute;
        if (pnode.getParam("focus_absolute", focus_absolute)) {
          cam->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
        }

        bool auto_exposure;
        if (pnode.getParam("auto_exposure", auto_exposure)) {
          int val;
          if (auto_exposure) {
            val = V4L2_EXPOSURE_AUTO;
          } else {
            val = V4L2_EXPOSURE_MANUAL;
          }
          cam->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
        }

        int exposure_absolute;
        if (pnode.getParam("exposure_absolute", exposure_absolute)) {
          cam->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
        }

        int brightness;
        if (pnode.getParam("brightness", brightness)) {
          cam->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
        }

        int power_line_frequency;
        if (pnode.getParam("power_line_frequency", power_line_frequency)) {
          int val;
          if (power_line_frequency == 0) {
            val = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
          } else if (power_line_frequency == 50) {
            val = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
          } else if (power_line_frequency == 60) {
            val = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
          } else {
            printf("power_line_frequency=%d not supported. Using auto.\n", power_line_frequency);
            val = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
          }
          cam->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
        }
      }

      // TODO:
      // - add params for
      //   contrast
      //   saturation
      //   hue
      //   white balance temperature, auto and manual
      //   gamma
      //   sharpness
      //   backlight compensation
      //   exposure auto priority
      //   zoom
      // - add generic parameter list:
      //   [(id0, val0, name0), (id1, val1, name1), ...]


      /* and turn on the streamer */
      ok = true;
      image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
    }

    void Camera::sendInfo(ImagePtr &image, ros::Time time) {
      CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));

      /* Throw out any CamInfo that's not calibrated to this camera mode */
      if (info->K[0] != 0.0 &&
           (image->width != info->width
              || image->height != info->height)) {
        info.reset(new CameraInfo());
      }

      /* If we don't have a calibration, set the image dimensions */
      if (info->K[0] == 0.0) {
        info->width = image->width;
        info->height = image->height;
      }

      info->header.stamp = time;
      info->header.frame_id = frame;

      info_pub.publish(info);
    }

    void Camera::sendInfoJpeg(ros::Time time) {
      CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));
      info->header.stamp = time;
      info->header.frame_id = frame;

      info_pub.publish(info);
    }

    void Camera::feedImages() {
      unsigned int pair_id = 0;
      bool first = true;
      while (ok) {
        unsigned char *img_frame[MAX_AGGREGATED_CAMERAS];
        uint32_t bytes_used;
        int idx[MAX_AGGREGATED_CAMERAS];
        int cam_num;

        for (int cam_num=0; cam_num<MAX_AGGREGATED_CAMERAS; cam_num++) {
            img_frame[cam_num] = NULL;
        }

        ros::Time capture_time = ros::Time::now();

        int cam_image_size;
        for (int cam_num=0; cam_num<num_cameras; cam_num++) {
          idx[cam_num] = camera_array[cam_num]->grab(&img_frame[cam_num], bytes_used);
          if (first) {
            // On first captured images, check that aggregated cameras produce the same size image
            if (cam_num == 0) {
              cam_image_size = bytes_used;
            } else {
              if (cam_image_size != bytes_used) {
                ROS_ERROR_STREAM("Camera aggregation was requested, but the cameras are either jpeg or different sizes - falling back to single-camera, non-aggregated mode");
                num_cameras = 1;
              }
            }
          }
        }

        /* Read in every frame the camera generates, but only send each
         * (skip_frames + 1)th frame. It's set up this way just because
         * this is based on Stereo...
         */
        if (skip_frames == 0 || frames_to_skip == 0) {
          if (img_frame[0] && format != "jpeg") {   // assume if img_frame[0] is valid the rest are too
             // FIXME: indentation is 1 space too many in this & the next block
             ImagePtr image(new Image);

             image->height = height;
             // if num_cameras > 1 we'll aggregate the images side-by-side
             image->width = width * num_cameras;
             image->step = 3 * width * num_cameras;
             image->encoding = image_encodings::RGB8;

             image->header.stamp = capture_time;
             image->header.seq = pair_id;

             image->header.frame_id = frame;

             image->data.resize(image->step * image->height);

             if(num_cameras == 1) {
               // optimize the common case, & also keep it working how it used to for one camera
               memcpy(&image->data[0], img_frame[0], width*height * 3);
             } else {
               int image_x_offset;    // right cam offset in composite image
               int image_xy_offset;   // how far in and down this camera's row is offset
               // aggregate multiple cameras by placing each one's row in the proper place
               // in the aggregate's row, that is, rows in camera-order
               for (int cam_num=0; cam_num<num_cameras; cam_num++) {
                 image_x_offset = cam_num * width * 3;    // right cam offset
                 for (int row_num=0; row_num<height; row_num++) {
                   image_xy_offset = row_num * (image->step) + image_x_offset;
                   memcpy(&(image->data[image_xy_offset]), 
                            img_frame[cam_num] + row_num * width * 3, 
                            width * 3);
                 }
               }
             }

             pub.publish(image);

             sendInfo(image, capture_time);

             ++pair_id;
          } else if (img_frame[0] && format == "jpeg") {
             CompressedImagePtr image(new CompressedImage);

             image->header.stamp = capture_time;
             image->header.seq = pair_id;

             image->header.frame_id = frame;

             image->data.resize(bytes_used);

             // don't attempt to aggregate jpeg camera images
             memcpy(&image->data[0], img_frame[0], bytes_used);

             pubjpeg.publish(image);

             sendInfoJpeg(capture_time);

             ++pair_id;
          }

          frames_to_skip = skip_frames;
          first = false;
        } else {
          frames_to_skip--;
        }

        for (int cam_num=0; cam_num<num_cameras; cam_num++) {
          if (img_frame[cam_num]) camera_array[cam_num]->release(idx[cam_num]);
        }
      }
    }

    Camera::~Camera() {
      ok = false;
      image_thread.join();
      for(int cam_num=0; cam_num<MAX_AGGREGATED_CAMERAS; cam_num++) {
          if (camera_array[cam_num]) delete camera_array[cam_num];
      }
    }


};

