#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "uvc_cam/uvc_cam.h"
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>

#define MAX_AGGREGATED_CAMERAS 10

namespace uvc_camera {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
    void sendInfoJpeg(ros::Time time);
    void feedImages();
    ~Camera();

  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;

    int width, height, fps, skip_frames, frames_to_skip;
	int num_cameras;
    std::string device, frame, format;
    bool rotate;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    ros::Publisher pubjpeg;
    ros::Publisher info_pub;

    uvc_cam::Cam *camera_array[MAX_AGGREGATED_CAMERAS];
    boost::thread image_thread;
};

};

