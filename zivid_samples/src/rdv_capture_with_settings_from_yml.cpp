#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

namespace
{
const ros::Duration default_wait_duration{ 30 };

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

}  // namespace

void plcCallback(const std_msgs::String::ConstPtr& msg)
{
	// TODO: plc 메세지에 따라서 변경 필요
	ROS_INFO("Calling capture");
	capture();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rdv_capture_with_settings_from_yml_cpp");
  ros::NodeHandle nh;

  ROS_INFO("Starting rdv_capture_with_settings_from_yml.cpp");

  CHECK(ros::service::waitForService("/zivid_camera/capture", default_wait_duration));

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string samples_path = ros::package::getPath("zivid_samples");
  std::string settings_path = samples_path + "/settings/camera_settings.yml";
  ROS_INFO("Loading settings from: %s", settings_path.c_str());
  zivid_camera::LoadSettingsFromFile load_settings_from_file;
  load_settings_from_file.request.file_path = settings_path;
  CHECK(ros::service::call("/zivid_camera/load_settings_from_file", load_settings_from_file));

  ros::Subscriber plc_sub = nh.subscribe("/rdv_pcl/signal", 1, plcCallback);

  ros::waitForShutdown();

  return 0;
}