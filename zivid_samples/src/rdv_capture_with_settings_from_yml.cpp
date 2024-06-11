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

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <boost/thread.hpp>

ros::Publisher img_pub;
ros::Publisher pc_pub;
bool pub_flag = false;

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
    if (msg->data == "robot_start")
    {
        ROS_INFO("Calling capture");
        capture();
        pub_flag = true;
    }
    else
        pub_flag = false;
}

void dataCallback(
    const boost::shared_ptr<const sensor_msgs::Image>& in_img_msg,
    const boost::shared_ptr<const sensor_msgs::PointCloud2>& in_pc2_msg
)
{
    ROS_INFO("Callback message filter");

    if (pub_flag)
    {
        img_pub.publish(in_img_msg);
        pc_pub.publish(in_pc2_msg);

        ROS_INFO("Image / PCD Publish!!!");
    }
    else
        ROS_INFO("Stop pub");
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
    
    ros::Subscriber plc_sub = nh.subscribe("/rdv_plc/request", 1, plcCallback);

    img_pub = nh.advertise<sensor_msgs::Image>("/zivid_camera/color/image_color", 1);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/zivid_camera/points/xyzrgba", 1);

    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/zivid_camera/color/image_color", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/zivid_camera/points/xyzrgba", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), img_sub, pc_sub);
    sync.registerCallback(boost::bind(&dataCallback, _1, _2));

    ros::waitForShutdown();
    
    return 0;
}