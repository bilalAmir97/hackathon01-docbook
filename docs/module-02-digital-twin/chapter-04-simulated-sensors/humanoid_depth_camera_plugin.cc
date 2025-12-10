#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <sdf/sdf.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>

#include <thread>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace gazebo
{
  /// \brief A plugin that simulates a depth camera sensor for humanoid robots
  class HumanoidDepthCameraPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: HumanoidDepthCameraPlugin() {}

    /// \brief Destructor
    public: virtual ~HumanoidDepthCameraPlugin() {}

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that contains the plugin's configuration.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the parent sensor, which should be a depth camera sensor
      this->parentSensor =
        std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);

      if (!this->parentSensor)
      {
        gzerr << "HumanoidDepthCameraPlugin requires a DepthCamera Sensor as its parent.\n";
        return;
      }

      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("humanoid_robot"));

      // Get sensor topic names from SDF, with defaults
      std::string rgbTopicName = "/humanoid_robot/camera/image_raw";
      std::string depthTopicName = "/humanoid_robot/camera/depth/image_raw";
      std::string pointCloudTopicName = "/humanoid_robot/camera/depth/points";
      std::string cameraInfoTopicName = "/humanoid_robot/camera/camera_info";

      if (_sdf->HasElement("rgb_topic"))
        rgbTopicName = _sdf->Get<std::string>("rgb_topic");
      if (_sdf->HasElement("depth_topic"))
        depthTopicName = _sdf->Get<std::string>("depth_topic");
      if (_sdf->HasElement("pointcloud_topic"))
        pointCloudTopicName = _sdf->Get<std::string>("pointcloud_topic");
      if (_sdf->HasElement("camera_info_topic"))
        cameraInfoTopicName = _sdf->Get<std::string>("camera_info_topic");

      // Create ROS publishers
      this->imagePub = this->rosNode->advertise<sensor_msgs::Image>(rgbTopicName, 1);
      this->depthImagePub = this->rosNode->advertise<sensor_msgs::Image>(depthTopicName, 1);
      this->pointCloudPub = this->rosNode->advertise<sensor_msgs::PointCloud2>(pointCloudTopicName, 1);
      this->cameraInfoPub = this->rosNode->advertise<sensor_msgs::CameraInfo>(cameraInfoTopicName, 1);

      // Connect to the sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&HumanoidDepthCameraPlugin::OnUpdate, this));

      // Make sure the parent sensor is active
      this->parentSensor->SetActive(true);

      gzmsg << "HumanoidDepthCameraPlugin loaded with topics:" << std::endl;
      gzmsg << "  RGB: " << rgbTopicName << std::endl;
      gzmsg << "  Depth: " << depthTopicName << std::endl;
      gzmsg << "  PointCloud: " << pointCloudTopicName << std::endl;
      gzmsg << "  CameraInfo: " << cameraInfoTopicName << std::endl;
    }

    /// \brief Update the plugin during each simulation iteration
    private: void OnUpdate()
    {
      // Get the latest depth camera data
      this->parentSensor->DepthCamera()->Render();

      // Get image data
      unsigned int width = this->parentSensor->ImageWidth();
      unsigned int height = this->parentSensor->ImageHeight();

      // Get RGB image data
      const unsigned char *rgbData = this->parentSensor->ImageData();

      // Get depth data
      float *depthData = this->parentSensor->DepthData();

      // Create and publish RGB image
      sensor_msgs::Image rgbMsg;
      rgbMsg.header.stamp = ros::Time::now();
      rgbMsg.header.frame_id = this->parentSensor->ParentName();
      rgbMsg.width = width;
      rgbMsg.height = height;
      rgbMsg.encoding = "rgb8";
      rgbMsg.is_bigendian = 0;
      rgbMsg.step = width * 3; // 3 bytes per pixel for RGB
      rgbMsg.data.resize(width * height * 3);

      // Copy RGB data
      std::copy(rgbData, rgbData + width * height * 3, rgbMsg.data.begin());

      this->imagePub.publish(rgbMsg);

      // Create and publish depth image
      sensor_msgs::Image depthMsg;
      depthMsg.header.stamp = rgbMsg.header.stamp;
      depthMsg.header.frame_id = rgbMsg.header.frame_id;
      depthMsg.width = width;
      depthMsg.height = height;
      depthMsg.encoding = "32FC1"; // 32-bit float per pixel
      depthMsg.is_bigendian = 0;
      depthMsg.step = width * sizeof(float);
      depthMsg.data.resize(width * height * sizeof(float));

      // Copy depth data
      memcpy(&depthMsg.data[0], depthData, width * height * sizeof(float));

      this->depthImagePub.publish(depthMsg);

      // Publish camera info (simplified)
      sensor_msgs::CameraInfo camInfo;
      camInfo.header = rgbMsg.header;
      camInfo.width = width;
      camInfo.height = height;
      // Set default camera matrix parameters (these should be configured based on your camera)
      camInfo.K = {500.0, 0.0, width/2.0, 0.0, 500.0, height/2.0, 0.0, 0.0, 1.0};
      camInfo.P = {500.0, 0.0, width/2.0, 0.0, 0.0, 500.0, height/2.0, 0.0, 0.0, 0.0, 1.0, 0.0};

      this->cameraInfoPub.publish(camInfo);
    }

    /// \brief Pointer to the parent sensor
    private: std::shared_ptr<sensors::DepthCameraSensor> parentSensor;

    /// \brief Pointer to the ROS node
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS publishers for different data streams
    private: ros::Publisher imagePub;
    private: ros::Publisher depthImagePub;
    private: ros::Publisher pointCloudPub;
    private: ros::Publisher cameraInfoPub;

    /// \brief Connection to the sensor update event
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(HumanoidDepthCameraPlugin)
}