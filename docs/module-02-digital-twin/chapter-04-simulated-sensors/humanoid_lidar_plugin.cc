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
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
  /// \brief A plugin that simulates a LiDAR sensor for humanoid robots
  class HumanoidLidarPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: HumanoidLidarPlugin() {}

    /// \brief Destructor
    public: virtual ~HumanoidLidarPlugin() {}

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that contains the plugin's configuration.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the parent sensor, which should be a ray sensor
      this->parentSensor =
        std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

      if (!this->parentSensor)
      {
        gzerr << "HumanoidLidarPlugin requires a Ray Sensor as its parent.\n";
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

      // Get sensor topic name from SDF, default to "scan"
      std::string topicName = "/humanoid_robot/scan";
      if (_sdf->HasElement("topic"))
        topicName = _sdf->Get<std::string>("topic");

      // Create ROS publisher for laser scan messages
      this->rosPub = this->rosNode->advertise<sensor_msgs::LaserScan>(topicName, 1);

      // Connect to the sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&HumanoidLidarPlugin::OnUpdate, this));

      // Make sure the parent sensor is active
      this->parentSensor->SetActive(true);

      gzmsg << "HumanoidLidarPlugin loaded on topic: " << topicName << std::endl;
    }

    /// \brief Update the plugin during each simulation iteration
    private: void OnUpdate()
    {
      // Get the latest laser scan data from the parent sensor
      msgs::LaserScanStamped laserMsg = this->parentSensor->LaserScan();

      // Create a ROS LaserScan message
      sensor_msgs::LaserScan scanMsg;

      // Fill in the header
      scanMsg.header.stamp = ros::Time::now();
      scanMsg.header.frame_id = this->parentSensor->ParentName(); // Use sensor's parent link name

      // Fill in the laser scan parameters
      scanMsg.angle_min = laserMsg.scan().angle_min();
      scanMsg.angle_max = laserMsg.scan().angle_max();
      scanMsg.angle_increment = laserMsg.scan().angle_step();
      scanMsg.time_increment = 0; // For now, set to 0 (no time between measurements)
      scanMsg.scan_time = 1.0 / this->parentSensor->UpdateRate(); // Time between scans
      scanMsg.range_min = laserMsg.scan().range_min();
      scanMsg.range_max = laserMsg.scan().range_max();

      // Fill in the ranges
      int numRanges = laserMsg.scan().ranges_size();
      scanMsg.ranges.resize(numRanges);

      for (int i = 0; i < numRanges; ++i)
      {
        scanMsg.ranges[i] = laserMsg.scan().ranges(i);
      }

      // Fill in the intensities (if available)
      int numIntensities = laserMsg.scan().intensities_size();
      scanMsg.intensities.resize(numIntensities);

      for (int i = 0; i < numIntensities; ++i)
      {
        scanMsg.intensities[i] = laserMsg.scan().intensities(i);
      }

      // Publish the laser scan message
      this->rosPub.publish(scanMsg);
    }

    /// \brief Pointer to the parent sensor
    private: std::shared_ptr<sensors::RaySensor> parentSensor;

    /// \brief Pointer to the ROS node
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS publisher for laser scan messages
    private: ros::Publisher rosPub;

    /// \brief Connection to the sensor update event
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(HumanoidLidarPlugin)
}