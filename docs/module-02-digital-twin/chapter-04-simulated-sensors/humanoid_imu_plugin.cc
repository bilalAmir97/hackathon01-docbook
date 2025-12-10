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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
  /// \brief A plugin that simulates an IMU sensor for humanoid robots
  class HumanoidImuPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: HumanoidImuPlugin() {}

    /// \brief Destructor
    public: virtual ~HumanoidImuPlugin() {}

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that contains the plugin's configuration.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the parent sensor, which should be an IMU sensor
      this->parentSensor =
        std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);

      if (!this->parentSensor)
      {
        gzerr << "HumanoidImuPlugin requires an IMU Sensor as its parent.\n";
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

      // Get sensor topic name from SDF, default to "imu/data"
      std::string topicName = "/humanoid_robot/imu/data";
      if (_sdf->HasElement("topic"))
        topicName = _sdf->Get<std::string>("topic");

      // Create ROS publisher for IMU messages
      this->rosPub = this->rosNode->advertise<sensor_msgs::Imu>(topicName, 1);

      // Connect to the sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&HumanoidImuPlugin::OnUpdate, this));

      // Make sure the parent sensor is active
      this->parentSensor->SetActive(true);

      gzmsg << "HumanoidImuPlugin loaded on topic: " << topicName << std::endl;
    }

    /// \brief Update the plugin during each simulation iteration
    private: void OnUpdate()
    {
      // Get the latest IMU data from the parent sensor
      ignition::math::Pose3d pose = this->parentSensor->Pose();
      ignition::math::Vector3d angularVel = this->parentSensor->AngularVelocity();
      ignition::math::Vector3d linearAcc = this->parentSensor->LinearAcceleration();

      // Create a ROS IMU message
      sensor_msgs::Imu imuMsg;

      // Fill in the header
      imuMsg.header.stamp = ros::Time::now();
      imuMsg.header.frame_id = this->parentSensor->ParentName(); // Use sensor's parent link name

      // Fill in the orientation (convert from ignition math to ROS quaternion)
      // Note: This is a simplified approach - in practice, you might need to handle
      // coordinate system conversions between Gazebo and ROS
      imuMsg.orientation.x = pose.Rot().X();
      imuMsg.orientation.y = pose.Rot().Y();
      imuMsg.orientation.z = pose.Rot().Z();
      imuMsg.orientation.w = pose.Rot().W();

      // Fill in orientation covariance (set to 0 if not available)
      imuMsg.orientation_covariance[0] = -1; // Set to -1 to indicate that the orientation is not known

      // Fill in angular velocity
      imuMsg.angular_velocity.x = angularVel.X();
      imuMsg.angular_velocity.y = angularVel.Y();
      imuMsg.angular_velocity.z = angularVel.Z();

      // Fill in angular velocity covariance (set to 0 if not available)
      for (int i = 0; i < 9; i++)
        imuMsg.angular_velocity_covariance[i] = 0.0;

      // Fill in linear acceleration
      imuMsg.linear_acceleration.x = linearAcc.X();
      imuMsg.linear_acceleration.y = linearAcc.Y();
      imuMsg.linear_acceleration.z = linearAcc.Z();

      // Fill in linear acceleration covariance (set to 0 if not available)
      for (int i = 0; i < 9; i++)
        imuMsg.linear_acceleration_covariance[i] = 0.0;

      // Publish the IMU message
      this->rosPub.publish(imuMsg);
    }

    /// \brief Pointer to the parent sensor
    private: std::shared_ptr<sensors::ImuSensor> parentSensor;

    /// \brief Pointer to the ROS node
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief ROS publisher for IMU messages
    private: ros::Publisher rosPub;

    /// \brief Connection to the sensor update event
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(HumanoidImuPlugin)
}