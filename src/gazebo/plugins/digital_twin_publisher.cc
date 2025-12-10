#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <gazebo/util/system.hh>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{
  class DigitalTwinPublisher : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for later use
      this->model = _model;

      // Initialize ROS if it hasn't been initialized already
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_streamer",
                 ros::init_options::NoSigintHandler);
      }

      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("gazebo"));

      // Create publisher for robot state
      this->pub = this->rosNode->advertise<geometry_msgs::Pose>(
          "/gazebo/robot_pose", 1);

      // Listen to the update event (this gets called every simulation iteration)
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DigitalTwinPublisher::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Get the current pose of the model
      ignition::math::Pose3d pose = this->model->WorldPose();

      // Convert to ROS message
      geometry_msgs::Pose pose_msg;
      pose_msg.position.x = pose.Pos().X();
      pose_msg.position.y = pose.Pos().Y();
      pose_msg.position.z = pose.Pos().Z();
      pose_msg.orientation.x = pose.Rot().X();
      pose_msg.orientation.y = pose.Rot().Y();
      pose_msg.orientation.z = pose.Rot().Z();
      pose_msg.orientation.w = pose.Rot().W();

      // Publish the pose
      this->pub.publish(pose_msg);
    }

    private: physics::ModelPtr model;
    private: ros::Publisher pub;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DigitalTwinPublisher)
}