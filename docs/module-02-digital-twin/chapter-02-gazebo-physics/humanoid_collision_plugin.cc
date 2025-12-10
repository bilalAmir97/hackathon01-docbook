#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

namespace gazebo
{
  class HumanoidCollisionPlugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer
      this->model = _model;

      // Initialize ROS if not already initialized
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_collision_detector",
                  ros::init_options::NoSigintHandler);
      }

      // Create ROS node handle
      this->rosNode.reset(new ros::NodeHandle("gazebo_collision_detector"));

      // Create publisher for collision events
      this->collisionPub = this->rosNode->advertise<std_msgs::String>(
          "/humanoid/collisions", 10);

      // Connect to the world update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&HumanoidCollisionPlugin::OnUpdate, this));

      gzdbg << "HumanoidCollisionPlugin loaded for model: "
            << this->model->GetName() << "\n";
    }

    public: void OnUpdate()
    {
      // Check for collisions with each link of the humanoid
      for (unsigned int i = 0; i < this->model->GetChildCount(); ++i)
      {
        physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(
            this->model->GetChild(i));

        if (link)
        {
          // Get collision events for this link
          for (unsigned int j = 0; j < link->GetCollisionCount(); ++j)
          {
            physics::CollisionPtr collision = link->GetCollision(j);

            // Check contact manager for contacts involving this collision
            physics::ContactManager *contactManager =
                this->model->GetWorld()->GetPhysicsEngine()->GetContactManager();

            // Get contacts (simplified approach)
            for (unsigned int k = 0; k < collision->GetShapeCount(); ++k)
            {
              physics::ShapePtr shape = collision->GetShape(k);

              // In a real implementation, we would check the contact manager
              // for actual collision data, but for this educational example
              // we'll just publish a message when the humanoid is on the ground

              // Check if the humanoid is making contact with ground
              // (simplified check based on position)
              math::Pose pose = link->GetWorldPose();
              if (pose.pos.z < 0.1 && link->GetName() == "left_lower_leg" ||
                  link->GetName() == "right_lower_leg")
              {
                // Likely making contact with ground
                std_msgs::String msg;
                std::stringstream ss;
                ss << "Humanoid contact detected on link: " << link->GetName()
                   << " at position: " << pose.pos.x << ", " << pose.pos.y
                   << ", " << pose.pos.z;
                msg.data = ss.str();

                this->collisionPub.publish(msg);
              }
            }
          }
        }
      }
    }

    private: physics::ModelPtr model;
    private: ros::NodeHandlePtr rosNode;
    private: ros::Publisher collisionPub;
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HumanoidCollisionPlugin)
}