#include <boost/bind.hpp>
#include <functional>

// #include <gazebo/transport/transport.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <iostream>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std;
namespace gazebo
{
  class CustomMessages : public WorldPlugin
  {
    physics::WorldPtr _world ;
    transport::PublisherPtr  world_control_pub_;
    ros::Publisher  stepped_;
    ros::Subscriber goalSubscriber;
    public : CustomMessages()
    {
      ROS_INFO("v2");
      std::cout << " working "<<std::endl;
      steppedmsg.data=true;
    } 

    public : void StartThread() {
    std::thread  m_member_thread(&CustomMessages::ThreadFunction, this);
    m_member_thread.detach();
    }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {


      // Create a new transport node
      transport::NodePtr gazebonode_(new transport::Node());

      // Initialize the node with the world name
      gazebonode_->Init(_world->Name());

      // Create a publisher on the ~/world_control topic
      this->world_control_pub_ = gazebonode_->Advertise<gazebo::msgs::WorldControl>("~/world_control");

  
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
      


     
      this->node = new ros::NodeHandle(this->GetHandle());
      this->pub =this->node->advertise<rosgraph_msgs::Clock>("custom_Clock", 2, false);
      this->stepped_ =this->node->advertise<std_msgs::Bool>("Stepped", 1, false);
      goalSubscriber=this->node->subscribe<std_msgs::Int32>("stepper",1, &CustomMessages::rosStepperCB ,this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      std::cout<< _world->SimTime() << std::endl;
      this->_world=_world;
      
      // this->StartThread();
      
    }
    public: void ThreadFunction()
    {std::cout << " actual thread "<<std::endl;
    rosgraph_msgs::Clock ros_time_;
      while (1)
      {
      
      ros_time_.clock.fromSec(this->_world->SimTime().Double() );
      this->pub.publish(ros_time_);
      

      }
    }
    public: void OnUpdate()
    {

      
    }
    public: void rosStepperCB(const std_msgs::Int32::ConstPtr& msg)
    {    
      
      auto beg = chrono::high_resolution_clock::now();
 



        gazebo::msgs::WorldControl step_msg;
        auto x =  this->_world->SimTime().Double()  + (msg->data *0.01);

        bool to_pause = (msg->data >= 1);
        step_msg.set_pause(to_pause);

        if (msg->data > 1)
        { // Multi-step:
          step_msg.set_multi_step(msg->data);
        }
        else
        { // One-step:
          step_msg.set_step(msg->data == 1);
        }
        
        world_control_pub_->Publish(step_msg);
        rosgraph_msgs::Clock ros_time_;
        rosgraph_msgs::Clock ros_time_2;

        
        
        
        std::cout<<"the time should be : "<<x <<std::endl;
        while (!(this->_world->SimTime().Double()  >= x-0.009 ))
        {
          std::cout <<"currunt time is "<<this->_world->SimTime().Double() <<std::endl;
          std::this_thread::sleep_for(std::chrono::nanoseconds(200));
        }

        this->stepped_.publish(steppedmsg);
            // Taking a timestamp after the code is ran
        auto end =  chrono::high_resolution_clock::now();

        auto duration =  chrono::duration_cast< chrono::nanoseconds>(end - beg);
        std::cout<<"elapsed time is "<<duration.count() <<std::endl;
    }

    // Pointer to the model
    ros::NodeHandle *node;
    ros::Publisher pub;
    std_msgs::Bool steppedmsg;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(CustomMessages);
}

