#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{

  

  ros::spinOnce();

  // Get the parent sensor.
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
/*
  std::cout << " Collision between[" << contacts.contact(0).collision1()
              << "] and [" << contacts.contact(0).collision2() << "]\n";

  std::cout << " Collision between[" << contacts.contact(1).collision1()
              << "] and [" << contacts.contact(1).collision2() << "]\n";*/


  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::cout << i << " Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";

    std_msgs::String msg;

      std::stringstream force_x;
      std::stringstream force_y;
      std::stringstream force_z;
      
      force_x <<  contacts.contact(i).wrench(0).body_2_wrench().force().x();
      msg.data = force_x.str();
      chatter_pub.publish(msg);

      std::cout << i << "HEEEEY! " << contacts.contact(i).wrench(0).body_2_wrench().force().x() << "\n";
  }

}
