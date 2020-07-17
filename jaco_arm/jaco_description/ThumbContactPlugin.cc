#include "ThumbContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ThumbContactPlugin)

/////////////////////////////////////////////////
ThumbContactPlugin::ThumbContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ThumbContactPlugin::~ThumbContactPlugin()
{
}

/////////////////////////////////////////////////
void ThumbContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{

  

  ros::spinOnce();

  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ThumbContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ThumbContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ThumbContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();

  float sum = 0;

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
      sum = sum + contacts.contact(i).wrench(0).body_2_wrench().force().x();   
  }


  std_msgs::Int16 msg;

  float force_x;
  
  if(sum == 0){
    force_x = 0;
  }
  
  else{
    force_x = (sum/contacts.contact_size());
  }

  int hap;

  if (force_x == 0){
    hap = 0;
  }
  else{
    hap = force_x*255/130;
  }

  msg.data = abs(hap);
  chatter_pub.publish(msg);

}
