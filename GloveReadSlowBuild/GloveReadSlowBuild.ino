#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

std_msgs::Int16 thumb_vals;
std_msgs::Int16 pointer_vals;
std_msgs::Int16 middle_vals;

//--------THUMB---------------//

void hapticThumbCB( const std_msgs::Int16& msg){
  analogWrite(3, 255);
  digitalWrite(13, HIGH);  
}

void hapticThumbOFFCB( const std_msgs::Empty& toggle_msg){
  analogWrite(3, 0);
}

ros::Subscriber<std_msgs::Int16> thumbHaptic("haptic_thumb", &hapticThumbCB ); 
ros::Subscriber<std_msgs::Empty> thumbHapticOFF("haptic_thumb_OFF", &hapticThumbOFFCB );


//---------POINTER----------//

void hapticPointerCB( const std_msgs::Int16& msg){
  analogWrite(5, msg.data);
}

void hapticPointerOFFCB( const std_msgs::Empty& toggle_msg){
  analogWrite(5, 0);
}

ros::Subscriber<std_msgs::Int16> pointerHaptic("haptic_pointer", &hapticPointerCB ); 
ros::Subscriber<std_msgs::Empty> pointerHapticOFF("haptic_pointer_OFF", &hapticPointerOFFCB ); 


//---------MIDDLE------------//

void hapticMiddleCB( const std_msgs::Int16& msg){
  analogWrite(9, 255);
}

void hapticMiddleOFFCB( const std_msgs::Empty& toggle_msg){
  analogWrite(9, 0);
}

ros::Subscriber<std_msgs::Int16> middleHaptic("haptic_middle", &hapticMiddleCB ); 
ros::Subscriber<std_msgs::Empty> middleHapticOFF("haptic_middle_OFF", &hapticMiddleOFFCB );

//--------CURVATURE SENSING------------//
ros::Publisher thumbPos("thumbPos", &thumb_vals);
ros::Publisher pointerPos("pointerPos", &pointer_vals);
ros::Publisher middlePos("middlePos", &middle_vals);

void setup()
{
  pinMode(13, OUTPUT);
  
  //analogWriteFrequency(3, 60);
  //analogWriteFrequency(5, 60);
  //analogWriteFrequency(9, 60);
  
  nh.initNode();
  
  nh.subscribe(thumbHaptic);
  nh.subscribe(thumbHapticOFF);

  nh.subscribe(pointerHaptic);
  nh.subscribe(pointerHapticOFF);

  nh.subscribe(middleHaptic);
  nh.subscribe(middleHapticOFF);

  nh.advertise(thumbPos);
  nh.advertise(pointerPos);
  nh.advertise(middlePos);

}

void loop()
{ 
  int thumbVal = analogRead(A0);
  thumb_vals.data = thumbVal;
  thumbPos.publish( &thumb_vals );

  int pointerVal = analogRead(A1);
  pointer_vals.data = pointerVal;
  pointerPos.publish( &pointer_vals );

  int middleVal = analogRead(A2);
  middle_vals.data = middleVal;
  middlePos.publish( &middle_vals );

  nh.spinOnce();
  delay(25);
}
