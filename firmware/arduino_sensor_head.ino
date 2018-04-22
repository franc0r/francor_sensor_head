#include "Arduino.h"

#include <Servo.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <francor_msgs/SensorHeadCmd.h>

class SensorHead{
public:
  SensorHead()
  {
    _pan_speed = 0;
    _tilt_speed = 0;
  }
  void attach(const int pin_pan_servo, const int pin_tilt_servo)
  {
    _pan.attach(pin_pan_servo);
    _tilt.attach(pin_tilt_servo);
  }

  /**
   * @brief Set the Pos object
   * 
   * @param pan_pos  -> min = -90, max = +90
   * @param tilt_pos -> min = -90, max = +90
   */
  void setPos(const int pan_pos, const int tilt_pos)
  {
    _pan_speed  = 0;
    _tilt_speed = 0;
    _curr_pan_pos  = ::map(pan_pos, -90, 90, 500, 2500);
    _curr_tilt_pos = ::map(tilt_pos, -90, 90, 500, 2500);

    this->write(_curr_pan_pos, _curr_tilt_pos);
  }
  void setSpeed(const int pan_speed, const int tilt_speed)
  {
    _pan_speed  = pan_speed;
    _tilt_speed = tilt_speed;
  }

  void tick()
  {
    _curr_pan_pos  += _pan_speed;
    _curr_tilt_pos += _tilt_speed;

    _curr_pan_pos = constrain(_curr_pan_pos, 500, 2500);
    _curr_tilt_pos = constrain(_curr_tilt_pos, 500, 2500);

    this->write(_curr_pan_pos, _curr_tilt_pos);
  }

  francor_msgs::SensorHeadCmd getPos() const
  {
    francor_msgs::SensorHeadCmd cmd;
    cmd.pan  = ::map(_curr_pan_pos, 500, 2500, -90, 90);
    cmd.tilt = ::map(_curr_tilt_pos, 500, 2500, -90, 90);
    return cmd;
  }

  francor_msgs::SensorHeadCmd getSpeed() const
  {
    francor_msgs::SensorHeadCmd cmd;
    cmd.pan  = _pan_speed;
    cmd.tilt = _tilt_speed;
    return cmd;
  }

private:

  void write(const int pan_pos, const int tilt_pos)
  {
    _pan.writeMicroseconds(pan_pos);
    _tilt.writeMicroseconds(tilt_pos);
  }

  Servo _pan;
  Servo _tilt;

  int _pan_speed;
  int _tilt_speed;

  int _curr_pan_pos;
  int _curr_tilt_pos;
};


SensorHead g_sensor_head;

ros::NodeHandle  nh;
std_msgs::String log_msg;
francor_msgs::SensorHeadCmd sensor_head_pos;
ros::Publisher pub_log("sensor_head/log", &log_msg);
ros::Publisher pub_pos("sensor_head/pos", &sensor_head_pos);

// -- functions --

void sub_set_speed_callback(const francor_msgs::SensorHeadCmd& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
  log_msg.data = "got set_speed_callback()";
  pub_log.publish(&log_msg);
  g_sensor_head.setSpeed(msg.pan, msg.tilt);
}

void sub_set_pos_callback(const francor_msgs::SensorHeadCmd& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
  log_msg.data = "got set_pos_callback()";
  pub_log.publish(&log_msg);

  g_sensor_head.setPos(msg.pan, msg.tilt);

}

// -- subs --

ros::Subscriber<francor_msgs::SensorHeadCmd> sub_set_speed("sensor_head/set_speed", &sub_set_speed_callback);
ros::Subscriber<francor_msgs::SensorHeadCmd> sub_set_pos("sensor_head/set_pos", &sub_set_pos_callback);


void setup()
{
  // Serial.begin(9600);

  g_sensor_head.attach(6, 4);
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_log);
  nh.advertise(pub_pos);
  nh.subscribe(sub_set_speed);
  nh.subscribe(sub_set_pos);

  g_sensor_head.setPos(0,0);
  
}

void loop()
{
  g_sensor_head.tick();

  //pub current pos
  francor_msgs::SensorHeadCmd cmd = g_sensor_head.getPos();
  sensor_head_pos.pan  = cmd.pan;
  sensor_head_pos.tilt = cmd.tilt;
  pub_pos.publish(&sensor_head_pos);

  nh.spinOnce();

  delay(10);
}

