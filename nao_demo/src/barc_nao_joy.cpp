/*
 * Nao joystick commands for BARC demos. 
 */

#include <ros/ros.h>
#include <nao_teleop/teleop_nao_joy.h>
#include <nao_components/LEDs.h>
#include <std_msgs/String.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

/**
 * \brief BARC demo teleoperation via joystick.
 *
 * Subscribes to "joy" messages, and remaps them to Nao demo commands.
 */
class BARCNaoJoy : public TeleopNaoJoy
{
public:
  BARCNaoJoy();

protected:
  void joyCallback(const Joy::ConstPtr& joy);
  void setRGB(uint8_t _r, uint8_t _g, uint8_t _b,  ros::Duration _d = ros::Duration(0,0)); 

private:
  //Button for allowing LED action
  int m_modifyLEDsBtn;
  int m_eyesBtn;
  int m_earsBtn;
  int m_headBtn;
  int m_chestBtn;

  //random number generator
  boost::mt19937 gen;

  ros::Publisher m_ledsPub;

  nao_components::LEDs m_leds;

};

BARCNaoJoy::BARCNaoJoy() : m_modifyLEDsBtn(6), 
			   m_eyesBtn(0), 
			   m_earsBtn(2),
			   m_headBtn(3), 
			   m_chestBtn(1) {
  subscribeToJoystick(&BARCNaoJoy::joyCallback, this);
  m_ledsPub = nh.advertise<nao_components::LEDs>("leds", 10);
}

void BARCNaoJoy::setRGB(uint8_t _r, uint8_t _g, uint8_t _b, ros::Duration _d) {
  m_leds.red = _r;
  m_leds.green = _g;
  m_leds.blue = _b;
  m_leds.duration = _d;
}

void BARCNaoJoy::joyCallback(const Joy::ConstPtr& joy) {
  
  initializePreviousJoystick(joy);

  if(m_enabled && buttonPressed(m_modifyLEDsBtn, joy)) {

    boost::uniform_int<> dist(0, 255);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > rand(gen, dist);

    if(buttonTriggered(m_earsBtn, joy)) {
      ROS_INFO("ears");
      //ears can only be blue
      setRGB(0,0,rand());
      m_leds.led = nao_components::LEDs::ledBothEars;
      m_ledsPub.publish(m_leds);
    }
    else if(buttonTriggered(m_eyesBtn, joy)) {
      ROS_INFO("eyes");
      setRGB(rand(),rand(),rand());
      m_leds.led = nao_components::LEDs::ledBothEyes;
      m_ledsPub.publish(m_leds);
    }
    else if(buttonTriggered(m_headBtn, joy)) {
      ROS_INFO("head");
      //head can only be blue
      setRGB(0,0,rand());
      m_leds.led = nao_components::LEDs::ledHead;
      m_ledsPub.publish(m_leds);
    }
    else if(buttonTriggered(m_chestBtn, joy)) {
      ROS_INFO("chest");
      setRGB(rand(),rand(),rand());
      m_leds.led = nao_components::LEDs::ledChest;
      m_ledsPub.publish(m_leds);
    }

    // update joystick state
    setPreviousJoystick(joy);
  }
  else {
    TeleopNaoJoy::joyCallback(joy);
  }
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "teleop_nao");
   BARCNaoJoy teleopNao;

   // rate of publishing motion commands (too high stresses the Nao's CPU)
   double publishRate = 10.0;
   teleopNao.privateNh.param("motion_publish_rate", publishRate, publishRate);
   ros::Rate pubRate(publishRate);

   while(teleopNao.nh.ok()){
      ros::spinOnce();

      teleopNao.pubMsg();
      pubRate.sleep();
   }

   return 0;
}
