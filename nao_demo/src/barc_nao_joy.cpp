/*
 * Nao joystick commands for BARC demos. 
 */

#include <ros/ros.h>
#include <nao_teleop/teleop_nao_joy.h>
#include <nao_components/LEDs.h>
#include <nao_components/BehaviorAction.h>
#include <std_msgs/String.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

/**
 * \brief BARC demo teleoperation via joystick.
 *
 * Subscribes to "joy" messages, and remaps them to Nao demo commands.
 */
class BARCNaoJoy : public nao_teleop::TeleopNaoJoy
{
public:
  BARCNaoJoy();

  void setRGB(nao_components::LEDs & _led, 
	      uint8_t _r, uint8_t _g, uint8_t _b,  ros::Duration _d = ros::Duration(0,0)) const; 

  void publishAllLEDs();
    
protected:
  void joyCallback(const Joy::ConstPtr& joy);
  void setRGB(uint8_t _r, uint8_t _g, uint8_t _b,  ros::Duration _d = ros::Duration(0,0)); 
  void setAllRGB(uint8_t _r, uint8_t _g, uint8_t _b,  ros::Duration _d = ros::Duration(0,0)); 

  void handleLEDCallback(const Joy::ConstPtr& joy);
  void handleLEDCallbackRandom(const Joy::ConstPtr& joy);
  void handleBehaviorCallback(const Joy::ConstPtr& joy);
  void handlePoseCallback(const Joy::ConstPtr& joy);
  

private:
  //Button for allowing LED actions
  int m_modifyLEDsBtn;
  int m_eyesBtn;
  int m_chestBtn;
  int m_earsBtn;
  int m_headBtn;

  //Button for allowing behavior actions
  int m_modifyBehaviorBtn;
  int m_wipeBtn;
  int m_sitDownBtn;
  int m_helloBtn;
  int m_standUpBtn;


  //Button for allowing pose changes
  int m_modifyPoseBtn;
  int m_reachBtn;
  int m_raiseBtn;

  //random number generator
  boost::mt19937 gen;

  ros::Publisher m_ledsPub;
  nao_components::LEDs m_leds;

  ros::Duration m_behaviorTimeOut;
  actionlib::SimpleActionClient<nao_components::BehaviorAction> m_behaviorManagerClient;


  //storage for message for LEDs

  // Based on the assumption that this remains true
  // TODO add an assert to check
  //
  // enum { ledBothEars = 0 };
  // enum { ledBothEyes = 1 };
  // enum { ledHead = 2 };
  // enum { ledChest = 3 };

  nao_components::LEDs m_allLEDs[4];

  void defaultLEDs();


  // LED state
  enum LEDDisplayState { DEFAULT, EXCITEMENT, CURIOSITY, CONCENTRATION, CONTENT };
  LEDDisplayState m_ledState;
  

};

BARCNaoJoy::BARCNaoJoy() : m_modifyLEDsBtn(6), 
			   m_eyesBtn(0), 
			   m_chestBtn(1),
			   m_earsBtn(2),
			   m_headBtn(3), 
			   m_modifyBehaviorBtn(7), 
			   m_wipeBtn(0),
			   m_sitDownBtn(1),
			   m_helloBtn(2),
			   m_standUpBtn(3),
			   m_modifyPoseBtn(4),
			   m_reachBtn(0),
			   m_raiseBtn(3),
			   m_behaviorTimeOut(20.0),
			   m_behaviorManagerClient("behavior_action", true) {

  subscribeToJoystick(&BARCNaoJoy::joyCallback, this);
  m_ledsPub = nh.advertise<nao_components::LEDs>("leds", 10);

  if (!m_behaviorManagerClient.waitForServer(ros::Duration(3.0))){
    ROS_WARN_STREAM("Could not connect to \"behavior_action\" action server, "
		    << "there will be no behaviors available on button presses.\n"
		    << "Is the behaviour_manager node running?");
  }

  // init led messages
  m_allLEDs[nao_components::LEDs::ledBothEars].led = nao_components::LEDs::ledBothEars;
  m_allLEDs[nao_components::LEDs::ledBothEyes].led = nao_components::LEDs::ledBothEyes;
  m_allLEDs[nao_components::LEDs::ledHead].led = nao_components::LEDs::ledHead;
  m_allLEDs[nao_components::LEDs::ledChest].led = nao_components::LEDs::ledChest;

  defaultLEDs();
  m_ledState = DEFAULT;
}


void BARCNaoJoy::setRGB(uint8_t _r, uint8_t _g, uint8_t _b, ros::Duration _d) {
  setRGB(m_leds,_r,_g,_b,_d);
}

void BARCNaoJoy::setRGB(nao_components::LEDs & _leds, uint8_t _r, uint8_t _g, uint8_t _b, ros::Duration _d) const {
  _leds.red = _r;
  _leds.green = _g;
  _leds.blue = _b;
  _leds.duration = _d;
}


void BARCNaoJoy::setAllRGB(uint8_t _r, uint8_t _g, uint8_t _b, ros::Duration _d) {

  // set eyes and chest to RGB
  setRGB(m_allLEDs[nao_components::LEDs::ledBothEyes],_r,_g,_b,_d);    
  setRGB(m_allLEDs[nao_components::LEDs::ledChest],_r,_g,_b,_d);    

  //turn off head and ears as they only do blue
  setRGB(m_allLEDs[nao_components::LEDs::ledBothEars],0,0,0,_d);    
  setRGB(m_allLEDs[nao_components::LEDs::ledHead],0,0,0,_d);    

}

void BARCNaoJoy::handlePoseCallback(const Joy::ConstPtr& joy) {


  std::string pose_name = "";
  
  
  if (buttonTriggered(m_reachBtn, joy) && m_bodyPoseClient.isServerConnected()){
    pose_name = "r_arm_reach";        
  }
  else if (buttonTriggered(m_raiseBtn, joy) && m_bodyPoseClient.isServerConnected()){
    pose_name = "r_arm_raise";        
  }

  if(pose_name != "") {
    callBodyPoseClient(pose_name);
  }  
}


void BARCNaoJoy::handleBehaviorCallback(const Joy::ConstPtr& joy) {


  if(m_behaviorManagerClient.isServerConnected()) {
    nao_components::BehaviorGoal goal;
    goal.behavior_name = "";
    if(buttonTriggered(m_standUpBtn, joy)) {
      ROS_INFO("stand_up");
      goal.behavior_name="stand_up";    
    }
    else if(buttonTriggered(m_sitDownBtn, joy)) {
      ROS_INFO("sit_down");
      goal.behavior_name="sit_down";
    }
    else if(buttonTriggered(m_helloBtn
      , joy)) {
      ROS_INFO("say_hello");
      goal.behavior_name="say_hello";
    }
    else if(buttonTriggered(m_wipeBtn, joy)) {
      ROS_INFO("wipe_brow");
      goal.behavior_name="wipe_brow";
    }
    
    if(goal.behavior_name.length() > 0) {
      m_behaviorManagerClient.sendGoalAndWait(goal, m_behaviorTimeOut);
      actionlib::SimpleClientGoalState state = m_behaviorManagerClient.getState();
      if (state != actionlib::SimpleClientGoalState::SUCCEEDED){
	ROS_ERROR("Behavior \"%s\" did not succeed (%s): %s", goal.behavior_name.c_str(), state.toString().c_str(), state.text_.c_str());
      } else{
	ROS_INFO("Pose action \"%s\" succeeded", goal.behavior_name.c_str());
      }         
    }
  }

}

void BARCNaoJoy::handleLEDCallbackRandom(const Joy::ConstPtr& joy) {
  
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
  
}

void BARCNaoJoy::defaultLEDs() {

  // all blue and full brightness
  for(unsigned int i = 0; i <= nao_components::LEDs::ledChest; i++) {
    setRGB(m_allLEDs[i],0,0,255);    
  }

}




void BARCNaoJoy::publishAllLEDs() {

  for(unsigned int i = 0; i <= nao_components::LEDs::ledChest; i++) {
    m_ledsPub.publish(m_allLEDs[i]);    
  }

}

void BARCNaoJoy::handleLEDCallback(const Joy::ConstPtr& joy) {
  

  // 3 on js, excitement - yellow
  if(buttonTriggered(m_earsBtn, joy)) {
    if(m_ledState == EXCITEMENT) {
      m_ledState = DEFAULT;
    }
    else {
      ROS_DEBUG("excitement - yellow");
      setAllRGB(255,255,0);
      m_ledState = EXCITEMENT;
    }
  }
  // 1 on js, curiosity - violet
  else if(buttonTriggered(m_eyesBtn, joy)) {
    if(m_ledState == CURIOSITY) {
      m_ledState = DEFAULT;
    }
    else {
      ROS_DEBUG("curiosity - violet");
      setAllRGB(143,0,255);
      m_ledState = CURIOSITY;
    }
  }
  // 4 on js, concentration - red
  else if(buttonTriggered(m_headBtn, joy)) {
    if(m_ledState == CONCENTRATION) {
      m_ledState = DEFAULT;
    }
    else {
      ROS_DEBUG("concentration - red");
      setAllRGB(255,0,0);
      m_ledState = CONCENTRATION;
    }
  }
  // 2 on js, content - green
  else if(buttonTriggered(m_chestBtn, joy)) {
    if(m_ledState == CONTENT) {
      m_ledState = DEFAULT;
    }
    else {
      ROS_DEBUG("content - green");
      setAllRGB(0,255,0);    
      m_ledState = CONTENT;
    }
  }  
 
  if(m_ledState == DEFAULT) {
    defaultLEDs();
    ROS_DEBUG("default - blue");
  }
  


}


void BARCNaoJoy::joyCallback(const Joy::ConstPtr& joy) {
  
  initializePreviousJoystick(joy);

  if(m_enabled && buttonPressed(m_modifyLEDsBtn, joy)) {
    handleLEDCallback(joy);
    // update joystick state
    setPreviousJoystick(joy);
  }
  else if(m_enabled && buttonPressed(m_modifyBehaviorBtn, joy)) {
    handleBehaviorCallback(joy);
    // update joystick state
    setPreviousJoystick(joy);
  }
  else if(m_enabled && buttonPressed(m_modifyPoseBtn, joy)) {
    handlePoseCallback(joy);
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

   //led rate doesn't need to ne that quick

   unsigned int count = 0;

   while(teleopNao.nh.ok()){
      ros::spinOnce();

      teleopNao.pubMsg();

      if(count++ % 10 == 0) {
	teleopNao.publishAllLEDs();
      }

      pubRate.sleep();


   }

   return 0;
}
