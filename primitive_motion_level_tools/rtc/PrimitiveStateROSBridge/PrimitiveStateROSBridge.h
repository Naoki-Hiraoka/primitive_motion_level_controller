#ifndef PrimitiveStateROSBridge_H
#define PrimitiveStateROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <primitive_motion_level_msgs/idl/PrimitiveState.hh>

#include <ros/ros.h>
#include <primitive_motion_level_msgs/PrimitiveStateArray.h>
#include <primitive_motion_level_msgs/PrimitiveStateArrayArray.h>

#include <urdf/model.h>
#include <cnoid/Body>

class PrimitiveStateROSBridge : public RTC::DataFlowComponentBase{
protected:
  std::shared_ptr<urdf::Model> robot_urdf_;
  cnoid::BodyPtr robot_vrml_;

  ros::NodeHandle nh; // これがないとうまく通信できなくなったり、CPU使用率100%になったりする

  primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandRTM_;
  RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandRTMIn_;
  ros::Publisher pub_;

  ros::Subscriber sub_;
  primitive_motion_level_msgs::TimedPrimitiveStateSeq m_primitiveCommandROS_;
  RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeq> m_primitiveCommandROSOut_;

  primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq m_primitiveCommandSeqRTM_;
  RTC::InPort <primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq> m_primitiveCommandSeqRTMIn_;
  ros::Publisher seqPub_;

  ros::Subscriber seqSub_;
  primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq m_primitiveCommandSeqROS_;
  RTC::OutPort <primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq> m_primitiveCommandSeqROSOut_;
public:
  PrimitiveStateROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  void topicCallback(const primitive_motion_level_msgs::PrimitiveStateArray::ConstPtr& msg);
  void seqTopicCallback(const primitive_motion_level_msgs::PrimitiveStateArrayArray::ConstPtr& msg);

protected:
  static void primitiveStateIdl2Msg(const primitive_motion_level_msgs::PrimitiveStateIdl& in, primitive_motion_level_msgs::PrimitiveState& out, const cnoid::BodyPtr& robot_vrml, const std::shared_ptr<urdf::Model>& robot_urdf);
  static void primitiveStateMsg2Idl(const primitive_motion_level_msgs::PrimitiveState& in, primitive_motion_level_msgs::PrimitiveStateIdl& out, const cnoid::BodyPtr& robot_vrml, const std::shared_ptr<urdf::Model>& robot_urdf);
};


extern "C"
{
  void PrimitiveStateROSBridgeInit(RTC::Manager* manager);
};

#endif // PrimitiveStateROSBridge_H
