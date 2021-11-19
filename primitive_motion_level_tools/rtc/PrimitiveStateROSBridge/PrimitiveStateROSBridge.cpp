#include "PrimitiveStateROSBridge.h"
#include <tf2/utils.h>

#include <cnoid/BodyLoader>

PrimitiveStateROSBridge::PrimitiveStateROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager),
  m_primitiveCommandROSOut_("primitiveStateOut", m_primitiveCommandROS_),
  m_primitiveCommandRTMIn_("primitiveStateIn", m_primitiveCommandRTM_),
  m_primitiveCommandSeqROSOut_("primitiveStateSeqOut", m_primitiveCommandSeqROS_),
  m_primitiveCommandSeqRTMIn_("primitiveStateSeqIn", m_primitiveCommandSeqRTM_)
{
}

RTC::ReturnCode_t PrimitiveStateROSBridge::onInitialize(){
  addOutPort("primitiveStateOut", m_primitiveCommandROSOut_);
  addInPort("primitiveStateIn", m_primitiveCommandRTMIn_);
  addOutPort("primitiveStateSeqOut", m_primitiveCommandSeqROSOut_);
  addInPort("primitiveStateSeqIn", m_primitiveCommandSeqRTMIn_);

  cnoid::BodyLoader bodyLoader;

  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  this->robot_vrml_ = bodyLoader.load(fileName);
  if(!this->robot_vrml_){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->robot_urdf_ = std::make_shared<urdf::Model>();
  this->robot_urdf_->initParam("robot_description");

  ros::NodeHandle pnh("~");
  sub_ = pnh.subscribe("input", 1, &PrimitiveStateROSBridge::topicCallback, this);
  pub_ = pnh.advertise<primitive_motion_level_msgs::PrimitiveStateArray>("output", 1);
  seqSub_ = pnh.subscribe("seq_input", 1, &PrimitiveStateROSBridge::seqTopicCallback, this);
  seqPub_ = pnh.advertise<primitive_motion_level_msgs::PrimitiveStateArrayArray>("seq_output", 1);

  return RTC::RTC_OK;
}

std::string URDFToVRMLLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& URDFLinkName){
  std::shared_ptr<const urdf::Link> link = robot_urdf->getLink(URDFLinkName);
  if(link){
    if(link->parent_joint){
      return link->parent_joint->name;
    }else if (link == robot_urdf->getRoot()){
      return robot_vrml->rootLink()->name();
    }
  }
  std::cerr << "\x1b[31m" << "[URDFToVRMLLinkName] failed to find link [" << URDFLinkName << "]" << "\x1b[39m" << std::endl;
  return URDFLinkName;
};

std::string VRMLToURDFLinkName(cnoid::BodyPtr robot_vrml, std::shared_ptr<urdf::Model> robot_urdf, const std::string& VRMLLinkName){
  std::shared_ptr<const urdf::Joint> joint = robot_urdf->getJoint(VRMLLinkName);
  if(joint){
    return joint->child_link_name;
  }else if (robot_vrml->rootLink()->name() == VRMLLinkName){
    return robot_urdf->getRoot()->name;
  }
  std::cerr << "\x1b[31m" << "[VRMLToURDFLinkName] failed to find link [" << VRMLLinkName << "]" << "\x1b[39m" << std::endl;
  return VRMLLinkName;
};

RTC::ReturnCode_t PrimitiveStateROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();
  if(this->m_primitiveCommandRTMIn_.isNew()){
    this->m_primitiveCommandRTMIn_.read();

    primitive_motion_level_msgs::PrimitiveStateArray msg;
    msg.header.stamp = ros::Time::now();
    for(int i=0;i<m_primitiveCommandRTM_.data.length();i++){
      primitive_motion_level_msgs::PrimitiveState state;
      PrimitiveStateROSBridge::primitiveStateIdl2Msg(m_primitiveCommandRTM_.data[i], state, this->robot_vrml_, this->robot_urdf_);
      msg.primitive_state.push_back(state);
    }
    this->pub_.publish(msg);
  }
  if(this->m_primitiveCommandSeqRTMIn_.isNew()){
    this->m_primitiveCommandSeqRTMIn_.read();

    primitive_motion_level_msgs::PrimitiveStateArrayArray msg;
    msg.header.stamp = ros::Time::now();
    for(int i=0;i<m_primitiveCommandSeqRTM_.data.length();i++){
      primitive_motion_level_msgs::PrimitiveStateArray stateArray;
      stateArray.header.stamp.sec = m_primitiveCommandSeqRTM_.data[i].tm.sec;
      stateArray.header.stamp.nsec = m_primitiveCommandSeqRTM_.data[i].tm.nsec;
      for(int j=0;j<m_primitiveCommandSeqRTM_.data[i].data.length();j++){
        primitive_motion_level_msgs::PrimitiveState state;
        PrimitiveStateROSBridge::primitiveStateIdl2Msg(m_primitiveCommandSeqRTM_.data[i].data[j], state, this->robot_vrml_, this->robot_urdf_);
        stateArray.primitive_state.push_back(state);
      }
      msg.primitive_states.push_back(stateArray);
    }
    this->seqPub_.publish(msg);
  }
  return RTC::RTC_OK;
}

void PrimitiveStateROSBridge::topicCallback(const primitive_motion_level_msgs::PrimitiveStateArray::ConstPtr& msg) {
  coil::TimeValue coiltm(coil::gettimeofday());
  m_primitiveCommandROS_.tm.sec  = coiltm.sec();
  m_primitiveCommandROS_.tm.nsec = coiltm.usec() * 1000;
  m_primitiveCommandROS_.data.length(msg->primitive_state.size());
  for(int i=0;i<msg->primitive_state.size();i++){
    PrimitiveStateROSBridge::primitiveStateMsg2Idl(msg->primitive_state[i], m_primitiveCommandROS_.data[i], this->robot_vrml_, this->robot_urdf_);
  }

  m_primitiveCommandROSOut_.write();

}

void PrimitiveStateROSBridge::seqTopicCallback(const primitive_motion_level_msgs::PrimitiveStateArrayArray::ConstPtr& msg) {
  coil::TimeValue coiltm(coil::gettimeofday());
  m_primitiveCommandSeqROS_.tm.sec  = coiltm.sec();
  m_primitiveCommandSeqROS_.tm.nsec = coiltm.usec() * 1000;
  m_primitiveCommandSeqROS_.data.length(msg->primitive_states.size());
  for(int i=0;i<msg->primitive_states.size();i++){
    m_primitiveCommandSeqROS_.data[i].tm.sec = msg->primitive_states[i].header.stamp.sec;
    m_primitiveCommandSeqROS_.data[i].tm.nsec = msg->primitive_states[i].header.stamp.nsec;
    m_primitiveCommandSeqROS_.data[i].data.length(msg->primitive_states[i].primitive_state.size());
    for(int j=0;j<msg->primitive_states[i].primitive_state.size();j++){
      PrimitiveStateROSBridge::primitiveStateMsg2Idl(msg->primitive_states[i].primitive_state[j], m_primitiveCommandSeqROS_.data[i].data[j], this->robot_vrml_, this->robot_urdf_);
    }
  }
  m_primitiveCommandSeqROSOut_.write();

}

void PrimitiveStateROSBridge::primitiveStateIdl2Msg(const primitive_motion_level_msgs::PrimitiveStateIdl& in, primitive_motion_level_msgs::PrimitiveState& out, const cnoid::BodyPtr& robot_vrml, const std::shared_ptr<urdf::Model>& robot_urdf) {
  out.name = std::string(in.name);
  if(std::string(in.parentLinkName) == "com") out.parent_link_name = "com";
  else out.parent_link_name = VRMLToURDFLinkName(robot_vrml, robot_urdf, std::string(in.parentLinkName));
  out.local_pose.position.x = in.localPose.position.x;
  out.local_pose.position.y = in.localPose.position.y;
  out.local_pose.position.z = in.localPose.position.z;
  tf2::Quaternion quat;
  quat.setRPY(in.localPose.orientation.r,
              in.localPose.orientation.p,
              in.localPose.orientation.y);
  out.local_pose.orientation.x = quat.x();
  out.local_pose.orientation.y = quat.y();
  out.local_pose.orientation.z = quat.z();
  out.local_pose.orientation.w = quat.w();
  out.time = in.time;
  out.pose.position.x = in.pose.position.x;
  out.pose.position.y = in.pose.position.y;
  out.pose.position.z = in.pose.position.z;
  quat.setRPY(in.pose.orientation.r,
              in.pose.orientation.p,
              in.pose.orientation.y);
  out.pose.orientation.x = quat.x();
  out.pose.orientation.y = quat.y();
  out.pose.orientation.z = quat.z();
  out.pose.orientation.w = quat.w();
  out.wrench.resize(6);
  for(int j=0;j<6;j++) out.wrench[j] = in.wrench[j];
  out.pose_follow_gain.resize(6);
  for(int j=0;j<6;j++) out.pose_follow_gain[j] = in.poseFollowGain[j];
  out.wrench_follow_gain.resize(6);
  for(int j=0;j<6;j++) out.wrench_follow_gain[j] = in.wrenchFollowGain[j];
  out.is_poseC_global = in.isPoseCGlobal;
  out.poseC.resize(in.poseC.length()*6);
  for(int j=0;j<in.poseC.length();j++){
    for(int k=0;k<6;k++) out.poseC[j*6+k] = in.poseC[j][k];
  }
  out.poseld.resize(in.poseld.length());
  for(int j=0;j<in.poseld.length();j++){
    out.poseld[j] = in.poseld[j];
  }
  out.poseud.resize(in.poseud.length());
  for(int j=0;j<in.poseud.length();j++){
    out.poseud[j] = in.poseud[j];
  }
  out.is_wrenchC_global = in.isWrenchCGlobal;
  out.wrenchC.resize(in.wrenchC.length()*6);
  for(int j=0;j<in.wrenchC.length();j++){
    for(int k=0;k<6;k++) out.wrenchC[j*6+k] = in.wrenchC[j][k];
  }
  out.wrenchld.resize(in.wrenchld.length());
  for(int j=0;j<in.wrenchld.length();j++){
    out.wrenchld[j] = in.wrenchld[j];
  }
  out.wrenchud.resize(in.wrenchud.length());
  for(int j=0;j<in.wrenchud.length();j++){
    out.wrenchud[j] = in.wrenchud[j];
  }
  out.M.resize(6);
  for(int j=0;j<6;j++) out.M[j] = in.M[j];
  out.D.resize(6);
  for(int j=0;j<6;j++) out.D[j] = in.D[j];
  out.K.resize(6);
  for(int j=0;j<6;j++) out.K[j] = in.K[j];
  out.act_wrench.resize(6);
  for(int j=0;j<6;j++) out.act_wrench[j] = in.actWrench[j];
  out.support_com = in.supportCOM;
}

void PrimitiveStateROSBridge::primitiveStateMsg2Idl(const primitive_motion_level_msgs::PrimitiveState& in, primitive_motion_level_msgs::PrimitiveStateIdl& out, const cnoid::BodyPtr& robot_vrml, const std::shared_ptr<urdf::Model>& robot_urdf) {
  out.name = in.name.c_str();
  if(in.parent_link_name == "com") out.parentLinkName = "com";
  else out.parentLinkName = URDFToVRMLLinkName(robot_vrml, robot_urdf, in.parent_link_name).c_str();
  out.localPose.position.x = in.local_pose.position.x;
  out.localPose.position.y = in.local_pose.position.y;
  out.localPose.position.z = in.local_pose.position.z;
  // tf2::Matrix3x3::getEulerYPRやtf2::Matrix3x3::getRPYにはバグがある https://github.com/ros/geometry2/issues/504
  cnoid::Vector3 ypr = Eigen::Quaterniond(in.local_pose.orientation.w,in.local_pose.orientation.x,in.local_pose.orientation.y,in.local_pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  out.localPose.orientation.r = ypr[2];
  out.localPose.orientation.p = ypr[1];
  out.localPose.orientation.y = ypr[0];
  out.time = in.time;
  out.pose.position.x = in.pose.position.x;
  out.pose.position.y = in.pose.position.y;
  out.pose.position.z = in.pose.position.z;
  ypr = Eigen::Quaterniond(in.pose.orientation.w,in.pose.orientation.x,in.pose.orientation.y,in.pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  out.pose.orientation.r = ypr[2];
  out.pose.orientation.p = ypr[1];
  out.pose.orientation.y = ypr[0];
  out.wrench.length(6);
  if(in.wrench.size() == 6){
    for(int j=0;j<6;j++) out.wrench[j] = in.wrench[j];
  }else{
    for(int j=0;j<6;j++) out.wrench[j] = 0.0;
  }
  out.poseFollowGain.length(6);
  if(in.pose_follow_gain.size() == 6) {
    for(int j=0;j<6;j++) out.poseFollowGain[j] = in.pose_follow_gain[j];
  }else{
    for(int j=0;j<6;j++) out.poseFollowGain[j] = 0.0;
  }
  out.wrenchFollowGain.length(6);
  if(in.wrench_follow_gain.size() == 6) {
    for(int j=0;j<6;j++) out.wrenchFollowGain[j] = in.wrench_follow_gain[j];
  }else{
    for(int j=0;j<6;j++) out.wrenchFollowGain[j] = 0.0;
  }
  out.isPoseCGlobal = in.is_poseC_global;
  if(in.poseC.size() % 6 == 0){
    out.poseC.length(in.poseC.size() / 6);
    for(int j=0;j<out.poseC.length(); j++) {
      out.poseC[j].length(6);
      for(int k=0;k<6;k++){
        out.poseC[j][k] = in.poseC[j*6+k];
      }
    }
  }
  out.poseld.length(in.poseld.size());
  for(int j=0;j<out.poseld.length(); j++) {
    out.poseld[j] = in.poseld[j];
  }
  out.poseud.length(in.poseud.size());
  for(int j=0;j<out.poseud.length(); j++) {
    out.poseud[j] = in.poseud[j];
  }
  out.isWrenchCGlobal = in.is_wrenchC_global;
  if(in.wrenchC.size() % 6 == 0){
    out.wrenchC.length(in.wrenchC.size() / 6);
    for(int j=0;j<out.wrenchC.length(); j++) {
      out.wrenchC[j].length(6);
      for(int k=0;k<6;k++){
        out.wrenchC[j][k] = in.wrenchC[j*6+k];
      }
    }
  }
  out.wrenchld.length(in.wrenchld.size());
  for(int j=0;j<out.wrenchld.length(); j++) {
    out.wrenchld[j] = in.wrenchld[j];
  }
  out.wrenchud.length(in.wrenchud.size());
  for(int j=0;j<out.wrenchud.length(); j++) {
    out.wrenchud[j] = in.wrenchud[j];
  }
  out.M.length(6);
  if(in.M.size() == 6) {
    for(int j=0;j<6;j++) out.M[j] = in.M[j];
  }else{
    for(int j=0;j<6;j++) out.M[j] = 0.0;
  }
  out.D.length(6);
  if(in.D.size() == 6) {
    for(int j=0;j<6;j++) out.D[j] = in.D[j];
  }else{
    for(int j=0;j<6;j++) out.D[j] = 0.0;
  }
  out.K.length(6);
  if(in.K.size() == 6) {
    for(int j=0;j<6;j++) out.K[j] = in.K[j];
  }else{
    for(int j=0;j<6;j++) out.K[j] = 0.0;
  }
  out.actWrench.length(6);
  if(in.act_wrench.size() == 6) {
    for(int j=0;j<6;j++) out.actWrench[j] = in.act_wrench[j];
  }else{
    for(int j=0;j<6;j++) out.actWrench[j] = 0.0;
  }
  out.supportCOM = in.support_com;
}

static const char* PrimitiveStateROSBridge_spec[] = {
  "implementation_id", "PrimitiveStateROSBridge",
  "type_name",         "PrimitiveStateROSBridge",
  "description",       "PrimitiveStateROSBridge component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void PrimitiveStateROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(PrimitiveStateROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<PrimitiveStateROSBridge>, RTC::Delete<PrimitiveStateROSBridge>);
    }
};
