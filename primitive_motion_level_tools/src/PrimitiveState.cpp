#include <primitive_motion_level_tools/PrimitiveState.h>

#include <cnoid/EigenUtil>
#include <iostream>

namespace primitive_motion_level_tools {
  PrimitiveState::PrimitiveState() : PrimitiveState::PrimitiveState("") {
  }

  PrimitiveState::PrimitiveState(const std::string& name) :
    name_(name),
    parentLinkName_(""),
    localPose_(cnoid::Position::Identity()),
    time_(0.0),
    targetPoseRaw_(cnoid::Position::Identity()),
    targetPose_(cnoid::Position::Identity()),
    targetPosePrev_(cnoid::Position::Identity()),
    targetPosePrevPrev_(cnoid::Position::Identity()),
    targetPositionInterpolator_(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::LINEAR),
    targetOrientationInterpolator_(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::LINEAR),
    targetWrenchRaw_(cnoid::Vector6::Zero()),
    targetWrench_(cnoid::Vector6::Zero()),
    targetWrenchInterpolator_(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::LINEAR),
    poseFollowGain_(cnoid::Vector6::Zero()),
    wrenchFollowGain_(cnoid::Vector6::Zero()),
    isPoseCGlobal_(false),
    poseC_(0,6),
    poseld_(0),
    poseud_(0),
    isWrenchCGlobal_(false),
    wrenchC_(0,6),
    wrenchld_(0),
    wrenchud_(0),
    M_(cnoid::Vector6::Zero()),
    D_(cnoid::Vector6::Zero()),
    K_(cnoid::Vector6::Zero()),
    actWrench_(cnoid::Vector6::Zero()),
    supportCOM_(false),
    isInitial_(true)
  {
  }

  void PrimitiveState::updateFromIdl(const primitive_motion_level_msgs::PrimitiveStateIdl& idl) {
    this->name_ = idl.name;
    this->parentLinkName_ = idl.parentLinkName;
    this->localPose_.translation()[0] = idl.localPose.position.x;
    this->localPose_.translation()[1] = idl.localPose.position.y;
    this->localPose_.translation()[2] = idl.localPose.position.z;
    this->localPose_.linear() = cnoid::rotFromRpy(idl.localPose.orientation.r,idl.localPose.orientation.p,idl.localPose.orientation.y);
    this->time_ = idl.time;
    cnoid::Position pose;
    pose.translation()[0] = idl.pose.position.x;
    pose.translation()[1] = idl.pose.position.y;
    pose.translation()[2] = idl.pose.position.z;
    pose.linear() = cnoid::rotFromRpy(idl.pose.orientation.r,idl.pose.orientation.p,idl.pose.orientation.y);
    this->targetPoseRaw_ = pose;
    if(!this->isInitial_ && idl.time > 0.0){
      this->targetPositionInterpolator_.setGoal(pose.translation(),idl.time);
      this->targetOrientationInterpolator_.setGoal(pose.linear(),idl.time);
    }else{
      this->targetPositionInterpolator_.reset(pose.translation());
      this->targetOrientationInterpolator_.reset(pose.linear());
    }
    cnoid::Vector6 wrench; for(size_t i=0;i<6;i++) wrench[i] = idl.wrench[i];
    this->targetWrenchRaw_ = wrench;
    if(!this->isInitial_ && idl.time > 0.0){
      this->targetWrenchInterpolator_.setGoal(wrench,idl.time);
    }else{
      this->targetWrenchInterpolator_.reset(wrench);
    }
    for(size_t i=0;i<6;i++) this->poseFollowGain_[i] = idl.poseFollowGain[i];
    for(size_t i=0;i<6;i++) this->wrenchFollowGain_[i] = idl.wrenchFollowGain[i];
    this->isPoseCGlobal_ = idl.isPoseCGlobal;
    this->poseC_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(idl.poseC.length(),6);
    for(size_t i=0;i<idl.poseC.length();i++)
      for(size_t j=0;j<6;j++)
        if(idl.poseC[i][j]!=0) this->poseC_.insert(i,j) = idl.poseC[i][j];
    this->poseld_.resize(idl.poseld.length());
    for(size_t i=0;i<idl.poseld.length();i++) this->poseld_[i] = idl.poseld[i];
    this->poseud_.resize(idl.poseud.length());
    for(size_t i=0;i<idl.poseud.length();i++) this->poseud_[i] = idl.poseud[i];
    if(this->poseC_.rows() != this->poseld_.rows() ||
       this->poseld_.rows() != this->poseud_.rows()){
      std::cerr << "\x1b[31m[PrimitiveState::updateFromIdl] " << "dimension mismatch" << "\x1b[39m" << std::endl;
      this->poseC_.resize(0,6);
      this->poseld_.resize(0);
      this->poseud_.resize(0);
    }
    this->isWrenchCGlobal_ = idl.isWrenchCGlobal;
    this->wrenchC_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(idl.wrenchC.length(),6);
    for(size_t i=0;i<idl.wrenchC.length();i++)
      for(size_t j=0;j<6;j++)
        if(idl.wrenchC[i][j]!=0) this->wrenchC_.insert(i,j) = idl.wrenchC[i][j];
    this->wrenchld_.resize(idl.wrenchld.length());
    for(size_t i=0;i<idl.wrenchld.length();i++) this->wrenchld_[i] = idl.wrenchld[i];
    this->wrenchud_.resize(idl.wrenchud.length());
    for(size_t i=0;i<idl.wrenchud.length();i++) this->wrenchud_[i] = idl.wrenchud[i];
    if(this->wrenchC_.rows() != this->wrenchld_.rows() ||
       this->wrenchld_.rows() != this->wrenchud_.rows()){
      std::cerr << "\x1b[31m[PrimitiveCommand::updateFromIdl] " << "dimension mismatch" << "\x1b[39m" << std::endl;
      this->wrenchC_.resize(0,6);
      this->wrenchld_.resize(0);
      this->wrenchud_.resize(0);
    }
    for(size_t i=0;i<6;i++) this->M_[i] = idl.M[i];
    for(size_t i=0;i<6;i++) this->D_[i] = idl.D[i];
    for(size_t i=0;i<6;i++) this->K_[i] = idl.K[i];
    for(size_t i=0;i<6;i++) this->actWrench_[i] = idl.actWrench[i];
    this->supportCOM_ = idl.supportCOM;

    this->isInitial_ = false;

  }

  void PrimitiveState::updateFromMsg(const primitive_motion_level_msgs::PrimitiveState& msg) {
    this->name_ = msg.name;
    this->parentLinkName_ = msg.parent_link_name;
    this->localPose_.translation()[0] = msg.local_pose.position.x;
    this->localPose_.translation()[1] = msg.local_pose.position.y;
    this->localPose_.translation()[2] = msg.local_pose.position.z;
    this->localPose_.linear() = cnoid::Matrix3(cnoid::Quaternion(msg.local_pose.orientation.w,msg.local_pose.orientation.x,msg.local_pose.orientation.y,msg.local_pose.orientation.z));
    this->time_ = msg.time;
    cnoid::Position pose;
    pose.translation()[0] = msg.pose.position.x;
    pose.translation()[1] = msg.pose.position.y;
    pose.translation()[2] = msg.pose.position.z;
    pose.linear() = cnoid::Matrix3(cnoid::Quaternion(msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z));
    this->targetPoseRaw_ = pose;
    if(!this->isInitial_ && msg.time > 0.0){
      this->targetPositionInterpolator_.setGoal(pose.translation(),msg.time);
      this->targetOrientationInterpolator_.setGoal(pose.linear(),msg.time);
    }else{
      this->targetPositionInterpolator_.reset(pose.translation());
      this->targetOrientationInterpolator_.reset(pose.linear());
    }
    if(msg.wrench.size() == 6){
      cnoid::Vector6 wrench; for(size_t i=0;i<6;i++) wrench[i] = msg.wrench[i];
      this->targetWrenchRaw_ = wrench;
      if(!this->isInitial_ && msg.time > 0.0){
        this->targetWrenchInterpolator_.setGoal(wrench,msg.time);
      }else{
        this->targetWrenchInterpolator_.reset(wrench);
      }
    }
    if(msg.pose_follow_gain.size()==6) for(size_t i=0;i<6;i++) this->poseFollowGain_[i] = msg.pose_follow_gain[i];
    if(msg.wrench_follow_gain.size()==6) for(size_t i=0;i<6;i++) this->wrenchFollowGain_[i] = msg.wrench_follow_gain[i];
    this->isPoseCGlobal_ = msg.is_poseC_global;
    if(msg.poseC.size() %6 == 0){
      this->poseC_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(msg.poseC.size()/6,6);
      for(size_t i=0;i<msg.poseC.size()/6;i++)
        for(size_t j=0;j<6;j++)
          if(msg.poseC[i*6+j]!=0) this->poseC_.insert(i,j) = msg.poseC[i*6+j];
    }
    this->poseld_.resize(msg.poseld.size());
    for(size_t i=0;i<msg.poseld.size();i++) this->poseld_[i] = msg.poseld[i];
    this->poseud_.resize(msg.poseud.size());
    for(size_t i=0;i<msg.poseud.size();i++) this->poseud_[i] = msg.poseud[i];
    if(this->poseC_.rows() != this->poseld_.rows() ||
       this->poseld_.rows() != this->poseud_.rows()){
      std::cerr << "\x1b[31m[PrimitiveState::updateFromIdl] " << "dimension mismatch" << "\x1b[39m" << std::endl;
      this->poseC_.resize(0,6);
      this->poseld_.resize(0);
      this->poseud_.resize(0);
    }
    this->isWrenchCGlobal_ = msg.is_wrenchC_global;
    if(msg.wrenchC.size() % 6 == 0){
      this->wrenchC_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(msg.wrenchC.size()/6,6);
      for(size_t i=0;i<msg.wrenchC.size()/6;i++)
        for(size_t j=0;j<6;j++)
          if(msg.wrenchC[i*6+j]!=0) this->wrenchC_.insert(i,j) = msg.wrenchC[i*6+j];
    }
    this->wrenchld_.resize(msg.wrenchld.size());
    for(size_t i=0;i<msg.wrenchld.size();i++) this->wrenchld_[i] = msg.wrenchld[i];
    this->wrenchud_.resize(msg.wrenchud.size());
    for(size_t i=0;i<msg.wrenchud.size();i++) this->wrenchud_[i] = msg.wrenchud[i];
    if(this->wrenchC_.rows() != this->wrenchld_.rows() ||
       this->wrenchld_.rows() != this->wrenchud_.rows()){
      std::cerr << "\x1b[31m[PrimitiveCommand::updateFromIdl] " << "dimension mismatch" << "\x1b[39m" << std::endl;
      this->wrenchC_.resize(0,6);
      this->wrenchld_.resize(0);
      this->wrenchud_.resize(0);
    }
    if(msg.M.size()==6) for(size_t i=0;i<6;i++) this->M_[i] = msg.M[i];
    if(msg.D.size()==6) for(size_t i=0;i<6;i++) this->D_[i] = msg.D[i];
    if(msg.K.size()==6) for(size_t i=0;i<6;i++) this->K_[i] = msg.K[i];
    if(msg.act_wrench.size()==6) for(size_t i=0;i<6;i++) this->actWrench_[i] = msg.act_wrench[i];
    this->supportCOM_ = msg.support_com;

    this->isInitial_ = false;
  }

  primitive_motion_level_msgs::PrimitiveState PrimitiveState::toMsg() {
    primitive_motion_level_msgs::PrimitiveState msg;
    msg.name = this->name_;
    msg.parent_link_name = this->parentLinkName_;
    msg.local_pose.position.x = this->localPose_.translation()[0];
    msg.local_pose.position.y = this->localPose_.translation()[1];
    msg.local_pose.position.z = this->localPose_.translation()[2];
    cnoid::Quaternion quat = cnoid::Quaternion(this->localPose_.linear());
    msg.local_pose.orientation.x = quat.x();
    msg.local_pose.orientation.y = quat.y();
    msg.local_pose.orientation.z = quat.z();
    msg.local_pose.orientation.w = quat.w();
    msg.time = this->time_;
    msg.pose.position.x = this->targetPose_.translation()[0];
    msg.pose.position.y = this->targetPose_.translation()[1];
    msg.pose.position.z = this->targetPose_.translation()[2];
    quat = this->targetPose_.linear();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.wrench.resize(6);
    for(int i=0;i<6;i++) msg.wrench[i] = this->targetWrench_[i];
    msg.pose_follow_gain.resize(6);
    for(int i=0;i<6;i++) msg.pose_follow_gain[i] = this->poseFollowGain_[i];
    msg.wrench_follow_gain.resize(6);
    for(int i=0;i<6;i++) msg.wrench_follow_gain[i] = this->wrenchFollowGain_[i];
    msg.is_poseC_global = this->isPoseCGlobal_;
    cnoid::MatrixXd poseC = this->poseC_;
    msg.poseC.resize(poseC.rows()*poseC.cols());
    for(int i=0;i<poseC.rows();i++){
      for(int j=0;j<poseC.cols();j++){
        msg.poseC[i*6+j] = poseC(i,j);
      }
    }
    msg.poseld.resize(this->poseld_.size());
    for(int i=0;i<this->poseld_.size();i++) msg.poseld[i] = this->poseld_[i];
    msg.poseud.resize(this->poseud_.size());
    for(int i=0;i<this->poseud_.size();i++) msg.poseud[i] = this->poseud_[i];
    msg.is_wrenchC_global = this->isWrenchCGlobal_;
    cnoid::MatrixXd wrenchC = this->wrenchC_;
    msg.wrenchC.resize(wrenchC.rows()*wrenchC.cols());
    for(int i=0;i<wrenchC.rows();i++){
      for(int j=0;j<wrenchC.cols();j++){
        msg.wrenchC[i*6+j] = wrenchC(i,j);
      }
    }
    msg.wrenchld.resize(this->wrenchld_.size());
    for(int i=0;i<this->wrenchld_.size();i++) msg.wrenchld[i] = this->wrenchld_[i];
    msg.wrenchud.resize(this->wrenchud_.size());
    for(int i=0;i<this->wrenchud_.size();i++) msg.wrenchud[i] = this->wrenchud_[i];
    msg.act_wrench.resize(6);
    for(int i=0;i<6;i++) msg.act_wrench[i] = this->actWrench_[i];
    msg.M.resize(6);
    for(int i=0;i<6;i++) msg.M[i] = this->M_[i];
    msg.D.resize(6);
    for(int i=0;i<6;i++) msg.D[i] = this->D_[i];
    msg.K.resize(6);
    for(int i=0;i<6;i++) msg.K[i] = this->K_[i];
    msg.support_com = this->supportCOM_;

    return msg;
  }

  void PrimitiveState::updateFromParam(ros::NodeHandle& nh, const std::string& ns) {
    std::vector<double> doublevec;

    if(!nh.getParam(ns+"name",this->name_))
      std::cerr << "\x1b[31m[PrimitiveState::updateFromParam] " << "name not defined" << "\x1b[39m" << std::endl;
    nh.getParam(ns+"parent_link_name",this->parentLinkName_);
    if(nh.getParam(ns+"local_pose",doublevec) && doublevec.size()==7){
      this->localPose_.translation()[0] = doublevec[0];
      this->localPose_.translation()[1] = doublevec[1];
      this->localPose_.translation()[2] = doublevec[2];
      this->localPose_.linear() = cnoid::Matrix3(cnoid::Quaternion(doublevec[6],doublevec[3],doublevec[4],doublevec[5]));
    }
    nh.getParam(ns+"support_com",this->supportCOM_);
    nh.getParam(ns+"time",this->time_);
    nh.getParam(ns+"is_wrenchC_global",this->isWrenchCGlobal_);
    if(nh.getParam(ns+"wrenchC",doublevec) && doublevec.size()%6==0){
      this->wrenchC_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(doublevec.size()/6,6);
      for(size_t i=0;i<doublevec.size()/6;i++)
        for(size_t j=0;j<6;j++)
          if(doublevec[i*6+j]!=0) this->wrenchC_.insert(i,j) = doublevec[i*6+j];
    }
    if(nh.getParam(ns+"wrenchld",doublevec)){
      this->wrenchld_.resize(doublevec.size());
      for(size_t i=0;i<doublevec.size();i++) this->wrenchld_[i] = doublevec[i];
    }
    if(nh.getParam(ns+"wrenchud",doublevec)){
      this->wrenchud_.resize(doublevec.size());
      for(size_t i=0;i<doublevec.size();i++) this->wrenchud_[i] = doublevec[i];
    }
    if(this->wrenchC_.rows() != this->wrenchld_.rows() ||
       this->wrenchld_.rows() != this->wrenchud_.rows()){
      std::cerr << "\x1b[31m[PrimitiveCommand::updateFromParam] " << this->name_ << " dimension mismatch" << this->wrenchC_.rows() << " " << this->wrenchld_.rows() << " " << this->wrenchud_.rows() << "\x1b[39m" << std::endl;
      this->wrenchC_.resize(0,6);
      this->wrenchld_.resize(0);
      this->wrenchud_.resize(0);
    }
    if(nh.getParam(ns+"pose_follow_gain",doublevec) && doublevec.size()==6){
      for(size_t i=0;i<6;i++) this->poseFollowGain_[i] = doublevec[i];
    }
    if(nh.getParam(ns+"wrench_follow_gain",doublevec) && doublevec.size()==6){
      for(size_t i=0;i<6;i++) this->wrenchFollowGain_[i] = doublevec[i];
    }
    if(nh.getParam(ns+"M",doublevec) && doublevec.size()==6){
      for(size_t i=0;i<6;i++) this->M_[i] = doublevec[i];
    }
    if(nh.getParam(ns+"D",doublevec) && doublevec.size()==6){
      for(size_t i=0;i<6;i++) this->D_[i] = doublevec[i];
    }
    if(nh.getParam(ns+"K",doublevec) && doublevec.size()==6){
      for(size_t i=0;i<6;i++) this->K_[i] = doublevec[i];
    }
  }

  void PrimitiveState::updateTargetForOneStep(double dt) {
    this->targetPosePrevPrev_ = this->targetPosePrev_;
    this->targetPosePrev_ = this->targetPose_;
    cnoid::Vector3 trans;
    this->targetPositionInterpolator_.get(trans, dt);
    this->targetPose_.translation() = trans;
    cnoid::Matrix3 R;
    this->targetOrientationInterpolator_.get(R, dt);
    this->targetPose_.linear() = R;
    this->targetWrenchInterpolator_.get(this->targetWrench_, dt);
  }

  void PrimitiveStates::updateFromIdl(const primitive_motion_level_msgs::TimedPrimitiveStateSeq& idl){
    this->time_ = idl.tm.sec + idl.tm.nsec * 0.000000001;

    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::iterator it = this->primitiveState_.begin(); it != this->primitiveState_.end(); ) {
      bool found = false;
      for(size_t i=0;i<idl.data.length();i++) {
        if(std::string(idl.data[i].name)==it->first) found = true;
      }
      if (!found) it = this->primitiveState_.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(size_t i=0;i<idl.data.length();i++){
      if(this->primitiveState_.find(std::string(idl.data[i].name))==this->primitiveState_.end()){
        this->primitiveState_[std::string(idl.data[i].name)] = std::make_shared<primitive_motion_level_tools::PrimitiveState>(std::string(idl.data[i].name));
      }
    }
    // 各指令値の反映
    for(size_t i=0;i<idl.data.length();i++){
      const primitive_motion_level_msgs::PrimitiveStateIdl& idlstate = idl.data[i];
      std::shared_ptr<primitive_motion_level_tools::PrimitiveState> state = this->primitiveState_[std::string(idlstate.name)];
      state->updateFromIdl(idlstate);
    }
  }

  void PrimitiveStates::updateFromMsg(const primitive_motion_level_msgs::PrimitiveStateArray& msg){
    this->time_ = msg.header.stamp.sec + msg.header.stamp.nsec * 0.000000001;

    // 消滅したEndEffectorを削除
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::iterator it = this->primitiveState_.begin(); it != this->primitiveState_.end(); ) {
      bool found = false;
      for(size_t i=0;i<msg.primitive_state.size();i++) {
        if(msg.primitive_state[i].name==it->first) found = true;
      }
      if (!found) it = this->primitiveState_.erase(it);
      else ++it;
    }
    // 増加したEndEffectorの反映
    for(size_t i=0;i<msg.primitive_state.size();i++){
      if(this->primitiveState_.find(msg.primitive_state[i].name)==this->primitiveState_.end()){
        this->primitiveState_[msg.primitive_state[i].name] = std::make_shared<primitive_motion_level_tools::PrimitiveState>(msg.primitive_state[i].name);
      }
    }
    // 各指令値の反映
    for(size_t i=0;i<msg.primitive_state.size();i++){
      const primitive_motion_level_msgs::PrimitiveState& msgstate = msg.primitive_state[i];
      std::shared_ptr<primitive_motion_level_tools::PrimitiveState> state = this->primitiveState_[msgstate.name];
      state->updateFromMsg(msgstate);
    }
  }

  void PrimitiveStates::updateTargetForOneStep(double dt) {
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::iterator it = this->primitiveState_.begin(); it != this->primitiveState_.end(); it++) {
      it->second->updateTargetForOneStep(dt);
    }
  }

  primitive_motion_level_msgs::PrimitiveStateArray PrimitiveStates::toMsg(){
    primitive_motion_level_msgs::PrimitiveStateArray msg;
    msg.header.stamp.sec = long(this->time_);
    msg.header.stamp.nsec = long((this->time_ - msg.header.stamp.sec) * 1e9);
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::iterator it = this->primitiveState_.begin(); it != this->primitiveState_.end(); it++) {
      msg.primitive_state.push_back(it->second->toMsg());
    }
    return msg;
  }

  void PrimitiveStatesSequence::updateFromIdl(const primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq& idl){
    this->primitiveStates_.resize(idl.data.length());
    for(int i=0;i<idl.data.length();i++){
      if(!this->primitiveStates_[i]) this->primitiveStates_[i] = std::make_shared<PrimitiveStates>();
      this->primitiveStates_[i]->updateFromIdl(idl.data[i]);
    }
  }

  void PrimitiveStatesSequence::updateFromMsg(const primitive_motion_level_msgs::PrimitiveStateArrayArray& msg){
    this->primitiveStates_.resize(msg.primitive_states.size());
    for(int i=0;i<msg.primitive_states.size();i++){
      if(!this->primitiveStates_[i]) this->primitiveStates_[i] = std::make_shared<PrimitiveStates>();
      this->primitiveStates_[i]->updateFromMsg(msg.primitive_states[i]);
    }
  }

  void PrimitiveStatesSequence::updateTargetForOneStep(double dt){
    if(this->primitiveStates_.size()>0) {
      this->primitiveStates_[0]->time() -= dt;
      if(this->primitiveStates_[0]->time()<0.0) this->primitiveStates_[0]->time() = 0.0;
    }
    for(int i=0;i<this->primitiveStates_.size();i++){
      this->primitiveStates_[i]->updateTargetForOneStep(dt);
    }
  }

  primitive_motion_level_msgs::PrimitiveStateArrayArray PrimitiveStatesSequence::toMsg(){
    primitive_motion_level_msgs::PrimitiveStateArrayArray msg;
    for(int i=0;i<this->primitiveStates_.size();i++) {
      msg.primitive_states.push_back(this->primitiveStates_[i]->toMsg());
    }
    return msg;
  }

};
