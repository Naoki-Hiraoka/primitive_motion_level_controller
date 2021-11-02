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
    targetPositionInterpolator_(cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),
    targetOrientationInterpolator_(cnoid::Matrix3::Identity(),cnoid::Vector3::Zero(),cnoid::Vector3::Zero(),cpp_filters::HOFFARBIB),
    targetWrenchRaw_(cnoid::Vector6::Zero()),
    targetWrench_(cnoid::Vector6::Zero()),
    targetWrenchInterpolator_(cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cnoid::Vector6::Zero(),cpp_filters::HOFFARBIB),
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

  void PrimitiveStates::updateTargetForOneStep(double dt) {
    for(std::map<std::string, std::shared_ptr<primitive_motion_level_tools::PrimitiveState> >::iterator it = this->primitiveState_.begin(); it != this->primitiveState_.end(); it++) {
      it->second->updateTargetForOneStep(dt);
    }
  }

  void PrimitiveStatesSequence::updateFromIdl(const primitive_motion_level_msgs::TimedPrimitiveStateSeqSeq& idl){
    this->primitiveStates_.resize(idl.data.length());
    for(int i=0;i<idl.data.length();i++){
      if(!this->primitiveStates_[i]) this->primitiveStates_[i] = std::make_shared<PrimitiveStates>();
      this->primitiveStates_[i]->updateFromIdl(idl.data[i]);
    }
  }

  void PrimitiveStatesSequence::updateTargetForOneStep(double dt){
    if(this->primitiveStates_.size()>0) {
      this->primitiveStates_[0]->time() -= dt;
      if(this->primitiveStates_[0]->time()<0.0) this->primitiveStates_[0]->time() = 0.0;
    }
  }

};
