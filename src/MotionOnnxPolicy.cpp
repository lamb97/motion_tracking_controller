//
// Created by qiayuanl on 5/14/25.
//

#include "motion_tracking_controller/MotionOnnxPolicy.h"

#include <iostream>

namespace legged {

void MotionOnnxPolicy::reset() {
  timeStep_ = startStep_;
  forward(vector_t::Zero(getObservationSize()));
  timeStep_ = startStep_;
  OnnxPolicy::reset();
}

vector_t MotionOnnxPolicy::forward(const vector_t& observations) {
  tensor2d_t timeStep(1, 1);
  timeStep(0, 0) = static_cast<tensor_element_t>(timeStep_++);
  inputTensors_[name2Index_.at("time_step")] = timeStep;
  OnnxPolicy::forward(observations);

  jointPosition_ = outputTensors_[name2Index_.at("joint_pos")].row(0).cast<scalar_t>();
  jointVelocity_ = outputTensors_[name2Index_.at("joint_vel")].row(0).cast<scalar_t>();
  bodyPositions_.clear();
  bodyOrientations_.clear();

  auto body_pos_w = outputTensors_[name2Index_.at("body_pos_w")].cast<scalar_t>();
  auto body_quat_w = outputTensors_[name2Index_.at("body_quat_w")].cast<scalar_t>();

  for (Eigen::Index i = 0; i < body_pos_w.rows(); ++i) {
    vector3_t pos = body_pos_w.row(i);
    vector_t quat = body_quat_w.row(i);
    quaternion_t ori;
    ori.w() = quat(0);
    ori.coeffs().head(3) = quat.tail(3);
    bodyPositions_.push_back(pos);
    bodyOrientations_.push_back(ori);
  }
  return getLastAction();
}

void MotionOnnxPolicy::parseMetadata() {
  OnnxPolicy::parseMetadata();
  anchorBodyName_ = getMetadataStr("anchor_body_name");
  std::cout << '\t' << "anchor_body_name: " << anchorBodyName_ << '\n';
  bodyNames_ = parseCsv<std::string>(getMetadataStr("body_names"));
  std::cout << '\t' << "body_names: " << bodyNames_ << '\n';
}

void MotionOnnxPolicy::checkInputOutput() {
  OnnxPolicy::checkInputOutput();

  // name2Index_ merges both namespaces and insert keeps the input entry.
  auto indexOnSide = [this](const std::string& name, const std::vector<const char*>& names, const std::string& side) {
    const auto it = name2Index_.find(name);
    if (it == name2Index_.end() || it->second >= names.size() || name != names[it->second]) {
      throw std::runtime_error("MotionOnnxPolicy: '" + name + "' is not a model " + side + ".");
    }
    return it->second;
  };
  // run() maps rank 2 as (s0, s1) and rank 3 as (s1, s2).
  auto shapeOf = [&](const std::string& name) {
    const auto& shape = outputShapes_[indexOnSide(name, outputNames_, "output")];
    if (shape.size() == 2) {
      return std::make_pair(static_cast<size_t>(shape[0]), static_cast<size_t>(shape[1]));
    }
    if (shape.size() == 3) {
      return std::make_pair(static_cast<size_t>(shape[1]), static_cast<size_t>(shape[2]));
    }
    throw std::runtime_error("MotionOnnxPolicy: '" + name + "' has unsupported rank " + std::to_string(shape.size()) + ".");
  };

  indexOnSide("time_step", inputNames_, "input");
  const auto bodyPos = shapeOf("body_pos_w");
  const auto bodyQuat = shapeOf("body_quat_w");
  const auto jointPos = shapeOf("joint_pos");
  const auto jointVel = shapeOf("joint_vel");

  if (bodyPos.first != bodyNames_.size()) {
    throw std::runtime_error("MotionOnnxPolicy: body_names has " + std::to_string(bodyNames_.size()) + " entries but body_pos_w has " +
                             std::to_string(bodyPos.first) + " rows.");
  }
  if (bodyQuat.first != bodyPos.first || bodyPos.second != 3 || bodyQuat.second != 4) {
    throw std::runtime_error("MotionOnnxPolicy: expected body_pos_w [" + std::to_string(bodyPos.first) + ",3] and body_quat_w [" +
                             std::to_string(bodyPos.first) + ",4].");
  }
  if (jointPos.second != jointNames_.size() || jointVel.second != jointNames_.size()) {
    throw std::runtime_error("MotionOnnxPolicy: joint_pos/joint_vel width does not match joint_names (" +
                             std::to_string(jointNames_.size()) + ").");
  }
}
}  // namespace legged
