#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::multibody::internal::IiwaKinematicConstraintTest;
using drake::multibody::internal::TwoFreeBodiesConstraintTest;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalGazeTargetConstraintAutoDiff(
    const MultibodyTree<AutoDiffXd>& tree, const Frame<AutoDiffXd>& frameA,
    const Vector3<double>& p_AS, const Vector3<double>& n_A,
    const Frame<AutoDiffXd>& frameB, const Vector3<double>& p_BT,
    double cone_half_angle, const Context<AutoDiffXd>& context) {
  Vector3<AutoDiffXd> p_AT;
  tree.CalcPointsPositions(context, frameB, p_BT.cast<AutoDiffXd>(), frameA,
                           &p_AT);
  const Vector3<AutoDiffXd> p_ST_A = p_AT - p_AS;
  Vector2<AutoDiffXd> y_autodiff;
  const Vector3<double> n_A_normalized = n_A.normalized();
  y_autodiff(0) = p_ST_A.dot(n_A_normalized);
  y_autodiff(1) = pow(p_ST_A.dot(n_A_normalized), 2) -
                  std::pow(std::cos(cone_half_angle), 2) * p_ST_A.squaredNorm();
  return y_autodiff;
}

TEST_F(IiwaKinematicConstraintTest, GazeTargetConstraint) {
  const Frame<double>& frameA = plant_->GetFrameByName("iiwa_link_7");
  const Frame<double>& frameB = plant_->GetFrameByName("iiwa_link_3");
  const Eigen::Vector3d p_AS(0.1, 0.2, 0.3);
  const Eigen::Vector3d n_A(-0.1, 0.3, 0.4);
  const Eigen::Vector3d p_BT(0.4, 0.2, -0.3);
  const double cone_half_angle{0.1 * M_PI};
  GazeTargetConstraint constraint(plant_, frameA, p_AS, n_A, frameB, p_BT,
                                  cone_half_angle, plant_context_);

  EXPECT_EQ(constraint.num_constraints(), 2);
  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_.tree().num_positions());
  EXPECT_TRUE(
      CompareMatrices(constraint.lower_bound(), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(
      constraint.upper_bound(),
      Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity())));

  Eigen::VectorXd q(iiwa_autodiff_.tree().num_positions());
  // arbitrary joint configuration.
  q << 0.1, 0.2, -0.3, 0.5, -0.2, -0.05, 0.34;
  AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);

  auto mbt_context_autodiff =
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get());
  mbt_context_autodiff->get_mutable_positions() = q_autodiff;
  Vector2<AutoDiffXd> y_autodiff_expected = EvalGazeTargetConstraintAutoDiff(
      iiwa_autodiff_.tree(),
      iiwa_autodiff_.tree().GetFrameByName(frameA.name()), p_AS, n_A,
      iiwa_autodiff_.tree().GetFrameByName(frameB.name()), p_BT,
      cone_half_angle, *context_autodiff_);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Test with non-identity gradient for q_autodiff.
  q_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      q, MatrixX<double>::Ones(q.size(), 2));
  mbt_context_autodiff->get_mutable_positions() = q_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalGazeTargetConstraintAutoDiff(
      iiwa_autodiff_.tree(),
      iiwa_autodiff_.tree().GetFrameByName(frameA.name()), p_AS, n_A,
      iiwa_autodiff_.tree().GetFrameByName(frameB.name()), p_BT,
      cone_half_angle, *context_autodiff_);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
}

TEST_F(TwoFreeBodiesConstraintTest, GazeTargetConstraint) {
  // All the numbers are chosen arbitrarily.
  const Eigen::Vector3d p_AS(0.2, -0.1, 0.3);
  const Eigen::Vector3d n_A(0.4, -0.3, 0.9);
  const Eigen::Vector3d p_BT(-0.02, 0.5, 1.3);
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.1 * M_PI, Eigen::Vector3d(0.2, -.9, 1.3).normalized()));
  const Eigen::Vector3d body1_position(0.5, 1.6, 2.3);
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, -1.5, 2.4).normalized()));
  const Eigen::Vector3d body2_position(0.1, -2.3, 10.2);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  // Compute the angle between the vector p_ST and n_A;
  const Eigen::Vector3d p_AT =
      body1_quaternion.inverse() *
      (body2_quaternion * p_BT + body2_position - body1_position);
  const Eigen::Vector3d p_ST_A = p_AT - p_AS;
  const double angle =
      std::acos(p_ST_A.dot(n_A) / (p_ST_A.norm() * n_A.norm()));

  {
    GazeTargetConstraint good_constraint(
        plant_, plant_->tree().get_frame(body1_index_), p_AS, n_A,
        plant_->tree().get_frame(body2_index_), p_BT, angle + 0.01 * M_PI,
        plant_context_);
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }
  {
    GazeTargetConstraint bad_constraint(
        plant_, plant_->tree().get_frame(body1_index_), p_AS, n_A,
        plant_->tree().get_frame(body2_index_), p_BT, angle - 0.01 * M_PI,
        plant_context_);
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}

TEST_F(IiwaKinematicConstraintTest, GazeTargetConstraintConstructorError) {
  // Check if the constructor for GazeTargetConstraint will throw errors if the
  // inputs are incorrect.
  const Eigen::Vector3d p_AS(1, 2, 3);
  const Eigen::Vector3d p_BT(2, 3, 4);
  const Frame<double>& frameA = plant_->GetFrameByName("iiwa_link_3");
  const Frame<double>& frameB = plant_->GetFrameByName("iiwa_link_4");
  // zero n_A
  EXPECT_THROW(
      GazeTargetConstraint(plant_, frameA, p_AS, Eigen::Vector3d::Zero(),
                           frameB, p_BT, 0.1, plant_context_),
      std::invalid_argument);

  // wrong cone_half_angle
  EXPECT_THROW(
      GazeTargetConstraint(plant_, frameA, p_AS, Eigen::Vector3d::Ones(),
                           frameB, p_BT, -0.1, plant_context_),
      std::invalid_argument);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
