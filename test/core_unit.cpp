/**
 * @file core_unit.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 */

#include "bilateral_arc_clearance_controller/bac_core.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

int failures = 0;

void
expect(bool condition, const std::string &message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

bool
near(float actual, float expected, float tolerance = 1e-5f)
{
  return std::fabs(actual - expected) <= tolerance;
}

void
testStraightArcGeometry()
{
  bac::BacCore core;
  const std::vector<bac::Point2D> points{ { 1.0f, 0.8f }, { 1.2f, -0.7f } };
  const bac::ArcEvaluation eval = core.evaluateArc(points, 0.4f, 0.0f, 4.0f);

  expect(near(eval.clearance_left, 0.8f), "straight arc left clearance");
  expect(near(eval.clearance_right, 0.7f), "straight arc right clearance");
  expect(eval.blocking_s == std::numeric_limits<float>::max(), "body-free arc has no blocking point");
}

void
testBlockingDistance()
{
  bac::BacCore core;
  const bac::ArcEvaluation eval = core.evaluateArc({ { 1.0f, 0.1f } }, 0.4f, 0.0f, 4.0f);

  expect(near(eval.blocking_s, 1.0f), "first body hit is reported along the arc");
}

void
testEmergencyStop()
{
  bac::BacCore core;
  const bac::Result result =
      core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{});

  expect(result.status == bac::Status::STOP, "point inside emergency zone stops the robot");
  expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f),
         "emergency output is zero");
}

void
testEmptyPathAndReset()
{
  bac::BacCore core;
  core.forceStop();
  expect(core.status() == bac::Status::STOP, "forceStop changes state");
  core.reset();
  expect(core.status() == bac::Status::CLEAR, "reset clears state");

  const bac::Result result = core.process({}, {}, bac::Twist2D{});
  expect(result.status == bac::Status::CLEAR, "empty path in open space is clear");
  expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f), "empty path holds position");
}

}  // namespace

int
main()
{
  testStraightArcGeometry();
  testBlockingDistance();
  testEmergencyStop();
  testEmptyPathAndReset();

  if (failures != 0)
  {
    std::cerr << failures << " core unit check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC core unit checks passed\n";
  return 0;
}
