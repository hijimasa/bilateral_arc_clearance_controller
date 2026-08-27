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
  // Moving into the emergency zone: braking has priority, output is zero.
  {
    bac::BacCore core;
    const bac::Result result =
        core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{ 0.3f, 0.0f });
    expect(result.status == bac::Status::STOP, "point inside emergency zone stops the robot");
    expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f),
           "emergency output is zero while moving");
  }

  // At standstill the emergency state must not freeze: with rear coverage
  // (limits.v_min < 0) the robot backs away from the frontal point instead.
  {
    bac::BacCore core;
    const bac::Result result =
        core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{});
    expect(result.output.v < 1e-4f, "standstill emergency never advances");
    expect(result.output.v < -1e-3f, "standstill emergency backs away when reverse is allowed");
  }

  // Without rear coverage (v_min = 0) the standstill emergency stays a stop.
  {
    bac::Params params;
    params.limits.v_min = 0.0f;
    bac::BacCore core(params);
    const bac::Result result =
        core.process({ { 0.59f, 0.0f } }, { { 1.0f, 0.0f } }, bac::Twist2D{});
    expect(result.status == bac::Status::STOP, "front-only sensing emergency stays a stop");
    expect(near(result.output.v, 0.0f) && near(result.output.w, 0.0f),
           "front-only emergency output is zero");
  }
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

void
testFaceAwayRecovery()
{
  // Standstill facing AWAY from the goal, open space ahead, a passable gap
  // in clutter behind toward the goal. The stop candidate must not win
  // forever (phantom clearance of the un-made straight run + the hysteresis
  // barrier against starting to turn used to freeze this state permanently).
  std::vector<bac::Point2D> world_points;
  for (float s = -0.6f; s <= 0.6f; s += 0.05f)
  {
    world_points.emplace_back(1.5f + s * 0.3f, 1.3f + s);
    world_points.emplace_back(1.5f + s * 0.3f, -1.5f + s);
  }

  float x = 0.0f, y = 0.0f, th = 2.0f;
  bac::BacCore core;
  bac::Twist2D current;
  for (int i = 0; i < 400; i++)
  {
    const float cs = std::cos(-th), sn = std::sin(-th);
    std::vector<bac::Point2D> points;
    for (const bac::Point2D &p : world_points)
    {
      const float dx = p.x - x, dy = p.y - y;
      points.emplace_back(cs * dx - sn * dy, sn * dx + cs * dy);
    }
    std::vector<bac::Point2D> path;
    for (float gx = 0.0f; gx <= 6.0f; gx += 0.25f)
    {
      const float dx = gx - x, dy = 0.0f - y;
      path.emplace_back(cs * dx - sn * dy, sn * dx + cs * dy);
    }
    const bac::Result result = core.process(points, path, current);
    current = result.output;
    th += current.w * 0.05f;
    x += current.v * std::cos(th) * 0.05f;
    y += current.v * std::sin(th) * 0.05f;
  }
  expect(x > 2.0f, "faces-away start turns around and passes the gap towards the goal");
}

}  // namespace

int
main()
{
  testStraightArcGeometry();
  testBlockingDistance();
  testEmergencyStop();
  testEmptyPathAndReset();
  testFaceAwayRecovery();

  if (failures != 0)
  {
    std::cerr << failures << " core unit check(s) failed\n";
    return 1;
  }
  std::cout << "All BAC core unit checks passed\n";
  return 0;
}
