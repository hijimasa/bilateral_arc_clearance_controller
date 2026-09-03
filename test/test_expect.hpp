/**
 * @file test_expect.hpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @brief The check-and-count helper every BAC test binary shares
 * @date 2026-09-03
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Not a framework, and deliberately not one: the package's tests are plain
 * executables that return non-zero when a check fails, so they build with no
 * test dependency in either the ament tree or the plain-CMake tree.
 *
 * This file exists because the SAME nine lines had been copied into every one
 * of them - ten by the time the differential-drive configuration guard was
 * added - and near() into six. A copy that drifts is the failure this package
 * keeps meeting; one definition cannot.
 *
 * THE OUTPUT SHAPE IS PART OF THE CONTRACT. A passing check is silent and a
 * failing one prints exactly one line, "FAIL: <message>", to stderr. Every CI
 * log this package has recorded is greppable by that prefix, and a suite's
 * closing count ("N ... check(s) failed") is read off `failures`. Changing
 * either would silently orphan those records.
 */

#pragma once
#ifndef BAC_SIM_TEST_EXPECT_HPP__
#define BAC_SIM_TEST_EXPECT_HPP__

#include <cmath>
#include <iostream>
#include <string>

namespace bac_test
{

/// Failed checks in this binary. Each main() returns non-zero when it is not 0.
inline int failures = 0;

/// Record one check. Silent on success; one "FAIL: <message>" line on failure.
inline void
expect(bool condition, const std::string &message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

/// Float comparison at the tolerance these suites take as "the same number".
inline bool
near(float actual, float expected, float tolerance = 1e-5f)
{
  return std::fabs(actual - expected) <= tolerance;
}

}  // namespace bac_test

#endif  // BAC_SIM_TEST_EXPECT_HPP__
