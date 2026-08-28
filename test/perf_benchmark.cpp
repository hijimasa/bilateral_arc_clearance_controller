/**
 * @file perf_benchmark.cpp
 * @author Masaaki Hijikata (hijikata@react-robot.com)
 * @copyright Copyright (c) 2026 Masaaki Hijikata
 *
 * Microbenchmark for BacCore::process(): per-tick latency percentiles
 * (p50 / p95 / max) after warm-up, per point count, in a corridor-like
 * scene. Emits a CSV of raw per-tick samples so the numbers quoted in the
 * documentation are reproducible (release re-re-review Medium 6).
 *
 * Usage: bac_perf_benchmark [csv_path]
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "bilateral_arc_clearance_controller/bac_core.hpp"

int
main(int argc, char **argv)
{
  const int         warmup = 50;
  const int         iters  = 2000;
  const std::string csv    = argc > 1 ? argv[1] : "";
  FILE             *fcsv   = csv.empty() ? nullptr : std::fopen(csv.c_str(), "w");
  if (fcsv)
  {
    std::fprintf(fcsv, "points,eval_pts,iter,us\n");
  }

  std::printf("%8s %9s %10s %10s %10s  (%d iters after %d warm-up; max = observed max of\n"
              "%8s %9s %10s %10s %10s   this run, NOT a WCET bound)\n",
              "points", "eval_pts", "p50[us]", "p95[us]", "max[us]", iters, warmup,
              "", "", "", "", "");

  for (int n_points : { 480, 1000, 2000, 4000 })
  {
    bac::BacCore core;

    // Corridor walls plus scattered clutter, resampled to n_points.
    std::vector<bac::Point2D> points;
    const int per_side = n_points / 2;
    for (int i = 0; i < per_side; i++)
    {
      const float x = -2.0f + 10.0f * static_cast<float>(i) / per_side;
      points.emplace_back(x, 0.9f + 0.02f * ((i * 7) % 5));
      points.emplace_back(x, -0.9f - 0.02f * ((i * 3) % 5));
    }
    std::vector<bac::Point2D> path;
    for (int i = 1; i <= 40; i++)
    {
      path.emplace_back(0.1f * static_cast<float>(i), 0.0f);
    }
    const bac::Twist2D current(0.3f, 0.1f);
    // Inputs beyond Params::max_points are decimated before candidate
    // evaluation - record the evaluated count in the raw CSV too, so the
    // file is self-describing.
    const int eval_pts = std::min<int>(n_points, bac::Params{}.max_points);

    for (int i = 0; i < warmup; i++)
    {
      core.process(points, path, current);
    }
    std::vector<double> us;
    us.reserve(iters);
    for (int i = 0; i < iters; i++)
    {
      const auto t0 = std::chrono::steady_clock::now();
      core.process(points, path, current);
      const auto t1 = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double, std::micro>(t1 - t0).count();
      us.push_back(dt);
      if (fcsv)
      {
        std::fprintf(fcsv, "%d,%d,%d,%.2f\n", n_points, eval_pts, i, dt);
      }
    }
    std::sort(us.begin(), us.end());
    std::printf("%8d %9d %10.1f %10.1f %10.1f\n", n_points, eval_pts, us[us.size() / 2],
                us[static_cast<size_t>(us.size() * 0.95)], us.back());
  }
  if (fcsv)
  {
    std::fclose(fcsv);
  }
  return 0;
}
