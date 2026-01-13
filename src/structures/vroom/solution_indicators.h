#ifndef SOLUTION_INDICATORS_H
#define SOLUTION_INDICATORS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <tuple>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"

namespace vroom::utils {

struct SolutionIndicators {
  Priority priority_sum{0};
  unsigned assigned{0};
  Eval eval;
  unsigned used_vehicles{0};
  // Hash based on the ordered sizes of routes in the solution.
  uint32_t routes_hash;

  SolutionIndicators() = default;

  template <class Route>
  SolutionIndicators(const Input& input, const std::vector<Route>& sol)
    : SolutionIndicators() {
    Index v_rank = 0;
    for (const auto& r : sol) {
      priority_sum += utils::priority_sum_for_route(input, r.route);
      assigned += r.route.size();

      eval += utils::route_eval_for_vehicle(input, v_rank, r.route);
      ++v_rank;

      if (!r.empty()) {
        used_vehicles += 1;
      }
    }

    std::vector<uint32_t> routes_sizes;
    routes_sizes.reserve(sol.size());
    std::ranges::transform(sol,
                           std::back_inserter(routes_sizes),
                           [](const auto& r) { return std::size(r); });
    std::ranges::sort(routes_sizes);
    routes_hash = get_vector_hash(routes_sizes);
  }

  // Conditional objective function:
  // - If priorities are set: maximize profit (priority_sum - cost)
  // - If no priorities: use default lexicographic (max jobs, min cost)
  friend bool operator<(const SolutionIndicators& lhs,
                        const SolutionIndicators& rhs) {
    // Check if priorities are being used
    const bool using_priorities =
      (lhs.priority_sum > 0) || (rhs.priority_sum > 0);

    if (using_priorities) {
      // Profit-based comparison: maximize (priority_sum * scale - cost)
      // When using custom cost matrix, VROOM scales costs by:
      // DURATION_FACTOR * COST_FACTOR = 100 * 3600 = 360,000
      // We must scale priority to match.
      constexpr int64_t PRIORITY_SCALE = DURATION_FACTOR * COST_FACTOR;  // 360,000

      const int64_t lhs_profit =
        static_cast<int64_t>(lhs.priority_sum) * PRIORITY_SCALE - lhs.eval.cost;
      const int64_t rhs_profit =
        static_cast<int64_t>(rhs.priority_sum) * PRIORITY_SCALE - rhs.eval.cost;

      if (lhs_profit != rhs_profit) {
        return lhs_profit > rhs_profit;
      }

      // Tie-breakers: more jobs, fewer vehicles, lower duration
      return std::tie(rhs.assigned,
                      lhs.used_vehicles,
                      lhs.eval.duration,
                      lhs.eval.distance,
                      lhs.routes_hash) < std::tie(lhs.assigned,
                                                  rhs.used_vehicles,
                                                  rhs.eval.duration,
                                                  rhs.eval.distance,
                                                  rhs.routes_hash);
    }

    // Default lexicographic: max jobs, then min cost
    return std::tie(rhs.assigned,
                    lhs.eval.cost,
                    lhs.used_vehicles,
                    lhs.eval.duration,
                    lhs.eval.distance,
                    lhs.routes_hash) < std::tie(lhs.assigned,
                                                rhs.eval.cost,
                                                rhs.used_vehicles,
                                                rhs.eval.duration,
                                                rhs.eval.distance,
                                                rhs.routes_hash);
  }
};

} // namespace vroom::utils

#endif

