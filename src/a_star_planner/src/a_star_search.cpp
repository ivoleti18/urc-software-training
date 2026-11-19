// The A* search itself. See a_star_search.hpp and Lesson 10 for the requirements.

#include "a_star_planner/a_star_search.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace robonav_training
{
namespace
{

bool isBlocked(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int col, int row)
{
  return !grid.inBounds(col, row) || blocked[grid.index(col, row)] != 0;
}

}  // namespace

std::vector<int> astarSearch(
  const GridMap & grid, const std::vector<std::uint8_t> & blocked,
  int start_col, int start_row, int goal_col, int goal_row)
{
  // Part 1 is provided: reject malformed inputs and impossible endpoints.
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
    blocked.size() != grid.cellCount() ||
    !grid.inBounds(start_col, start_row) || !grid.inBounds(goal_col, goal_row) ||
    isBlocked(grid, blocked, start_col, start_row) ||
    isBlocked(grid, blocked, goal_col, goal_row))
  {
    return {};
  }

  const std::size_t cell_count = grid.cellCount();
  const std::size_t start_index = grid.index(start_col, start_row);
  const std::size_t goal_index = grid.index(goal_col, goal_row);
  std::vector<double> g_score(
    cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(cell_count, -1);
  std::vector<bool> closed(cell_count, false);

  g_score[start_index] = 0.0;

  auto heuristic = [&](int col, int row) {
    
    const double dc = static_cast<double>(col - goal_col);
    const double dr = static_cast<double>(row - goal_row);

    return std::hypot(dc, dr) * grid.resolution;
  };

  struct Entry
  {
    double f;
    int col;
    int row;
  };

  auto cmp = [](const Entry & a, const Entry & b) {
    return a.f > b.f;
  };

  std::priority_queue<Entry, std::vector<Entry>, decltype(cmp)> open(cmp);

  
  // start the code w the first cell
  open.push({heuristic(start_col, start_row), start_col, start_row});

  // cells needed to be explored
  while (!open.empty()) {
    const Entry current = open.top();
    open.pop();

    const std::size_t current_index = grid.index(current.col, current.row);

    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;

    // if the goal is reached then we can reconstruct the path
    if (current_index == goal_index) {
      std::vector<int> reversed;
      int index = static_cast<int>(goal_index);

      // when it is the first cell with no comeback it will be -1 and the loop will stop
      while (index != -1) {
        reversed.push_back(index);
        index = came_from[static_cast<std::size_t>(index)];
      }
      return std::vector<int>(reversed.rbegin(), reversed.rend());
    }

    // otherwise we add all the neighbors to the open list
    for (int neighbor = 0; neighbor < 9; ++neighbor) {
      // TODO: coordinates, validity checks, move cost, and relaxation
      // 2 offsets
      const int dc = (neighbor % 3) - 1;
      const int dr = (neighbor / 3) - 1;

      // skip the current cell since it is already explored
      if (dc == 0 && dr == 0) {
        continue;
      }
      // neighbor coordinates
      const int nc = current.col + dc;
      const int nr = current.row + dr;

      // if the square is blocked then fade
      if (isBlocked(grid, blocked, nc, nr)) {
        continue;
      }

      // done after to prevent out of bounds error
      const std::size_t neighbor_index = grid.index(nc, nr);

      // if it's already explored then fade
      if (closed[neighbor_index]) {
        continue;
      }

      // for diagonal, we also check the orthogonal neighbors to prevent cutting corners
      if (dc != 0 && dr != 0) {
        if (
          isBlocked(grid, blocked, current.col + dc, current.row) ||
          isBlocked(grid, blocked, current.col, current.row + dr)
        ) {
          continue;
        }
      }

      // update the g_score with the new move cost
      const double step_cost =
        std::hypot(static_cast<double>(dc),
                  static_cast<double>(dr)) * grid.resolution;

      const double tentative_g =
        g_score[current_index] + step_cost;

      // checks if this score to the neighbor is better than a prev one
      if (tentative_g < g_score[neighbor_index]) {
        g_score[neighbor_index] = tentative_g;

        came_from[neighbor_index] =
          static_cast<int>(current_index);

        open.push({
          tentative_g + heuristic(nc, nr),
          nc,
          nr
        });
      }

    }
  }
  return {};
}

}  // namespace robonav_training
