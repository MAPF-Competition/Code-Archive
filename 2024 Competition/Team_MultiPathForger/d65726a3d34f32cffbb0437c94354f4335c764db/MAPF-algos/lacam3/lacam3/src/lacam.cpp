#include "../include/lacam.hpp"

LACAM_PREFIX_Solution solve(const LACAM_PREFIX_Instance &ins, int verbose, const Deadline *deadline,
               int seed)
{
  info(1, verbose, deadline, "pre-processing");
  auto planner = LACAM_PREFIX_Planner(&ins, verbose, deadline, seed);
  return planner.solve();
}
