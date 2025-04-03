#ifndef HUNGARIAN
#define HUNGARIAN
#include <vector>

namespace MyPlanner {
// This is the function declaration that other files can call.
// We don't need all the internal helper declarations here—just the public function(s).
    std::vector<int> hungarian_solve(const std::vector<std::vector<int>> &cost_matrix);

}
#endif //HUNGARIAN