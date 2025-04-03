#include "lap.h"
#include <vector>
#include <limits>

namespace MyPlanner {

    static const int INF = std::numeric_limits<int>::max();

    std::vector<int> lap(const std::vector<std::vector<int>>& cost_matrix)
    {
        const int n = static_cast<int>(cost_matrix.size());
        // -------------------------------------------------------------------------
        // The JV algorithm uses the following major arrays (n is the problem size):
        //   - u: Stores the potential (dual variables) for each row.
        //   - v: Stores the potential (dual variables) for each column.
        //   - p: p[j] = i means: column j is matched with row i.
        //   - way: For each column j, way[j] indicates the column predecessor
        //          in the augmenting path. Used during the shortest path search.
        //
        // We'll implement an integer-based version. In practice, JV is often done
        // with floating types, but this is valid for integer cost as well.
        // -------------------------------------------------------------------------

        std::vector<int> u(n+1), v(n+1), p(n+1), way(n+1);

        // p[j] = row that is matched to column j.
        // We'll treat indices in [1..n] for convenience, then shift at the end.

        // Initialize matching
        for(int j = 1; j <= n; j++) {
            p[j] = 0;   // column j initially matched with "0" (none)
            v[j] = 0;   // column j's potential
        }
        for(int i = 1; i <= n; i++) {
            p[0] = i;      // start augmenting path from row i
            int j0 = 0;    // "current" column
            std::vector<int> minv(n+1, INF);
            std::vector<bool> used(n+1, false);

            do {
                used[j0] = true;
                int i0 = p[j0];  // row currently matched with j0
                int j1 = 0;
                int delta = INF;

                // Explore columns to advance in the BFS / shortest path
                for(int j = 1; j <= n; j++) {
                    if(!used[j]) {
                        // Cost from i0 to j plus adjusted potential
                        int cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                        if(cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if(minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                // Update potentials
                for(int j = 0; j <= n; j++) {
                    if(used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while(p[j0] != 0);

            // Now we have an augmenting path. Reconstruct matching.
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while(j0 != 0);
        }

        // p[j] = i means column j is matched with row i.
        // We want a result assignment[i] = j for each row i in [1..n].
        // We'll build that in assignment (shift indices back to 0-based).
        std::vector<int> assignment(n, -1);
        for(int j = 1; j <= n; j++) {
            int i = p[j];           // row
            if(i >= 1 && i <= n) {
                assignment[i - 1] = j - 1;
            }
        }

        return assignment;
    }
}