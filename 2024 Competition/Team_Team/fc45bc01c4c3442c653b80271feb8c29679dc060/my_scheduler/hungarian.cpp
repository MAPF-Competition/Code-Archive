#include <vector>
#include <limits>
#include <algorithm>

namespace MyPlanner {

    // ------------------------------------------------------
    // 2) Step 1: Row/Column reduction (no negative handling)
    // ------------------------------------------------------
    static void step1(std::vector<std::vector<int>> &m, int &step) {
        int n = (int)m.size();

        // Subtract min from each row
        for (int r = 0; r < n; r++) {
            int minval = *std::min_element(m[r].begin(), m[r].end());
            if (minval > 0) {
                for (int c = 0; c < n; c++) {
                    m[r][c] -= minval;
                }
            }
        }

        // Subtract min from each column
        for (int c = 0; c < n; c++) {
            int minval = std::numeric_limits<int>::max();
            for (int r = 0; r < n; r++) {
                if (m[r][c] < minval) minval = m[r][c];
            }
            // If minval is 0 or INT_MAX, do nothing
            if (minval > 0 && minval < std::numeric_limits<int>::max()) {
                for (int r = 0; r < n; r++) {
                    m[r][c] -= minval;
                }
            }
        }

        step = 2;
    }

    // ----------------------------------------------------
    // 2a) Utility: clear cover vectors
    // ----------------------------------------------------
    static inline void clear_covers(std::vector<int> &v) {
        std::fill(v.begin(), v.end(), 0);
    }

    // ------------------------------------------------------------------------
    // 3) Step 2: Star zeros that are found (greedy initial assignment)
    // ------------------------------------------------------------------------
    static void step2(const std::vector<std::vector<int>> &m,
                      std::vector<std::vector<int>> &mask,
                      std::vector<int> &rowCover,
                      std::vector<int> &colCover,
                      int &step) {
        int n = (int)m.size();

        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                if (m[r][c] == 0 && rowCover[r] == 0 && colCover[c] == 0) {
                    mask[r][c] = 1; // star this zero
                    rowCover[r] = 1;
                    colCover[c] = 1;
                }
            }
        }

        // Reset covers for next steps
        clear_covers(rowCover);
        clear_covers(colCover);
        step = 3;
    }

    // --------------------------------------------------------------------
    // 4) Step 3: Cover all columns containing a starred zero
    // --------------------------------------------------------------------
    static void step3(const std::vector<std::vector<int>> &mask,
                      std::vector<int> &colCover,
                      int &step) {
        int n = (int)mask.size();
        int colcount = 0;

        for (int c = 0; c < n; c++) {
            for (int r = 0; r < n; r++) {
                if (mask[r][c] == 1) {
                    colCover[c] = 1;
                    break; // proceed to next column
                }
            }
            if (colCover[c] == 1) colcount++;
        }

        // If all columns covered, we are done
        if (colcount >= n) {
            step = 7;
        } else {
            step = 4;
        }
    }

    // ---------------------------------------------------------------
    // 5) We keep these small find_* helpers (inlined for performance)
    // ---------------------------------------------------------------
    static inline void find_star_in_col(int c, int &r,
                                        const std::vector<std::vector<int>> &mask) {
        r = -1;
        int n = (int)mask.size();
        for (int i = 0; i < n; i++) {
            if (mask[i][c] == 1) {
                r = i;
                break;
            }
        }
    }

    static inline void find_prime_in_row(int r, int &c,
                                         const std::vector<std::vector<int>> &mask) {
        c = -1;
        int n = (int)mask.size();
        for (int j = 0; j < n; j++) {
            if (mask[r][j] == 2) {
                c = j;
                break;
            }
        }
    }

    // -----------------------------------------------------------
    // 5a) For augmenting path toggling: stars->0, primes->1
    // -----------------------------------------------------------
    static void augment_path(std::vector<std::vector<int>> &path,
                             int path_count,
                             std::vector<std::vector<int>> &mask) {
        for (int i = 0; i < path_count; i++) {
            int r = path[i][0];
            int c = path[i][1];
            if (mask[r][c] == 1) {
                mask[r][c] = 0;
            } else if (mask[r][c] == 2) {
                mask[r][c] = 1;
            }
        }
    }

    static void erase_primes(std::vector<std::vector<int>> &mask) {
        for (auto &row : mask) {
            for (auto &val : row) {
                if (val == 2) val = 0;
            }
        }
    }

    // -----------------------------------------------------------
    // 6) Step 5: Use the discovered prime to find an augmenting
    //    path, flip stars & primes along it, then reset covers.
    // -----------------------------------------------------------
    static void step5(std::vector<std::vector<int>> &path,
                      int path_row_0,
                      int path_col_0,
                      std::vector<std::vector<int>> &mask,
                      std::vector<int> &rowCover,
                      std::vector<int> &colCover,
                      int &step) {
        int path_count = 1;
        path[0][0] = path_row_0;
        path[0][1] = path_col_0;

        bool done = false;
        int r = -1, c = -1;
        while (!done) {
            // Find a starred zero in the column of the last prime
            find_star_in_col(path[path_count - 1][1], r, mask);
            if (r > -1) {
                path_count++;
                path[path_count - 1][0] = r;
                path[path_count - 1][1] = path[path_count - 2][1];
            } else {
                done = true;
            }

            if (!done) {
                // Find a primed zero in the row of the newly found star
                find_prime_in_row(path[path_count - 1][0], c, mask);
                path_count++;
                path[path_count - 1][0] = path[path_count - 2][0];
                path[path_count - 1][1] = c;
            }
        }

        // Invert stars/primes along the path
        augment_path(path, path_count, mask);

        // Clear covers and erase primes
        clear_covers(rowCover);
        clear_covers(colCover);
        erase_primes(mask);

        step = 3;
    }

    // ---------------------------------------------------------------
    // 7) Step 6: Adjust the matrix by the smallest uncovered value
    //    THEN rebuild the zero-locations structure for faster step4.
    // ---------------------------------------------------------------
    static void buildZeroInRow(const std::vector<std::vector<int>> &m,
                               std::vector<std::vector<int>> &zeroInRow) {
        int n = (int)m.size();
        for (int r = 0; r < n; r++) {
            zeroInRow[r].clear();
            for (int c = 0; c < n; c++) {
                if (m[r][c] == 0) {
                    zeroInRow[r].push_back(c);
                }
            }
        }
    }

    static void step6(std::vector<std::vector<int>> &m,
                      const std::vector<int> &rowCover,
                      const std::vector<int> &colCover,
                      int &step,
                      std::vector<std::vector<int>> &zeroInRow) {
        int n = (int)m.size();
        int minval = std::numeric_limits<int>::max();

        // Find the smallest uncovered value
        for (int r = 0; r < n; r++) {
            if (rowCover[r] == 1) continue;
            for (int c = 0; c < n; c++) {
                if (colCover[c] == 0 && m[r][c] < minval) {
                    minval = m[r][c];
                }
            }
        }
        if (minval == std::numeric_limits<int>::max()) {
            // Something degenerate; no uncovered cells, or all infinite
            step = 7; // effectively done
            return;
        }

        // Update matrix
        for (int r = 0; r < n; r++) {
            for (int c = 0; c < n; c++) {
                if (rowCover[r] == 1) {
                    m[r][c] += minval;
                }
                if (colCover[c] == 0) {
                    m[r][c] -= minval;
                }
            }
        }

        // Rebuild zero-locations since matrix changed
        buildZeroInRow(m, zeroInRow);

        step = 4;
    }

    // ---------------------------------------------------------
    // 8) Step 4: Find an uncovered zero via zeroInRow lists
    //            If none found -> step6
    //            Otherwise prime it and ...
    // ---------------------------------------------------------
    static void step4(const std::vector<std::vector<int>> &m,
                      std::vector<std::vector<int>> &mask,
                      std::vector<int> &rowCover,
                      std::vector<int> &colCover,
                      int &path_row_0,
                      int &path_col_0,
                      int &step,
                      const std::vector<std::vector<int>> &zeroInRow) {
        int n = (int)m.size();
        bool done = false;

        while (!done) {
            int row = -1, col = -1;

            // Instead of scanning the entire matrix, use zeroInRow:
            for (int r = 0; r < n && row == -1; r++) {
                if (rowCover[r] == 1) continue;
                // Check only columns where m[r][c] == 0
                for (int cCandidate : zeroInRow[r]) {
                    if (colCover[cCandidate] == 0 && mask[r][cCandidate] == 0) {
                        // Found an uncovered zero
                        row = r;
                        col = cCandidate;
                        break;
                    }
                }
            }

            // If no uncovered zero found
            if (row == -1) {
                step = 6;
                done = true;
            } else {
                // Prime the found zero
                mask[row][col] = 2;

                // Check if there's a starred zero in that row
                int star_col = -1;
                for (int c2 = 0; c2 < n; c2++) {
                    if (mask[row][c2] == 1) {
                        star_col = c2;
                        break;
                    }
                }

                if (star_col != -1) {
                    // Cover this row and uncover the column of the star
                    rowCover[row] = 1;
                    colCover[star_col] = 0;
                } else {
                    // No starred zero in row -> we found the start of augmenting path
                    step = 5;
                    path_row_0 = row;
                    path_col_0 = col;
                    done = true;
                }
            }
        }
    }

    // ------------------------------------------------------------
    // 9) Main solver: hungarian_solve
    // ------------------------------------------------------------
    std::vector<int> hungarian_solve(const std::vector<std::vector<int>> &cost_matrix)
    {
        // 1) Copy input so we can safely modify
        int original_rows = (int) cost_matrix.size();
        if (original_rows == 0) return {};
        int original_cols = (int) cost_matrix[0].size();
        if (original_cols == 0) return std::vector<int>(original_rows, -1);

        // We will work on a local matrix
        std::vector<std::vector<int>> matrix = cost_matrix;

        // 2) Since all costs are guaranteed non-negative, we skip negative shifting entirely

        // 3) Pad to square if needed
        int n = (int) matrix.size();  // The new dimension after padding

        // 4) Prepare structures
        //    mask: 0=none, 1=star, 2=prime
        std::vector<std::vector<int>> mask(n, std::vector<int>(n, 0));
        std::vector<int> rowCover(n, 0), colCover(n, 0);
        // 'path' for the augmenting path
        std::vector<std::vector<int>> path(n*2, std::vector<int>(2, 0));
        int path_row_0 = -1, path_col_0 = -1;
        int step = 1;
        bool done = false;

        // For faster zero lookups:
        // We'll rebuild zeroInRow after step1 and step6
        std::vector<std::vector<int>> zeroInRow(n);

        // 5) Main loop
        while (!done) {
            switch (step) {
                case 1:
                    step1(matrix, step);
                    // Build zeroInRow now that row/col reductions are done
                    buildZeroInRow(matrix, zeroInRow);
                    break;

                case 2:
                    step2(matrix, mask, rowCover, colCover, step);
                    break;

                case 3:
                    step3(mask, colCover, step);
                    break;

                case 4:
                    step4(matrix, mask, rowCover, colCover,
                          path_row_0, path_col_0, step, zeroInRow);
                    break;

                case 5:
                    step5(path, path_row_0, path_col_0, mask, rowCover, colCover, step);
                    break;

                case 6:
                    step6(matrix, rowCover, colCover, step, zeroInRow);
                    break;

                case 7:
                default:
                    done = true;
                    break;
            }
        }

        // 6) Build assignment for original matrix shape
        //    - If star is in a padded column, or if row >= original_rows,
        //      assignment gets -1.
        std::vector<int> assignment(original_rows, -1);
        for (int i = 0; i < std::min(n, original_rows); i++) {
            for (int j = 0; j < std::min(n, original_cols); j++) {
                if (mask[i][j] == 1) {
                    assignment[i] = j;
                    break; // found the star
                }
            }
        }

        return assignment;
    }
}