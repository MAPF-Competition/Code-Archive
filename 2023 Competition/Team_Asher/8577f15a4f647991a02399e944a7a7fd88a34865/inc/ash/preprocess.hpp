#pragma once
#include "SharedEnv.h"
#include "ash/utils.hpp"

#include <vector>
#include <ostream>

namespace ash {

class Preprocessor {
    public:

        typedef uint16_t Distance;
        static constexpr Distance MAX_DISTANCE =
            std::numeric_limits<Distance>::max();

        void init(const SharedEnvironment& env, bool multithreading = false);

        Distance get_shortest_distance(const AgentPosition& src,
                int dst) const;

        const std::vector<int>& get_free_locations() const
        {
            return free_locations;
        }

        double get_elapsed() const
        {
            return elapsed;
        }

        double get_memory_usage() const;

        int get_degree(int loc) const
        {
            return cell_properties[loc].degree;
        }

        int get_depth(int loc) const
        {
            return cell_properties[loc].depth;
        }

        bool is_deadend(int loc) const
        {
            return cell_properties[loc].is_deadend;
        }

    private:

        // [type definitions]

        typedef std::vector<Distance> DistanceMap;

        struct CellProperties
        {
            int degree;
            int depth;
            bool is_deadend;
        };

        // [helper methods]

        Distance* get_distance_map(int loc)
        {
            return &distances[4*free_locations.size()*location_index[loc]];
        }

        const Distance* get_distance_map(int loc) const
        {
            return &distances[4*free_locations.size()*location_index[loc]];
        }

        Distance& at(Distance* dm, const AgentPosition& src)
        {
            return dm[location_index[src.location]*4 +
                      static_cast<int>(src.orientation)];
        }

        Distance at(const Distance* dm, const AgentPosition& src) const
        {
            return dm[location_index[src.location]*4 +
                      static_cast<int>(src.orientation)];
        }

        void fill_map(int origin);

        void fill_cell_properties();

        // [attributes]

        const SharedEnvironment* env;
        double elapsed;
        vector<CellProperties> cell_properties;
        vector<int> free_locations;
        vector<int> location_index;
        std::vector<Distance> distances;
};

} // namespace ash

