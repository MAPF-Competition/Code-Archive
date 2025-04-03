#include "journey_graph.hpp"

namespace JG{

void make_unique(std::vector<size_t>& vec){
    std::sort(vec.begin(), vec.end());
    auto it = std::unique(vec.begin(), vec.end());
    vec.erase(it, vec.end());
}

int get_dist(const uint32_t& from, const uint32_t to) {
    // TODO: MINIMUM BETWEEN ALL DIRS
    uint32_t source = get_graph().get_node(from, 0);
    return get_hm().get(source, to);
}

PathSegment::PathSegment(size_t edge_id, size_t path_id, JourneyGraph& jg, PathSegment* prev, PathSegment* next): edge_id(edge_id), path_id(path_id), next(next), prev(prev) {
    jg.edges[edge_id].path_segment = this;
}

Path::Path(PathSegment* start_segment, JourneyGraph& jg): start(start_segment), end(jg.findLastSegmentInPath(start_segment)) {
    full_length = jg.edges[start_segment->edge_id].get_full_path_size();
    jg.nodes[jg.task_to_nodes[jg.edges[start_segment->edge_id].task_id].second].edge = start_segment->edge_id;
    jg.nodes[jg.task_to_nodes[jg.edges[start_segment->edge_id].task_id].first].edge = start_segment->edge_id;

}

}

JG::JourneyGraph &get_jg(){
    static JG::JourneyGraph jg;
    return jg;
}