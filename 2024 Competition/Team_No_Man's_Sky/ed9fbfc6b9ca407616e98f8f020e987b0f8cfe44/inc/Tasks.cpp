// #include <Tasks.h>
// #include <Objects/Environment/heuristic_matrix.hpp>

// void Task::calc_full_distance(){
//     if (full_distance != -1){
//         return;
//     }
//     auto& hm = get_hm();
//     int dist_sum = 0;
//     for (size_t i = 0; i + 1 < locations.size(); ++i){
//         uint32_t from = locations[i] + 1;
//         uint32_t to = locations[i+1] + 1;
//         dist_sum += hm.get_to_pos(from, to);
//     }
//     full_distance = dist_sum;
// }