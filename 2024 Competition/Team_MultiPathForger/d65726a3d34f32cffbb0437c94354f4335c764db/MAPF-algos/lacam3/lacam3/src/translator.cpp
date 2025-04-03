#include "../include/translator.hpp"

std::vector<LACAM_PREFIX_Path> translateConfigsToPaths(const std::vector<LACAM_PREFIX_Config> &configs)
{
  const auto N = configs.front().size();
  auto paths = std::vector<LACAM_PREFIX_Path>(N);
  for (auto i = 0; i < N; ++i) {
    auto T_i = get_path_cost(configs, i);
    for (auto t = 0; t <= T_i; ++t) {
      paths[i].push_back(configs[t][i]);
    }
  }
  return paths;
}

std::vector<LACAM_PREFIX_Config> translatePathsToConfigs(const std::vector<LACAM_PREFIX_Path> &paths)
{
  const auto N = paths.size();
  auto T = 0;
  for (int i = 0; i < N; ++i) {
    T = std::max(T, (int)paths[i].size() - 1);
  }

  std::vector<LACAM_PREFIX_Config> configs(T + 1, LACAM_PREFIX_Config(N, nullptr));
  for (auto i = 0; i < N; ++i) {
    const auto T_i = (int)paths[i].size() - 1;
    for (auto t = 0; t <= T; ++t) {
      configs[t][i] = paths[i][std::min(t, T_i)];
    }
  }
  return configs;
}
