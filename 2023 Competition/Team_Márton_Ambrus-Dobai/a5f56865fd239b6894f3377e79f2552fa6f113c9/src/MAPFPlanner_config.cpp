#include <MAPFPlanner.h>

void MAPFPlanner::read_config() {
  constexpr std::string_view config_path = "config.json";
  if (!std::filesystem::exists(config_path)) {
    std::cout << "Config file not found!" << endl;
    return;
  }

  std::ifstream config_file_stream(std::filesystem::path{config_path});
  auto config_json = nlohmann::json::parse(config_file_stream);

  if (auto it = config_json.find("map_proc_type"); it != config_json.end()) {
    if ("Simple" == *it)
      config.map_proc_type = Configuration::MapProcType::Simple;
    else if ("Limited" == *it)
      config.map_proc_type = Configuration::MapProcType::Limited;
    else
      config.map_proc_type = Configuration::MapProcType::Special;
  }

  if (auto it = config_json.find("export_map"); it != config_json.end()) {
    config.export_map = *it;
  }

  if (auto it = config_json.find("map_file_name"); it != config_json.end()) {
    config.map_file_name = *it;
  }

  if (auto it = config_json.find("reuse_path"); it != config_json.end()) {
    config.reuse_path = *it;
  }
};
