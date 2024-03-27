#include <MAPFPlanner.h>

int MAPFPlanner::get_cell_type(int loc) {
  if (env->map[loc]) return 16;

  int type = 0;

  enum Allowed : unsigned int { LEFT = 1U, DOWN = 2U, RIGHT = 4U, UP = 8U };
  for (int i = 0; -1 != neighbours[loc][i]; i++) {
    if (neighbours[loc][i] == loc + 1) {
      type |= Allowed::LEFT;
    }

    if (neighbours[loc][i] == loc - 1) {
      type |= Allowed::RIGHT;
    }

    if (neighbours[loc][i] == loc + env->cols) {
      type |= Allowed::DOWN;
    }

    if (neighbours[loc][i] == loc - env->cols) {
      type |= Allowed::UP;
    }
  }

  return type;
}

void MAPFPlanner::write_map(std::ostream& os) {
  // Write cell types
  {
    os << "\"types\": [\n";

    for (int i = 0; i < env->rows; i++) {
      int j = 0;
      os << "\t[" << get_cell_type(i * env->cols + j);
      for (j = 1; j < env->cols; j++) {
        os << "," << get_cell_type(i * env->cols + j);
      }
      os << "]" << (i < env->rows - 1 ? "," : "") << "\n";
    }

    os << "],\n";
  }

  // Write component numbers
  {
    os << "\"comps\": [\n";

    for (int i = 0; i < env->rows; i++) {
      int j = 0;
      os << "\t[" << components[i * env->cols + j];
      for (; j < env->cols; j++) {
        os << "," << components[i * env->cols + j];
      }
      os << "]" << (i < env->rows - 1 ? "," : "") << "\n";
    }

    os << "]\n";
  }
}

void MAPFPlanner::export_map() {
  std::ofstream export_stream(config.map_file_name);
  if (!export_stream) return;

  export_stream << "{\n";

  export_stream << "\"rows\": " << env->rows << ",\n";
  export_stream << "\"cols\": " << env->cols << ",\n";

  write_map(export_stream);

  export_stream << "}\n";
}
