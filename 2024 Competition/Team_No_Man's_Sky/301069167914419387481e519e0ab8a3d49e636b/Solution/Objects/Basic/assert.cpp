#include "assert.hpp"

#include <cstdlib>

bool my_assert_failed(const std::string &message, const std::string &filename, const int line) {
    std::cerr.flush();
    std::cout.flush();
    std::cerr << "assert failed at " << filename << ":" << line << '\n';
    std::cerr << "message: \"" << message << "\"\n";
    std::cerr.flush();
    std::cout.flush();
    throw std::runtime_error(message + ", failed at: " + filename + ":" + std::to_string(line));
    std::exit(100);
    //while(true){}
    return true;
}
