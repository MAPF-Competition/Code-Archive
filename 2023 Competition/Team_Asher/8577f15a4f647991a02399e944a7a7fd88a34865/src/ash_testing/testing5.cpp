#include "Grid.h"
#include "SharedEnv.h"
#include "ash/preprocess.hpp"

#include <iostream>
#include <iomanip>
#include <map>
#include <random>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

po::variables_map vm;

//template<class Ret>
//using PreprocessorF = Ret (ash::Preprocessor::*)(int)

template<class Ret>
void print_map(std::ostream& out, const SharedEnvironment& env,
        const ash::Preprocessor& preprocessor, Ret (ash::Preprocessor::*f)(int) const)
{
    for (int i = 0; i < env.map.size(); ++i)
    {
        if (i>0 && i%env.cols==0)
        {
            out << '\n';
        }
        if (env.map[i])
        {
            out << '@';
        }
        else
        {
            out << (preprocessor.*f)(i);
        }
    }
}

int main(int argc, char* argv[])
{
    po::options_description desc("Allowed options");

    desc.add_options()("help", "produce help message")
        ("mt", "Multithreaded map preprocessing")
        ("inputFile", po::value<std::string>()->required(),
         "input map file");

    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    po::notify(vm);

    auto input_map = vm["inputFile"].as<std::string>();
    bool multithreading = vm.count("mt");

    Grid map(input_map);
    SharedEnvironment env;
    env.rows = map.rows;
    env.cols = map.cols;
    env.map = std::move(map.map);
    std::cout << std::endl;

    ash::Preprocessor preprocessor;
    std::cout << "Multithreading: " << std::boolalpha << multithreading << std::endl;

    preprocessor.init(env, multithreading);
    std::cout << "Preprocessor init time: " << preprocessor.get_elapsed() << " s.\n"
              << "Memory (MB): " << preprocessor.get_memory_usage() << std::endl;

    print_map(std::cout, env, preprocessor, &ash::Preprocessor::get_degree);
    std::cout << "\n\n" << std::endl;
    print_map(std::cout, env, preprocessor, &ash::Preprocessor::get_depth);
    std::cout << "\n\n" << std::endl;

    std::cout << std::noboolalpha;
    print_map(std::cout, env, preprocessor, &ash::Preprocessor::is_deadend);
    std::cout << std::endl;
}

