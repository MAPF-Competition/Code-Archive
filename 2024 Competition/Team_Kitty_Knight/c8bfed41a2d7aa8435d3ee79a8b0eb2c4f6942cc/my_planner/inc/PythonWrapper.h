#pragma once
#include <pybind11/embed.h>
#include <pybind11/stl.h>

namespace py=pybind11;

class PythonWrapper {
public:
    py::scoped_interpreter guard;
    
    PythonWrapper() {
        auto sys=py::module_::import("sys");
        sys.attr("path").attr("append")("./my_python");
    }

    void test() {

        try {
            auto tester_module=py::module_::import("tester");
            auto tester=tester_module.attr("Tester")();
            tester.attr("test_print")();

            std::vector<int> vals={1,2,3};
            tester.attr("test_torch")(vals);

            tester.attr("test_nn")();

            // auto _ret=pybind11::cast<std::vector<int>>(ret);
            // std::cout<<"returned: "<<std::endl;
            // for (auto v:_ret) {
            //     std::cout<<v<<std::endl;
            // }
        } catch (py::error_already_set &e) {
            // Handle Python exceptions
            std::cerr << "Python error: " << e.what() << std::endl;
            exit(-991);
        }
    }
};