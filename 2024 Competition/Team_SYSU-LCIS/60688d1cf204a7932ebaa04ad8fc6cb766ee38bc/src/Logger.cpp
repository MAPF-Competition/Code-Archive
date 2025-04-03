#include "Logger.h"

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
src::severity_logger< logging::trivial::severity_level > lg;


Logger::Logger(std::string filename, int severity){
    this->core = logging::core::get();
    logging::add_common_attributes();
    logging::core::get()->set_filter
    (
        logging::trivial::severity >= severity
    );

    if (filename != "")
        logging::add_file_log
        (
            keywords::file_name = filename,
            keywords::format = "[%TimeStamp%]: *%Severity%* %Message%"
        );
}

void Logger::flush(){
    this->core->flush();
}


void Logger::log_info(std::string input)
{

    BOOST_LOG_SEV(lg, logging::trivial::info) << input;
}


void Logger::log_info(std::string input, int timestep)
{
    log_info("[timestep=" + std::to_string(timestep) + "] " + input);
}


void Logger::log_fatal(std::string input, int timestep)
{
    log_fatal("[timestep=" + std::to_string(timestep) + "] " + input);
}


void Logger::log_fatal(std::string input)
{

    BOOST_LOG_SEV(lg, logging::trivial::fatal) << input;
}


void Logger::log_warning(std::string input)
{

    BOOST_LOG_SEV(lg, logging::trivial::warning) << input;
}


void Logger::log_warning(std::string input, int timestep)
{
    log_warning("[timestep=" + std::to_string(timestep) + "] " + input);
}
