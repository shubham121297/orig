#pragma once

#include "osrm/engine_config.hpp"


namespace osrm
{
    namespace engine
    {
        std::istream &operator>>( std::istream &in, EngineConfig::Algorithm &algorithm );
    }
}

inline unsigned generateServerProgramOptions( const int argc,
                                              const char *argv[],
                                              boost::filesystem::path &base_path,
                                              std::string &ip_address,
                                              int &ip_port,
                                              bool &trial,
                                              osrm::EngineConfig &config,
                                              int &requested_thread_num );

std::string routedDIST( double lat1, double lon1, double lat2, double lon2, const char * argv );

std::string table( double lat1, double lon1, double lat2, double lon2, const char * argv );

std::string routed( double lat1, double lon1, double lat2, double lon2, const char * argv );

std::string routedStringDistances( std::string longlat, const char * argv );

std::string routedString( std::string longlat, const char * argv );

std::string near( double lat, double lon, const char * argv, double radius );

std::string addZeroStr( std::string date );


