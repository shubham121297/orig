#include "routed.hpp"
#include "server/server.hpp"
#include "util/exception_utils.hpp"
#include "util/log.hpp"
#include "util/meminfo.hpp"
#include "util/version.hpp"

//#include "fixture.hpp"

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include "osrm/engine_config.hpp"
#include "osrm/exception.hpp"
#include "osrm/osrm.hpp"
#include "osrm/storage_config.hpp"

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/any.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <cstdlib>

#include <signal.h>

#include <chrono>
#include <exception>
#include <future>
#include <iostream>
#include <memory>
#include <new>
#include <string>
#include <thread>

#include <iomanip>
#include <sstream>
#include <tuple>
#include <iostream>
#include "calculator.h"
#include <occi.h>

using namespace std;
using namespace oracle::occi;

#ifdef _WIN32
    boost::function0<void> console_ctrl_function;

    BOOL WINAPI console_ctrl_handler( DWORD ctrl_type )
    {
        switch ( ctrl_type )
        {
            case CTRL_C_EVENT :
            case CTRL_BREAK_EVENT :
            case CTRL_CLOSE_EVENT :
            case CTRL_SHUTDOWN_EVENT :
                console_ctrl_function();
            return TRUE;
            default :
                return FALSE;
        }
    }
#endif

using namespace osrm;

const static unsigned INIT_OK_START_ENGINE = 0;
const static unsigned INIT_OK_DO_NOT_START_ENGINE = 1;
const static unsigned INIT_FAILED = -1;

namespace osrm
{
    namespace engine
    {
        std::istream &operator>>( std::istream &in, EngineConfig::Algorithm &algorithm )
        {
            std::string token;
            in >> token;
            boost::to_lower( token );

            if ( token == "ch" || token == "corech" )
                algorithm = EngineConfig::Algorithm::CH;
            else if ( token == "mld" )
                algorithm = EngineConfig::Algorithm::MLD;
            else
                throw util::RuntimeError( token, ErrorCode::UnknownAlgorithm, SOURCE_REF );
            return in;
        }
    }
}

// generate boost::program_options object for the routing part
inline unsigned generateServerProgramOptions( const int argc,
                                              const char *argv[],
                                              boost::filesystem::path &base_path,
                                              std::string &ip_address,
                                              int &ip_port,
                                              bool &trial,
                                              EngineConfig &config,
                                              int &requested_thread_num )
{
    using boost::program_options::value;
    using boost::filesystem::path;

    const auto hardware_threads = std::max<int>( 1, std::thread::hardware_concurrency() );

    // declare a group of options that will be allowed only on command line
    boost::program_options::options_description generic_options( "Options" );
    generic_options.add_options()    //
        ( "version,v", "Show version" )( "help,h", "Show this help message" )(
            "verbosity,l",
            boost::program_options::value<std::string>( &config.verbosity )->default_value( "INFO" ),
            std::string( "Log verbosity level: " + util::LogPolicy::GetLevels() ).c_str() )(
                "trial", value<bool>( &trial )->implicit_value( true ), "Quit after initialization" );

    // declare a group of options that will be allowed on command line
    boost::program_options::options_description config_options( "Configuration" );
    config_options.add_options()    //
        ( "ip,i",
          value<std::string>( &ip_address )->default_value( "0.0.0.0" ),
          "IP address" )    //
        ( "port,p",
          value<int>( &ip_port )->default_value( 5000 ),
          "TCP/IP port" )    //
        ( "threads,t",
          value<int>( &requested_thread_num )->default_value( hardware_threads ),
          "Number of threads to use" )    //
        ( "shared-memory,s",
          value<bool>( &config.use_shared_memory )->implicit_value( true )->default_value( false ),
          "Load data from shared memory" )    //
        ( "algorithm,a",
          value<EngineConfig::Algorithm>( &config.algorithm )
          ->default_value( EngineConfig::Algorithm::CH, "CH" ),
          "Algorithm to use for the data. Can be CH, CoreCH, MLD." )    //
        ( "max-viaroute-size",
          value<int>( &config.max_locations_viaroute )->default_value( 500 ),
          "Max. locations supported in viaroute query" )    //
        ( "max-trip-size",
          value<int>( &config.max_locations_trip )->default_value( 100 ),
          "Max. locations supported in trip query" )    //
        ( "max-table-size",
          value<int>( &config.max_locations_distance_table )->default_value( 100 ),
          "Max. locations supported in distance table query" )    //
        ( "max-matching-size",
          value<int>( &config.max_locations_map_matching )->default_value( 100 ),
          "Max. locations supported in map matching query" )    //
        ( "max-nearest-size",
          value<int>( &config.max_results_nearest )->default_value( 7500 ),
          "Max. results supported in nearest query" )    //
        ( "max-alternatives",
          value<int>( &config.max_alternatives )->default_value( 3 ),
          "Max. number of alternatives supported in the MLD route query" )    //
        ( "max-matching-radius",
          value<double>( &config.max_radius_map_matching )->default_value( 5 ),
          "Max. radius size supported in map matching query" );

    // hidden options, will be allowed on command line, but will not be shown to the user
    boost::program_options::options_description hidden_options( "Hidden options" );
    hidden_options.add_options()(
        "base,b", value<boost::filesystem::path>( &base_path ), "base path to .osrm file" );

    // positional option
    boost::program_options::positional_options_description positional_options;
    positional_options.add( "base", 1 );

    // combine above options for parsing
    boost::program_options::options_description cmdline_options;
    cmdline_options.add( generic_options ).add( config_options ).add( hidden_options );

    const auto *executable = argv[ 0 ];
    boost::program_options::options_description visible_options(
        boost::filesystem::path( executable ).filename().string() + " <base.osrm> [<options>]" );
    visible_options.add( generic_options ).add( config_options );

    // parse command line options
    boost::program_options::variables_map option_variables;
    try
    {
        boost::program_options::store( boost::program_options::command_line_parser( argc, argv )
                                                                   .options( cmdline_options )
                                                                   .positional( positional_options )
                                                                   .run(),
                                       option_variables );
    }
    catch ( const boost::program_options::error &e )
    {
        util::Log( logERROR ) << e.what();
        return INIT_FAILED;
    }

    if ( option_variables.count( "version" ) )
    {
        std::cout << OSRM_VERSION << std::endl;
        return INIT_OK_DO_NOT_START_ENGINE;
    }

    if ( option_variables.count( "help" ) )
    {
        std::cout << visible_options;
        return INIT_OK_DO_NOT_START_ENGINE;
    }

    boost::program_options::notify( option_variables );

    if ( !config.use_shared_memory && option_variables.count( "base" ) )
    {
        return INIT_OK_START_ENGINE;
    }
    else if ( config.use_shared_memory && !option_variables.count( "base" ) )
    {
        return INIT_OK_START_ENGINE;
    }
    else if ( config.use_shared_memory && option_variables.count( "base" ) )
    {
        util::Log( logWARNING ) << "Shared memory settings conflict with path settings.";
    }

    // Adjust number of threads to hardware concurrency
    requested_thread_num = std::min( hardware_threads, requested_thread_num );

    std::cout << visible_options;
    return INIT_OK_DO_NOT_START_ENGINE;
}


/*
int routing()
{

   double lat1,lon1,lat2,lon2;
    string osrmnodes;
    string nodestr;
    double latlon[10000];
    double lat[100000];
    double lon[100000];
    int j = 0 ;
    int m = 0;
    int n = 0;




    while(cin >> latlon[j]){

        if (j%2 > 0){
            lon[n] = latlon[j];
            n++;
        }
         if (j%2 == 0){
            lat[m] = latlon[j];
            m++;
        }



        if (latlon[j] == 000)
            break;

        j++;
    }

        m = 0;
        n = 0;

    for (int k =0; k < j/4; k++){

        lat1 = lat[m];
        lon1 = lon[n];
        lat2 = lat[m+1];
        lon2 = lon[n+1];
        m = m+2;
        n = n+2;
       //auto osrm = getOSRM( "/home/hbss/osrm/data/map.osrm");
    auto osrm = getOSRM( "/data/map.osrm");
        {
        RouteParameters params;
            params.annotations_type = RouteParameters::AnnotationsType::Nodes;
            params.coordinates.push_back({util::FloatLongitude{lat1}, util::FloatLatitude{lon1}});
            params.coordinates.push_back({util::FloatLongitude{lat2}, util::FloatLatitude{lon2}});

            json::Object result;
            osrm.Route(params, result);

            const auto &routes = result.values["routes"].get<json::Array>().values;
            const auto &legs = routes[0].get<json::Object>().values.at("legs").get<json::Array>().values;
            const auto &annotation =
                legs[0].get<json::Object>().values.at("annotation").get<json::Object>();
            const auto &nodes = annotation.values.at("nodes").get<json::Array>().values;
            int length = nodes.size();
            for (int i = 0; i < length; i++)
            {
               int node = nodes[i].get<json::Number>().value;
                if (i+1 == length){
                nodestr = to_string(node);
                osrmnodes=osrmnodes+nodestr;
                }
                else{
                nodestr = to_string(node);
                osrmnodes=osrmnodes+nodestr;
                osrmnodes.push_back(',');
                }
            }
        }

        if(k+1 < j/4)
            osrmnodes.push_back(',');
    }


//cout << osrmnodes;

cout << "\n\n";

return 0;

}*/

string routedDIST( double lat1, double lon1, double lat2, double lon2, const char * argv )
{

    util::LogPolicy::GetInstance().Unmute();






    EngineConfig config;
    boost::filesystem::path base_path;
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore

    config.storage_config =
    {
        argv
    };

    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    //
    config.algorithm = EngineConfig::Algorithm::CH;
    // config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm
    {
        config
    };

    /* if (config.use_shared_memory)
     {
         util::Log() << "Loading from shared memory";
     }*/


    string osrmnodes;
    string nodestr;
    double distance;    ////// to match values in database
    string strdist;
    double time1;



    {

        RouteParameters params;

        json::Object result;


        params.annotations_type = RouteParameters::AnnotationsType::Nodes;
        params.coordinates.push_back(
                                     {
                                         util::FloatLongitude
                                         {
                                             lon1
                                         }, util::FloatLatitude
                                         {
                                             lat1
                                         }
                                     }
                                     );
        params.coordinates.push_back(
                                     {
                                         util::FloatLongitude
                                         {
                                             lon2
                                         }, util::FloatLatitude
                                         {
                                             lat2
                                         }
                                     }
                                     );

        params.overview = RouteParameters::OverviewType::Simplified;
        params.steps = true;
        params.continue_straight = false;
        // cout << "In routed: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        osrm.Route( params, result );
        // cout << "finished route: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;





        // Let's just use the first route

        try
        {
            const auto &routes = result.values[ "routes" ].get<json::Array>().values;
            auto &routes2 = result.values[ "routes" ].get<json::Array>();
            auto &route = routes2.values.at( 0 ).get<json::Object>();
            distance = route.values[ "distance" ].get<json::Number>().value;
            time1 = route.values[ "duration" ].get<json::Number>().value;
            distance = distance * ( 1 / 1609.34 );

            // cout << "finished nodes: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        }
        catch( ... )
        {
            //cout << "New distance ... " << endl;
            double blat = lat1;
            double gridlat = lat2;
            double blon = lon1;
            double gridlon = lon2;
            double dist = ( ( blat - gridlat ) *( blat - gridlat ) + ( blon - gridlon ) *( blon - gridlon ) ) ;
            dist = sqrt( dist ) * 3959.0 * 3.1415 / 180;

            distance = dist;
            time1 = 0.0;




        }
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "got nodes: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << osrmnodes << endl;
        //distance =  distance  * 0.00062137; //convert meters to miles

        //cout << "The distance is " << distance << endl;
        strdist = to_string( ( roundf( distance * 100 ) / 100 ) ) + "^" + to_string( ceil( time1 / 60 ) );

        // cout << distance << endl;
    }

    //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "end time: " << endl;
    //endtimer(timer, 0);

    return strdist;

}


string table( double lat1, double lon1, double lat2, double lon2, const char * argv )
{

    util::LogPolicy::GetInstance().Unmute();






    EngineConfig config;
    boost::filesystem::path base_path;
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore

    config.storage_config =
    {
        argv
    };

    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    //
    config.algorithm = EngineConfig::Algorithm::CH;
    // config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm
    {
        config
    };

    /* if (config.use_shared_memory)
     {
         util::Log() << "Loading from shared memory";
     }*/
    string time1 = "0";
    {

        //overview=simplified&steps=true&continue_straight=false

        TableParameters params;


        json::Object result;


        //params.annotations_type = RouteParameters::AnnotationsType::Nodes;
        params.coordinates.push_back(
                                     {
                                         util::FloatLongitude
                                         {
                                             lon1
                                         }, util::FloatLatitude
                                         {
                                             lat1
                                         }
                                     }
                                     );
        params.coordinates.push_back(
                                     {
                                         util::FloatLongitude
                                         {
                                             lon2
                                         }, util::FloatLatitude
                                         {
                                             lat2
                                         }
                                     }
                                     );
        params.sources.push_back( 0 );
        params.destinations.push_back( 1 );
        /*params.overview = RouteParameters::OverviewType::Simplified;
        params.steps = true;
        params.continue_straight = false;*/
        // cout << "In routed: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        osrm.Table( params, result );
        // cout << "finished route: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;





        // Let's just use the first route

        try
        {

            const auto code = result.values.at( "code" ).get<json::String>().value;
            const auto &durations_array = result.values.at( "durations" ).get<json::Array>().values;
            const auto durations_matrix = durations_array[ 0 ].get<json::Array>().values;
            time1 = to_string( ( int )ceil( ( ceil( durations_matrix[ 0 ].get<json::Number>().value ) / 60 ) )    /*in minutes*/ );


            // cout << "finished nodes: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        }
        catch( ... )
        {
            if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "ERROR with time! " << endl;
            time1 = "0";

        }
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "got nodes: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << osrmnodes << endl;
        //distance =  distance  * 0.00062137; //convert meters to miles

        //cout << "The distance is " << distance << endl;
        // cout << distance << endl;
    }

    //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "end time: " << endl;
    //endtimer(timer, 0);

    return time1;

}



string routed( double lat1, double lon1, double lat2, double lon2, const char * argv )
{

    util::LogPolicy::GetInstance().Unmute();



    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "in routed " << endl;


    EngineConfig config;
    //boost::filesystem::path base_path;
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore

    config.storage_config =
    {
        argv
    };
    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting 21" << endl;
    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    //
    config.algorithm = EngineConfig::Algorithm::CH;
    // config.algorithm = EngineConfig::Algorithm::MLD;
    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting 22" << endl;
    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm
    {
        config
    };
    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting 3" << endl;
    /* if (config.use_shared_memory)
     {
         util::Log() << "Loading from shared memory";
     }*/


    string osrmnodes;
    string nodestr;
    double distance;    ////// to match values in database
    double time1;




    {

        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting 1" << endl;
        //overview=simplified&steps=true&continue_straight=false

        RouteParameters params;


        json::Object result;


        params.annotations_type = RouteParameters::AnnotationsType::Nodes;
        params.coordinates.push_back(
                                     {
                                         util::FloatLongitude
                                         {
                                             lon1
                                         }, util::FloatLatitude
                                         {
                                             lat1
                                         }
                                     }
                                     );
        params.coordinates.push_back(
                                     {
                                         util::FloatLongitude
                                         {
                                             lon2
                                         }, util::FloatLatitude
                                         {
                                             lat2
                                         }
                                     }
                                     );

        params.overview = RouteParameters::OverviewType::Simplified;
        params.steps = true;
        params.continue_straight = false;
        // cout << "In routed: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting 2.1" << endl;
        osrm.Route( params, result );
        // cout << "finished route: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;

        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting 2" << endl;



        // Let's just use the first route

        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "starting " << endl;

        try
        {
            const auto &routes = result.values[ "routes" ].get<json::Array>().values;
            // cout << "got route  1  : " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
            auto &routes2 = result.values[ "routes" ].get<json::Array>();
            // cout << "got route 2 : " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
            auto &route = routes2.values.at( 0 ).get<json::Object>();
            //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "got route 1 : " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
            distance = route.values[ "distance" ].get<json::Number>().value;
            time1 = route.values[ "duration" ].get<json::Number>().value;
            //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "got distance: " <<  distance <<  endl;
            const auto &legs = routes[ 0 ].get<json::Object>().values.at( "legs" ).get<json::Array>().values;
            const auto &annotation = legs[ 0 ].get<json::Object>().values.at( "annotation" ).get<json::Object>();
            const auto &nodes = annotation.values.at( "nodes" ).get<json::Array>().values;
            int length = nodes.size();


            //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "node length : " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << " " << length << endl;
            for ( int i = 0;
                  i < length;
                  i++ )
            {



                unsigned long long node = nodes[ i ].get<json::Number>().value;
                if( node != 0 )
                {
                    if ( i + 1 == length )
                    {
                        nodestr = to_string( node );
                        osrmnodes += nodestr;
                        //cout << nodestr << endl;
                        // }
                    }
                    else
                    {
                        nodestr = to_string( node );
                        osrmnodes += nodestr;
                        osrmnodes += ",";
                        //cout << nodestr << endl;
                        // }
                    }

                }

            }


            distance = distance * ( 1 / 1609.34 );
            if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << distance << endl;

            // cout << "finished nodes: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        }
        catch( ... )
        {
            //cout << "New distance ... " << endl;
            double blat = lat1;
            double gridlat = lat2;
            double blon = lon1;
            double gridlon = lon2;
            double dist = ( ( blat - gridlat ) *( blat - gridlat ) + ( blon - gridlon ) *( blon - gridlon ) ) ;
            dist = sqrt( dist ) * 3959.0 * 3.1415 / 180;

            distance = dist;




        }
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "got nodes: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << osrmnodes << endl;
        //distance =  distance  * 0.00062137; //convert meters to miles

        //cout << "The distance is " << distance << endl;
        string strdist = to_string( ( roundf( distance * 100 ) / 100 ) );

        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Node length " << osrmnodes.length() << endl;
        osrmnodes.push_back( '^' );
        osrmnodes = osrmnodes + strdist;
        osrmnodes.push_back( '^' );
        osrmnodes = osrmnodes + to_string( ceil( time1 / 60 ) );
        // cout << distance << endl;
    }

    //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "end time: " << endl;
    //endtimer(timer, 0);
    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "done " << endl;

    return osrmnodes;

}


string routedStringDistances( string longlat, const char * argv )
{

    util::LogPolicy::GetInstance().Unmute();






    EngineConfig config;
    boost::filesystem::path base_path;
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore

    config.storage_config =
    {
        argv
    };

    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    //
    config.algorithm = EngineConfig::Algorithm::CH;
    // config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm
    {
        config
    };

    /* if (config.use_shared_memory)
     {
         util::Log() << "Loading from shared memory";
     }*/


    string osrmnodes;
    string nodestr;
    ////// to match values in database
    string finalroute = "";



    {

        RouteParameters params;

        json::Object result;
        params.annotations_type = RouteParameters::AnnotationsType::Nodes;
        string temp = longlat;

        while( temp != "" )
        {

            double lon1 = to_number( getNextToken( ( &temp ), "," ) );
            double lat1 = to_number( getNextToken( ( &temp ), "," ) );
            params.coordinates.push_back(
                                         {
                                             util::FloatLongitude
                                             {
                                                 lon1
                                             }, util::FloatLatitude
                                             {
                                                 lat1
                                             }
                                         }
                                         );
            cout << "In routed: " << lon1 << "," << lat1 << endl;

            // cout << "finished route: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;

        }

        params.overview = RouteParameters::OverviewType::Simplified;
        params.steps = true;
        params.continue_straight = false;

        osrm.Route( params, result );





        const auto &routes = result.values.at( "routes" ).get<json::Array>().values;



        for ( const auto &route : routes )
        {
            const auto &route_object = route.get<json::Object>();
            //const auto distance = route_object.values.at("distance").get<json::Number>().value;
            //const auto duration = route_object.values.at("duration").get<json::Number>().value;
            //const auto geometry = route_object.values.at("geometry").get<json::String>().value;
            const auto &legs = route_object.values.at( "legs" ).get<json::Array>().values;

            for ( const auto &leg : legs )
            {

                osrmnodes = "";
                try
                {
                    const auto &leg_object = leg.get<json::Object>();
                    double distance = leg_object.values.at( "distance" ).get<json::Number>().value;
                    distance = distance * ( 1 / 1609.34 );
                    osrmnodes = to_string( ( roundf( distance * 100 ) / 100 ) );
                }
                catch( ... )
                {


                    string temp2 = longlat;

                    double lon1 = ( int )to_number( getNextToken( ( &temp2 ), "," ) );
                    double lat1 = ( int )to_number( getNextToken( ( &temp2 ), "," ) );
                    double lon2 =( int )to_number( getNextToken( ( &temp2 ), "," ) );
                    double lat2 = ( int )to_number( getNextToken( ( &temp2 ), "," ) );

                    double blat = lat1;
                    double gridlat = lat2;
                    double blon = lon1;
                    double gridlon = lon2;
                    double dist = ( ( blat - gridlat ) *( blat - gridlat ) + ( blon - gridlon ) *( blon - gridlon ) ) ;
                    dist = sqrt( dist ) * 3959.0 * 3.1415 / 180;



                    osrmnodes = to_string( ( roundf( dist * 100 ) / 100 ) );


                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "ERROR DISTANCE so using " << dist << endl;

                }

                getNextToken( ( &longlat ), "," );
                getNextToken( ( &longlat ), "," );



                if( finalroute == "" )
                {
                    finalroute = osrmnodes;
                }
                else
                {
                    finalroute = finalroute + "?" + osrmnodes;
                }

            }
        }






    }



    return finalroute;

}















string routedString( string longlat, const char * argv )
{

    util::LogPolicy::GetInstance().Unmute();






    EngineConfig config;
    boost::filesystem::path base_path;
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore

    config.storage_config =
    {
        argv
    };

    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    //
    config.algorithm = EngineConfig::Algorithm::CH;
    // config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm
    {
        config
    };

    /* if (config.use_shared_memory)
     {
         util::Log() << "Loading from shared memory";
     }*/


    string osrmnodes;
    string nodestr;
    ////// to match values in database
    string finalroute = "";



    {

        RouteParameters params;

        json::Object result;
        params.annotations_type = RouteParameters::AnnotationsType::Nodes;
        string temp = longlat;

        while( temp != "" )
        {

            double lon1 = to_number( getNextToken( ( &temp ), "," ) );
            double lat1 = to_number( getNextToken( ( &temp ), "," ) );
            params.coordinates.push_back(
                                         {
                                             util::FloatLongitude
                                             {
                                                 lon1
                                             }, util::FloatLatitude
                                             {
                                                 lat1
                                             }
                                         }
                                         );
            //params.overview = RouteParameters::OverviewType::Simplified;
            //params.steps = true;
            //params.continue_straight = false;
            //cout << "In routed: " << lon1 <<"," << lat1 << endl;

            // cout << "finished route: " << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;

        }



        osrm.Route( params, result );





        const auto &routes = result.values.at( "routes" ).get<json::Array>().values;



        for ( const auto &route : routes )
        {
            const auto &route_object = route.get<json::Object>();
            //const auto distance = route_object.values.at("distance").get<json::Number>().value;
            //const auto duration = route_object.values.at("duration").get<json::Number>().value;
            //const auto geometry = route_object.values.at("geometry").get<json::String>().value;
            const auto &legs = route_object.values.at( "legs" ).get<json::Array>().values;

            for ( const auto &leg : legs )
            {
                osrmnodes = "";
                try
                {
                    const auto &leg_object = leg.get<json::Object>();
                    const auto &annotation = leg_object.values.at( "annotation" ).get<json::Object>();
                    const auto &nodes = annotation.values.at( "nodes" ).get<json::Array>().values;
                    double distance = leg_object.values.at( "distance" ).get<json::Number>().value;
                    int length = nodes.size();





                    for ( int i = 0;
                          i < length;
                          i++ )
                    {

                        unsigned long long node = nodes[ i ].get<json::Number>().value;

                        if ( i + 1 == length )
                        {
                            nodestr = to_string( node );
                            distance = distance * ( 1 / 1609.34 );
                            osrmnodes = osrmnodes + nodestr + "^" + to_string( ( roundf( distance * 100 ) / 100 ) );

                        }
                        else
                        {
                            nodestr = to_string( node );
                            osrmnodes = osrmnodes + nodestr;
                            osrmnodes.push_back( ',' );
                        }


                    }

                }




                catch( ... )
                {


                    string temp2 = longlat;

                    double lon1 = ( int )to_number( getNextToken( ( &temp2 ), "," ) );
                    double lat1 = ( int )to_number( getNextToken( ( &temp2 ), "," ) );
                    double lon2 =( int )to_number( getNextToken( ( &temp2 ), "," ) );
                    double lat2 = ( int )to_number( getNextToken( ( &temp2 ), "," ) );

                    double blat = lat1;
                    double gridlat = lat2;
                    double blon = lon1;
                    double gridlon = lon2;
                    double dist = ( ( blat - gridlat ) *( blat - gridlat ) + ( blon - gridlon ) *( blon - gridlon ) ) ;
                    dist = sqrt( dist ) * 3959.0 * 3.1415 / 180;



                    osrmnodes = "^" + to_string( ( roundf( dist * 100 ) / 100 ) );


                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "ERROR DISTANCE so using " << dist << endl;

                }

                getNextToken( ( &longlat ), "," );
                getNextToken( ( &longlat ), "," );



                if( finalroute == "" )
                {
                    finalroute = osrmnodes;
                }
                else
                {
                    finalroute = finalroute + "?" + osrmnodes;
                }

            }
        }






    }

    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "final route " << finalroute << endl;

    return finalroute;

}


string near( double lat, double lon, const char * argv, double radius )
{

    string osrmnodes;

    util::LogPolicy::GetInstance().Unmute();


    EngineConfig config;
    boost::filesystem::path base_path;
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore

    config.storage_config =
    {
        argv
    };

    // We support two routing speed up techniques:
    // - Contraction Hierarchies (CH): requires extract+contract pre-processing
    // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
    //
    config.algorithm = EngineConfig::Algorithm::CH;
    // config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
    const OSRM osrm
    {
        config
    };

    /*if (config.use_shared_memory)
    {
        util::Log() << "Loading from shared memory";
    }*/

    //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Nearest" <<endl;
    NearestParameters params;
    params.number_of_results = 27000;
    params.radiuses.push_back( radius );
    params.coordinates.push_back( osrm::util::Coordinate( osrm::util::FloatLongitude
                                                          {
                                                              lon
                                                          }, osrm::util::FloatLatitude
                                                          {
                                                              lat
                                                          }
                                                          ) );

    int distance = 0;
    json::Object result ;

    //  if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting nearest function" << endl;
    //starttimer(0);

    const auto status2 = osrm.Nearest( params, result );
    int mod = ( int )( floor( radius / 1500 ) );
    if ( mod % 2 != 0 )
    {
        //every even modular since they are paired nodes
        mod++;
    }
    //         if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "ending nearest function " << endl;
    //  starttimer(0);

    //cout << lon << " " << lat << endl;
    // if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting loop function" << endl;
    // starttimer(0);
    if ( status2 == Status::Ok )
    {
        int numberofnodes = 0;
        int total = 0;
        const auto &waypoints = result.values.at( "waypoints" ).get<json::Array>().values;
        for ( const auto &waypoint : waypoints )
        {

            if( numberofnodes % mod == 0 )
            {

                //cout << lon << " " << lat << endl;
                const auto &waypoint_object = waypoint.get<json::Object>();
                const auto &nodes = waypoint_object.values.at( "nodes" ).get<json::Array>().values;
                //const auto &location = waypoint_object.values.at("location").get<json::Array>().values;
                const auto node1 = ( unsigned long long )nodes[ 0 ].get<util::json::Number>().value;
                const auto node2 = ( unsigned long long )nodes[ 1 ].get<util::json::Number>().value;
                //const auto longitude = location[0].get<json::Number>().value;
                //const auto latitude = location[1].get<json::Number>().value;
                const auto distance1 = waypoint_object.values.at( "distance" ).get<json::Number>().value;

                //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse <<  "Distance " << distance1  << endl;
                //if(distance1 - distance > radius*0.03 && distance1 > radius/3 || distance1 <= radius/3 || distance == 0 && distance1 > radius/3 ){


                if( node1 != 0 && node2 != 0 )
                {
                    // take every 2rd edge pair to prune results with minimal loss of results
                    //distance  = distance1;
                    // if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting copy" << endl;
                    // starttimer(0);
                    osrmnodes += to_string( node1 );
                    osrmnodes += ',';
                    osrmnodes += to_string( node2 );
                    osrmnodes += ',';
                    //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "ending copy" << endl;
                    //starttimer(0);
                    total = total + 2;
                }
            }
            numberofnodes = numberofnodes + 2;

            //}
        }
        osrmnodes.pop_back();
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "ending loop function" << endl;
        // starttimer(0);
        //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "osrmnodes number " << total << endl ;

    }
    return osrmnodes;
}

string addZeroStr( string date )
{
    if ( date.length() == 1 )
      date = "0" + date;
    return date;
}


//Internal GSE

#include <string.h>
#include <iostream>
#include <deque>
#include <climits>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <boost/algorithm/string/find.hpp>
using namespace std;
using namespace boost;

#include <boost/lockfree/spsc_queue.hpp> // ring buffer
#include <utility>
#include <iostream>
#include <string>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>


using namespace std;

namespace bip = boost::interprocess;


int main( int argc, const char *argv[] ) try
{
    /*
    string shared_memory = argv[2];




    initosrmshm(shared_memory);
    */

    string client = argv[ 2 ];
    string date = argv[ 3 ];
    string datedow;
    string datestr;
    string upperStr;
    bool updatedb = true;
    string date2 = argv[ 3 ];
    if ( date2.length()>3 )
    {
        string datetemp = date;
        string datemonth;
        string dateday = getNextToken( & ( datetemp ), "-" );
        datetemp = getNextToken( & ( datetemp ), "-" );
        if ( datetemp == "JAN" )
          datemonth = "01";
        if ( datetemp == "FEB" )
          datemonth = "02";
        if ( datetemp == "MAR" )
          datemonth = "03";
        if ( datetemp == "APR" )
          datemonth = "04";
        if ( datetemp == "MAY" )
          datemonth = "05";
        if ( datetemp == "JUN" )
          datemonth = "06";
        if ( datetemp == "JUL" )
          datemonth = "07";
        if ( datetemp == "AUG" )
          datemonth = "08";
        if ( datetemp == "SEP" )
          datemonth = "09";
        if ( datetemp == "OCT" )
          datemonth = "10";
        if ( datetemp == "NOV" )
          datemonth = "11";
        if ( datetemp == "DEC" )
          datemonth = "12";
        datetemp = date;
        getNextToken( & ( datetemp ), "-" );
        getNextToken( & ( datetemp ), "-" );
        string dateyear = getNextToken( & ( datetemp ), "-" );

        if( dateyear.length()>2 )
        {
            dateyear.erase( 0, 2 );
        }
        datestr = dateday + datemonth + dateyear;

    }
    else
    {
        if ( date2 == "SUN" )
          date = "7";
        if ( date2 == "MON" )
          date = "1";
        if ( date2 == "TUE" )
          date = "2";
        if ( date2 == "WED" )
          date = "3";
        if ( date2 == "THU" )
          date = "4";
        if ( date2 == "FRI" )
          date = "5";
        if ( date2 == "SAT" )
          date = "6";
        datestr = date;
    }





    initshm( client, datestr );
    f_cout( LOGDIR + "osrm" + "^" + client + "^" + datestr + ".txt" );
    s_cout( LOGDIR + "gse_states_log" + "^" + client + "^" + datestr + ".txt" );
    gse << "The value of DEBUGROUTED is " << DEBUGROUTED << endl; //***JDC hack

    string osrmnodes;
    // create segment and corresponding allocator

    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Max value of keyt " << std::numeric_limits<key_t> ::max() << endl;

    int shmid = shmget ( key_shared_process, sizeof( char[ MAXPROCESSESCOL ][ MAXLGSTRSIZE ][ NUMBEROFPROCESSES ] ), IPC_CREAT | 0666 );
    shared_process_tab = ( char ( * )[ MAXPROCESSESCOL ][ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyrnum, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_number = ( char( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyrid, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_id = ( char ( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyfunc, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_function = ( char( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keytime, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_timestamp = ( char( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyrlatlong, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_latlon = ( char ( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyrdist, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_distance =( char ( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyredge, sizeof( char[ MAXREQUESTS ][ MAXVYLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_edges = ( char ( * )[ MAXVYLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyrtripidx, sizeof( char[ MAXREQUESTS ][ MAXVYLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_tripidx = ( char ( * )[ MAXVYLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( key_number_of_edges, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_numberofedges = ( char( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );

    shmid = shmget ( keyrtime, sizeof( char[ MAXREQUESTS ][ MAXLGSTRSIZE ] ), IPC_CREAT | 0666 );
    request_time = ( char ( * )[ MAXLGSTRSIZE ] )shmat( shmid, 0, 0 );



    strcpy( ( shared_process_tab[ OSRMSTATUS ][ 0 ] ) , ( "READY" ) );
    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:shared_process_tab[ OSRMSTATUS ][ 0 ]=READY" << endl;

    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "OSRM Server starting ... v2" << endl;
    bool foundhit = false;

    //int cnt = 1;
    //***main while loop - Begin***//
    while ( true )
    {

        std::clock_t startt;
        double durationt;
        startt = std::clock();






        //***main for loop - Begin***//
        for( int i = 0;
             i < MAXREQUESTS;
             i++ )
        {


            // if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "i is " << i << endl;
            //***Case 1 - Begin***//
            if( fetchRequest( i, "ID" ) != "" && ( fetchRequest( i, "FUNC" ) == "Access" || fetchRequest( i, "FUNC" ) == "AccessBOTH" ) && fetchRequest( i, "EDGES" ) == "" )
            {
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 1':fetchRequest( i, 'ID' ) != ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 1':fetchRequest( i, 'FUNC' ) == " << fetchRequest( i, "FUNC" ) << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 1':fetchRequest( i, 'EDGES' ) == " << fetchRequest( i, "EDGES" ) << endl;
                foundhit = true;
                string foo_str ( fetchRequest( i, "LATLONG" ) );
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "processing " << fetchRequest( i, "ID" ) << " " << foo_str << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Case 1 - ID: " << fetchRequest( i, "ID" ) << "; FUNC: " << fetchRequest( i, "FUNC" ) << endl;
                // string foo_str (fetchRequest(i, "LATLONG"));
                int len = foo_str.length();
                int j = 1;
                int distanc1 = 0;
                int distanc2 = 0;
                int previous = 0;
                string substring;
                int index = 0;
                string latlon[ 4 ] =
                {
                    "empty", "empty", "empty", "empty"
                };
                int d_comma = 0;
                int sub_len = 0;
                iterator_range<string::iterator> prev;
                string empty[ 4 ];

                while( foo_str != "" )
                {
                    latlon[ index ] = getNextToken( ( &foo_str ), "," );
                    index++;
                }

                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Done with lat long " << endl;
                /*
                            while (distanc1<len)
                            {
                                iterator_range<string::iterator> r = find_nth(foo_str, ",", j);

                                if (j == 1){
                                    distanc1 = distance(foo_str.begin(), r.begin());
                                    distanc2 = distance(foo_str.begin(), r.begin());
                                    substring = foo_str.substr(previous, distanc2);
                                    sub_len = substring.length();
                                    iterator_range<string::iterator> n = find_nth(substring, ",", 0);
                                    d_comma = distance(substring.begin(), n.begin());
                                    latlon[index] = substring.substr(0,d_comma);
                                    index++;
                                    latlon[index] = substring.substr(d_comma+1,sub_len);
                                    index++;
                                }

                                else{
                                    distanc1 = distance(foo_str.begin(), r.begin());
                                    distanc2 = distance(prev.begin(), r.begin());
                                    substring = foo_str.substr(previous+1, distanc2-1);
                                    sub_len = substring.length();
                                     iterator_range<string::iterator> n = find_nth(substring, ",", 0);
                                    d_comma = distance(substring.begin(), n.begin());
                                    latlon[index] = substring.substr(0,d_comma);
                                    index++;
                                    latlon[index] = substring.substr(d_comma+1,sub_len);
                                    index++;

                                }
                                previous = distanc1;
                                prev = r;
                                j = j+2;
                            }
                */


                const char * messages;

                if ( latlon[ 2 ] != "empty" )
                {

                    string temp;
                    string timebetweenstops;
                    double lat1, lon1, lat2, lon2;
                    stringstream geek0( latlon[ 0 ] );
                    geek0 >> lat1;
                    stringstream geek1( latlon[ 1 ] );
                    geek1 >> lon1;
                    stringstream geek2( latlon[ 2 ] );
                    geek2 >> lat2;
                    stringstream geek3( latlon[ 3 ] );
                    geek3 >> lon2;


                    temp = "";
                    /*if(fetchRequest(i, "FUNC") == "AccessBOTH"){
                        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Doing both" << endl;
                        temp = routed(lat1,lon1,lat2,lon2,argv[1]);
                        timebetweenstops = table(lat1,lon1,lat2,lon2,argv[1]);
                    }
                    else{
                        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "only doing time " << endl;
                        timebetweenstops = table(lat1,lon1,lat2,lon2,argv[1]);
                    }*/

                    temp = routed( lat1, lon1, lat2, lon2, argv[ 1 ] );


                    string substringnodes = getNextToken( &temp, "^" );
                    string substring = getNextToken( &temp, "^" );
                    timebetweenstops = getNextToken( &temp, "^" );


                    if( substringnodes.length() > MAXVYLGSTRSIZE )
                        substringnodes = "ERROR";

                    if( isSameLocation_onlyLatLong( lat1, lon1, lat2, lon2 ) )
                    {
                        substring = "0.00000";
                        timebetweenstops = "0";
                    }

                    updateRequest( i, substring, "DISTANCE" );
                    updateRequest( i, substringnodes, "EDGES" );
                    updateRequest( i, timebetweenstops, "TIME" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 1':updateRequest( " << i << ", " << substring << ", 'DISTANCE' );" << endl;
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 1':updateRequest( " << i << ", " << substringnodes << ", 'EDGES' );" << endl;
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 1':updateRequest( " << i << ", " << timebetweenstops << ", 'TIME' );" << endl;

                    temp = "";

                    for ( int j = 0;
                          j < 4;
                          ++j )
                    {
                        latlon[ j ] = "empty";
                    }

                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Finished with this id" << endl;






                }

            }
            //***Case 1 - End***//


            //***Case 2 - Begin***//
            if( fetchRequest( i, "EDGES" ) == "" && fetchRequest( i, "LATLONG" ) != "" && ( fetchRequest( i, "FUNC" ) == "FindBest" || fetchRequest( i, "FUNC" ) == "FindBestSINGLE" ) )
            {
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'EDGES' ) == ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'LATLONG' ) != ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'FUNC' ) == " << fetchRequest( i, "FUNC" ) << endl;
                //if there is a request and no response
                foundhit = true;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Lat longs for  " << request_latlon[ i ] << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Case 2 - LATLONG: " << fetchRequest( i, "LATLONG" ) << "; FUNC: " << fetchRequest( i, "FUNC" ) << endl;
                time_t curtime;
                curtime = starttimer( 0 );

                string foo_str = request_latlon[ i ];
                int len = foo_str.length();
                int j = 1;
                int distanc1 = 0;
                int distanc2 = 0;
                int previous = 0;
                string substring;
                int index = 0;
                string latlon[ 4 ] =
                {
                    "empty", "empty", "empty", "empty"
                };
                int d_comma = 0;
                int sub_len = 0;
                iterator_range<string::iterator> prev;
                string empty[ 4 ];

                while( foo_str != "" )
                {
                    latlon[ index ] = getNextToken( ( &foo_str ), "," );
                    index++;
                }

                /*

                while (distanc1<len)
                {
                    iterator_range<string::iterator> r = find_nth(foo_str, ",", j);

                    if (j == 1){
                        distanc1 = distance(foo_str.begin(), r.begin());
                        distanc2 = distance(foo_str.begin(), r.begin());
                        substring = foo_str.substr(previous, distanc2);
                        sub_len = substring.length();
                        iterator_range<string::iterator> n = find_nth(substring, ",", 0);
                        d_comma = distance(substring.begin(), n.begin());
                        latlon[index] = substring.substr(0,d_comma);
                        index++;
                        latlon[index] = substring.substr(d_comma+1,sub_len);
                        index++;
                    }

                    else{
                        distanc1 = distance(foo_str.begin(), r.begin());
                        distanc2 = distance(prev.begin(), r.begin());
                        substring = foo_str.substr(previous+1, distanc2-1);
                        sub_len = substring.length();
                         iterator_range<string::iterator> n = find_nth(substring, ",", 0);
                        d_comma = distance(substring.begin(), n.begin());
                        latlon[index] = substring.substr(0,d_comma);
                        index++;
                        latlon[index] = substring.substr(d_comma+1,sub_len);
                        index++;

                    }
                    previous = distanc1;
                    prev = r;
                    j = j+2;
                }

                */



                if ( latlon[ 2 ] != "empty" )
                {

                    string temp;
                    string timebetweenstops;
                    double lat1, lon1, lat2, lon2;
                    stringstream geek0( latlon[ 0 ] );
                    geek0 >> lat1;
                    stringstream geek1( latlon[ 1 ] );
                    geek1 >> lon1;
                    stringstream geek2( latlon[ 2 ] );
                    geek2 >> lat2;
                    stringstream geek3( latlon[ 3 ] );
                    geek3 >> lon2;

                    /*std::clock_t start;
                    double duration;
                    start = std::clock();*/


                    //cout << "Trying for: " << std::get<0>(foo) << " "  << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
                    temp = routed( lat1, lon1, lat2, lon2, argv[ 1 ] );
                    //cout << "Finished for " << std::get<0>(foo) << endl;
                    timebetweenstops = table( lat1, lon1, lat2, lon2, argv[ 1 ] );




                    //temp = routed(lat1,lon1,lat2,lon2,argv[1]);
                    //int indextable = std::get<0>(foo);

                    //////find foo2<0> update distance with returned distance


                    iterator_range<string::iterator> k = find_nth( temp, "^", 0 );
                    double distance2 = distance( k.begin() + 1, temp.end() );
                    string substring = temp.substr( temp.length() -distance2, temp.length() );
                    string substringnodes = temp.substr( 0, ( temp.length() -distance2 -1 ) );
                    //cout << "nodes " << substringnodes << endl;

                    //cout << "The distance is " <<substring << endl;

                    //duration = 1000.0 *( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                    //st
                    //duration = 1000.0 *( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                    //std::cout<<"time routed: "<< duration <<'\n';

                    //cout <<cnt<<endl;
                    //cnt ++;

                    if( substringnodes.length() > MAXVYLGSTRSIZE )
                    {
                        substringnodes = "ERROR";
                    }

                    //updateRequest(i, substring, "DISTANCE");--- don't need the distance to search the tree
                    updateRequest( i, substringnodes, "EDGES" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 2':updateRequest( " << i << ", " << substringnodes << ", 'EDGES' );" << endl;
                    
                    if( isSameLocation_onlyLatLong( lat1, lon1, lat2, lon2 ) )
                    {
                        timebetweenstops = "0";
                    }
                    updateRequest( i, timebetweenstops, "TIME" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 2':updateRequest( " << i << ", " << timebetweenstops << ", 'TIME' );" << endl;


                    //std::pair<int,shm::shared_string> foo2 (std::get<0>(foo),str2);

                    for ( int j = 0;
                          j < 4;
                          ++j )
                    {
                        latlon[ j ] = "empty";
                    }


                    //queueNodesR->push(foo2);
                    // std::cout << "Processed R: " << foo2.first << "\n";


                }



                else
                {

                    string temp;
                    double lat, lon;
                    stringstream geek1( latlon[ 0 ] );
                    geek1 >> lat;
                    stringstream geek2( latlon[ 1 ] );
                    geek2 >> lon;
                    //   if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Starting nearest " << endl;
                    // curtime = starttimer(0);
                    //cout << "nearest";
                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "radius " << to_number( request_numberofedges[ i ] ) << endl;
                    temp = near( lat, lon, argv[ 1 ], to_number( request_numberofedges[ i ] ) );
                    //if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "end nearest " << endl;
                    //   curtime = starttimer(0);
                    if( temp.length() > MAXVYLGSTRSIZE )
                    {

                        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Over max length size : " << temp.length() << " out of " << MAXVYLGSTRSIZE << endl;
                        temp = "ERROR";
                    }

                    if( temp == "" )
                    {
                        if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "bad lat longs, no returned edges" << endl;
                        temp = "ERROR";
                    }

                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Copying into request table for: " << request_id[ i ] << endl;
                    updateRequest( i, temp, "EDGES" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 2':updateRequest( " << i << ", " << temp << ", 'EDGES' );" << endl;
                    
                    //cout << " The edges are " << str2 << endl;
                    for ( int j = 0;
                          j < 4;
                          ++j )
                    {
                        latlon[ j ] = "empty";
                    }

                    // std::cout << "Processed N: " << foo2.first << "\n";

                }



                //cout << "OSRM: Pushing into queueNodesN size now " << queueNodes->read_available() <<endl;
                endtimer( curtime, 0 );

            }
            //***Case 2 - End***//


            //***Case 3 - Begin***//
            if( fetchRequest( i, "DISTANCE" ) == "" && fetchRequest( i, "LATLONG" ) != "" && ( fetchRequest( i, "FUNC" ) == "SlackDist" || fetchRequest( i, "FUNC" ) == "CalcDist" || fetchRequest( i, "FUNC" ) == "FindBestDist" || fetchRequest( i, "FUNC" ) == "FindBestDistSINGLE" || fetchRequest( i, "FUNC" ) == "NEEDDist" ) )
            {
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 3':fetchRequest( i, 'DISTANCE' ) == ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 3':fetchRequest( i, 'LATLONG' ) != ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 3':fetchRequest( i, 'FUNC' ) == " << fetchRequest( i, "FUNC" ) << endl;
                foundhit = true;
                string foo_str = request_latlon[ i ];
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Lat/Long " << foo_str << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Case 3 - LATLONG: " << fetchRequest( i, "LATLONG" ) << "; FUNC: " << fetchRequest( i, "FUNC" ) << endl;
                int len = foo_str.length();
                int j = 1;
                int distanc1 = 0;
                int distanc2 = 0;
                int previous = 0;
                string substring;
                int index = 0;
                string latlon[ 4 ] =
                {
                    "empty", "empty", "empty", "empty"
                };
                int d_comma = 0;
                int sub_len = 0;
                iterator_range<string::iterator> prev;
                string empty[ 4 ];

                while( foo_str != "" )
                {
                    latlon[ index ] = getNextToken( ( &foo_str ), "," );
                    index++;

                }

                /*

                while (distanc1<len)
                {
                    iterator_range<string::iterator> r = find_nth(foo_str, ",", j);

                    if (j == 1){
                        distanc1 = distance(foo_str.begin(), r.begin());
                        distanc2 = distance(foo_str.begin(), r.begin());
                        substring = foo_str.substr(previous, distanc2);
                        sub_len = substring.length();
                        iterator_range<string::iterator> n = find_nth(substring, ",", 0);
                        d_comma = distance(substring.begin(), n.begin());
                        latlon[index] = substring.substr(0,d_comma);
                        index++;
                        latlon[index] = substring.substr(d_comma+1,sub_len);
                        index++;
                    }

                    else{
                        distanc1 = distance(foo_str.begin(), r.begin());
                        distanc2 = distance(prev.begin(), r.begin());
                        substring = foo_str.substr(previous+1, distanc2-1);
                        sub_len = substring.length();
                         iterator_range<string::iterator> n = find_nth(substring, ",", 0);
                        d_comma = distance(substring.begin(), n.begin());
                        latlon[index] = substring.substr(0,d_comma);
                        index++;
                        latlon[index] = substring.substr(d_comma+1,sub_len);
                        index++;

                    }
                    previous = distanc1;
                    prev = r;
                    j = j+2;
                }
                */


                const char * messages;

                //cout << latlon[2] << endl;

                if ( latlon[ 2 ] != "empty" )
                {

                    string temp;
                    double lat1, lon1, lat2, lon2;
                    stringstream geek0( latlon[ 0 ] );
                    geek0 >> lat1;
                    stringstream geek1( latlon[ 1 ] );
                    geek1 >> lon1;
                    stringstream geek2( latlon[ 2 ] );
                    geek2 >> lat2;
                    stringstream geek3( latlon[ 3 ] );
                    geek3 >> lon2;

                    /*std::clock_t start;
                    double duration;
                    start = std::clock();*/



                    // cout << "Trying for: " << std::get<0>(foo) << " "  << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
                    /*if(fetchRequest(i, "FUNC") != "CalcDist"){
                        temp = routedDIST(lat1,lon1,lat2,lon2,argv[1]);
                    }
                    else{
                        temp = "0";
                    }*/
                    temp = "0";
                    string time1 = "0";
                    string dist = "0";
                    // cout << "Finished for " << std::get<0>(foo) << endl;

                    //if(fetchRequest(i, "FUNC") == "NEEDDist"){
                    temp = routedDIST( lat1, lon1, lat2, lon2, argv[ 1 ] );
                    dist = getNextToken( &temp, "^" );
                    time1 = getNextToken( &temp, "^" );
                    //}
                    //else{
                    //time1 = table(lat1,lon1,lat2,lon2,argv[1]);
                    //}



                    int indextable = ( int )to_number( fetchRequest( i, "TRIPIDX" ) );

                    //////find foo2<0> update distance with returned distance


                    //iterator_range<string::iterator> k = find_nth(temp, "^", 0);
                    //double distance2 = distance(k.begin()+1, temp.end());
                    //string substring = temp.substr(temp.length()-distance2, temp.length());
                    string substring = dist;



                    //cout << "nodes " << substringnodes << endl;

                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Distance " << substring << " for " << foo_str << endl;
                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Time " << time1 << " for " << foo_str << endl;

                    //duration = 1000.0 *( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                    //std::cout<<"time routed: "<< duration <<'\n';
                    messages = substring.c_str();

                    if( isSameLocation_onlyLatLong( lat1, lon1, lat2, lon2 ) )
                    {
                        substring = "0.00000";
                        time1 = "0";
                    }

                    updateRequest( i, substring, "DISTANCE" );
                    updateRequest( i, time1, "TIME" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 3':updateRequest( " << i << ", " << substring << ", 'DISTANCE' );" << endl;
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 3':updateRequest( " << i << ", " << time1 << ", 'TIME' );" << endl;

                    for ( int i = 0;
                          i < 4;
                          ++i )
                    {
                        latlon[ i ] = "empty";
                    }

                }



            }
            //***Case 3 - End***//


            //***Case 4 - Begin***//
            if( strcmp( shared_process_tab[ WRITINGFB ][ 0 ], "READY" ) == 0 && fetchRequest( i, "DISTANCE" ) == "" && fetchRequest( i, "EDGES" ) == "" && fetchRequest( i, "LATLONG" ) != "" && fetchRequest( i, "FUNC" ) == "GETOSRMFULLSTRINGACCESS" )
            {
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':shared_process_tab[ WRITINGFB ][ 0 ] == READY" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'DISTANCE' ) == ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'EDGES' ) == ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'LATLONG' ) != ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'FUNC' ) == GETOSRMFULLSTRINGACCESS" << endl;
                foundhit = true;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Getting full route" << endl;
                for( int j = 0;
                     j < MAXREQUESTS;
                     j++ )
                {
                    if( fetchRequest( j, "DISTANCE" ) == "" && fetchRequest( j, "LATLONG" ) != "" && fetchRequest( j, "FUNC" ) == "GETOSRMFULLSTRINGACCESS" && j < i && fetchRequest( j, "ID" ) == fetchRequest( i, "ID" ) )
                    {
                        //usleep(10);
                        i = j;
                        j = -1;


                    }
                }

                string foo_index;
                string foo_str;

                for( int j = i;
                     j < MAXREQUESTS;
                     j++ )
                {

                    if( fetchRequest( j, "DISTANCE" ) == "" && fetchRequest( j, "LATLONG" ) != "" && fetchRequest( j, "FUNC" ) == "GETOSRMFULLSTRINGACCESS" && fetchRequest( j, "ID" ) == fetchRequest( i, "ID" ) )
                    {

                        if( foo_str == "" )
                        {

                            foo_str = request_latlon[ j ];
                            foo_index = to_string( j );
                        }
                        else
                        {
                            foo_str = foo_str + "," + request_latlon[ j ];
                            foo_index = foo_index + "?" + to_string( j );
                        }

                    }
                }

                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Lat/Long " << foo_str << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << foo_index << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Case 4 - LATLONG: " << fetchRequest( i, "LATLONG" ) << "; FUNC: " << fetchRequest( i, "FUNC" ) << endl;



                // cout << "Trying for: " << std::get<0>(foo) << " "  << lon1 <<"," << lat1 << " 2: "<< lon2 <<"," << lat2 << endl;
                string temp = routedString( foo_str, argv[ 1 ] );
                // cout << "Finished for " << std::get<0>(foo) << endl;


                while( temp != "" )
                {

                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "temp " << i << " " << temp << endl;
                    int index = ( int )to_number( getNextToken( &foo_index, "?" ) );
                    string edges = getNextToken( &temp, "^" );
                    string distance = getNextToken( &temp, "?" );

                    if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "index " << index << " " << distance << " " << edges << endl;

                    if( edges.length() > MAXVYLGSTRSIZE )
                      edges = "ERROR";

                    //if(isSameLocation_onlyLatLong(lat1,lon1,lat2,lon2)){
                    //  distance = "0.0000";
                    // }
                    updateRequest( index, distance.c_str(), "DISTANCE" );
                    updateRequest( index, edges.c_str(), "EDGES" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 4':updateRequest( " << index << ", " << distance.c_str() << ", 'DISTANCE' );" << endl;
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 4':updateRequest( " << index << ", " << edges.c_str() << ", 'TIME' );" << endl;

                }

            }
            //***Case 4 - End***//

            //***Case 5 - Begin***//
            if( strcmp( shared_process_tab[ WRITINGFB ][ 0 ], "READY" ) == 0 && fetchRequest( i, "DISTANCE" ) == "" && fetchRequest( i, "LATLONG" ) != "" && fetchRequest( i, "FUNC" ) == "GETOSRMFULLSTRINGDISTANCES" )
            {
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':shared_process_tab[ WRITINGFB ][ 0 ] == READY" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'DISTANCE' ) == ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'LATLONG' ) != ''" << endl;
                if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed - confirm:'Case 2':fetchRequest( i, 'FUNC' ) == GETOSRMFULLSTRINGDISTANCES" << endl;
                for( int j = 0;
                     j < MAXREQUESTS;
                     j++ )
                {
                    if( fetchRequest( j, "DISTANCE" ) == "" && fetchRequest( j, "LATLONG" ) != "" && fetchRequest( j, "FUNC" ) == "GETOSRMFULLSTRINGDISTANCES" && j < i && fetchRequest( j, "ID" ) == fetchRequest( i, "ID" ) )
                    {
                        //usleep(10);
                        i = j;
                        j = -1;


                    }
                }

                string foo_index;
                string foo_str;

                for( int j = 0;
                     j < MAXREQUESTS;
                     j++ )
                {

                    if( fetchRequest( j, "DISTANCE" ) == "" && fetchRequest( j, "LATLONG" ) != "" && fetchRequest( j, "FUNC" ) == "GETOSRMFULLSTRINGDISTANCES" && fetchRequest( j, "ID" ) == fetchRequest( i, "ID" ) )
                    {

                        if( foo_str == "" )
                        {

                            foo_str = request_latlon[ j ];
                            foo_index = to_string( j );
                        }
                        else
                        {
                            foo_str = foo_str + "," + request_latlon[ j ];
                            foo_index = foo_index + "?" + to_string( j );
                        }

                    }
                }

                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Lat/Long " << foo_str << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "foo_index " << foo_index << endl;
                if(DEBUGROUTED == 1 || DEBUG == 1 ) gse << "Case 5 - LATLONG: " << fetchRequest( i, "LATLONG" ) << "; FUNC: " << fetchRequest( i, "FUNC" ) << endl;



                string temp = routedStringDistances( foo_str, argv[ 1 ] );
                // cout << "Finished for " << std::get<0>(foo) << endl;


                while( temp != "" && foo_index != "" )
                {
                    int index = ( int )to_number( getNextToken( &foo_index, "?" ) );
                    string distance = getNextToken( &temp, "?" );
                    // if(isSameLocation_onlyLatLong(lat1,lon1,lat2,lon2)){
                    //distance = "0.00000";

                    // }
                    updateRequest( index, distance.c_str(), "DISTANCE" );
                    if( DEBUGSTATE == 1 ) logstatechange << "osrm_routed:'main while loop':'main for loop':'Case 5':updateRequest( " << index << ", " << distance.c_str() << ", 'DISTANCE' );" << endl;
                    

                }

            }
            //***Case 5 - End***//



        }
        //***main for loop - End***//




        if( !foundhit ) //JDC Is this intended to reduce cpu time when there's nothing to do? Would normally be cycling through the While loop.
        	            //JDC But 0.1 seconds seems a bit long. Am reducing to 0.01.
        	//usleep( 100000 );    //microseconds
        	usleep( 10000 );
        else
        {
            foundhit = false;
            usleep( 100 );
        }


    }
    //***main while loop - End***//
}

catch ( const osrm::RuntimeError &e )
{
    util::Log( logERROR ) << e.what();
    return e.GetCode();
}
catch ( const std::bad_alloc &e )
{
    util::DumpMemoryStats();
    util::Log( logWARNING ) << "[exception] " << e.what();
    util::Log( logWARNING ) << "Please provide more memory or consider using a larger swapfile";
    return EXIT_FAILURE;
}
#ifdef _WIN32
    catch ( const std::exception &e )
    {
        util::Log( logERROR ) << "[exception] " << e.what();
        return EXIT_FAILURE;
    }
#endif
