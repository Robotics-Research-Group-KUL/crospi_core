#include "etasl_task_utils/outputhandlerfactory.hpp"
#include "etasl_task_utils/fileoutputhandler.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/registry.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create TopicOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class FileOutputHandlerFactory : public OutputHandlerFactory {

public:
    typedef std::shared_ptr<FileOutputHandlerFactory> SharedPtr;

    FileOutputHandlerFactory()
    {
    }

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$id": "fileoutputhandler.json",
                        "type" : "object",
                        "title":"Parameters for FileOutputHandler",
                        "description" : "A FileOutputHandler stores the data in memory and writes it to the given file when finalizing",
                        "properties" : {
                            "is-fileoutputhandler" : {
                                "type":"boolean"
                            },                            
                            "header" : {
                                "type":"boolean"
                            },
                            "maxlines": {
                                "type": "number",
                                "default": 100000
                            }
                            "variable-names": {
                                "type":"array",
                                "items": {
                                    "type":"string"
                                }
                            }
                        },
                        "required": ["is-topicoutputhandler","topic-name","variable-names"],
                        "additionalProperties" : false
                    }
                )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName()
    {
        return "fileoutputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual FileOutputHandler::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::string filename = jsonchecker->asString(parameters, "filename");
        bool header = jsonchecker->asBool(parameters, "header");
        size_t maxlines = jsonchecker->asUInt64(parameters, "maxlines");
        std::vector<std::string> varnames;
        for (auto n : jsonchecker->asArray(parameters, "variable-names")) {
            varnames.push_back(jsonchecker->asString(n, ""));
        }
        return std::make_shared<FileOutputHandler>(
            filename,
            header,
            maxlines,
            varnames);
    }

    virtual ~FileOutputHandlerFactory() { }
};

void registerFileOutputHandlerFactory()
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<OutputHandlerFactory>::registerFactory(std::make_shared<FileOutputHandlerFactory>());
}

} // namespace