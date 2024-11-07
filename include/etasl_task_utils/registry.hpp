#pragma once

#include "etasl_task_utils/etasl_error.hpp"
#include "jsoncpp/json/json.h"
#include <fmt/format.h>
#include <memory>
#include <unordered_map>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include "etasl_task_utils/json_checker.hpp"


namespace etasl {


/**
 * A registry stores a number of factories under a name.
 * 
 * The name under which the factory class is stored corresponds to the name returned by the
 * getName() method of the factory class.
 * 
 * The template parameter should correspond to the BASE CLASS, not to the implementations.  The base
 * class is typically an abstract class.
 * 
 * 
*/
template <typename Factory>
class Registry {

    std::unordered_map<std::string, typename Factory::SharedPtr> m;

protected:
    Registry() { }
    static Registry<Factory>& instance()
    {
        // it **is** thread-safe in C++11.
        static Registry<Factory> myInstance;
        return myInstance;
    }

public:
    Registry<Factory>& operator=(Registry<Factory> const&) = delete; // Copy assign
    Registry<Factory>& operator=(Registry<Factory>&&) = delete; // Move assign

    /**
     * @brief registers a factory  under the name given by factory.getName()
     * @param factory instance of type Factory.
     */
    static void registerFactory(typename Factory::SharedPtr factory)
    {
        Registry<Factory>& reg = Registry<Factory>::instance();
        reg.m[factory->getName()] = factory;
    }

    /**
     * @brief gets the schema of the factory corresponding to the given name
     * @param name std::string name of requested factory
     * @return JSON object
     */
    static Json::Value getSchema(const std::string& name)
    {
        Registry<Factory>& reg = Registry<Factory>::instance();
        auto r = reg.m.find(name);
        if (r == reg.m.end()) {
            throw etasl_error(etasl_error::NOT_REGISTERED, fmt::format("name '{}' is not registered", name));
        }
    }

    /**
     * @brief create solver factory with given name and parameters
     * @param parameters Json::Value parameters. One of the parameters has key "is-<name>" where
     * name corresponds to a name stored in the registry.  The registry uses this to ask the appropriate 
     * factory class to create an instance of the underlying object.
     * @param jsonchecker Pointer to json checker class which checks and returns json parameters, and handles errors
     * @return
     */
    static typename Factory::ProductSharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        Registry<Factory>& reg = Registry<Factory>::instance();
        for (auto p : parameters.getMemberNames()) {
            if (p.substr(0, 3) == "is-") {
                std::string name = p.substr(3);
                auto r = reg.m.find(name);
                if (r == reg.m.end()) {
                    std::stringstream ss;
                    ss << "Registered names are \n";
                    for (auto a:reg.m) {
                        ss << "\t" << a.first << std::endl;
                    }
                    throw etasl_error(etasl_error::NOT_REGISTERED, fmt::format("name '{}' is not registered\n{}", name,ss.str()));
                }
                return r->second->create(parameters, jsonchecker);
            }
        }
        throw etasl_error(etasl_error::NOT_REGISTERED, "do not know what to create because a key starting with 'is-' is missing");
    }
};

} // etasl