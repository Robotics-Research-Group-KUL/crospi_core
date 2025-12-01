#include <crospi_utils/etasl_error.hpp>
#include <crospi_utils/solverregistry.hpp>
#include <fmt/format.h>
namespace etasl {

SolverRegistry& SolverRegistry::instance()
{
    // it **is** thread-safe in C++11.
    static SolverRegistry myInstance;
    return myInstance;
}

/**
 * @brief register a factory
 * @param factory
 */
void SolverRegistry::registerFactory(SolverFactory::SharedPtr factory)
{
    m[factory->getName()] = factory;
}

KDL::solver::Ptr SolverRegistry::create(const std::string& name, const Json::Value& parameters)
{
    auto r = m.find(name);
    if (r == m.end()) {
        throw etasl_error(etasl_error::UNKNOWN_SOLVER_REQUESTED, fmt::format("Solver with name '{}' is not registered", name));
    }
    return r->second->create(parameters);
}

KDL::solver::Ptr createSolver(const std::string& name, const Json::Value& parameters)
{
    return SolverRegistry::instance().create(name, parameters);
}

}; // namespace etasl