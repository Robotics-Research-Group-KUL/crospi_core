#include "etasl_task_utils/blackboard.hpp"
#include <etasl_task_utils/etasl_error.hpp>
#include <etasl_task_utils/string_interpolate.hpp>
#include <filesystem>
#include <fmt/format.h>
#include <regex>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

namespace etasl {

Json::Value loadJSONFile(const std::string& fname)
{
    std::ifstream ifs(etasl::string_interpolate(fname));
    Json::Reader reader;
    Json::Value root;
    reader.parse(ifs, root);
    return root;
}

void saveJSONFILE(const Json::Value& data, const std::string& fn)
{
    Json::StyledWriter styled;
    std::ofstream of(etasl::string_interpolate(fn));
    of << styled.write(data);
    of.close();
}

Json::Value& getPath(Json::Value& root, const std::string& path)
{
    size_t i = 0;
    size_t prev = 0;
    Json::Value* p = &root;
    while (i < path.size()) {
        if (path[i] == '.') {
            auto name = path.substr(prev, i - prev);
            if (!p->isMember(name)) {
                (*p)[name] = Json::Value(Json::objectValue);
            }
            p = &((*p)[name]);
            prev = i + 1;
        }
        i++;
    }
    auto name = path.substr(prev);
    if (!p->isMember(name)) {
        (*p)[name] = Json::Value(Json::objectValue);
    }
    return (*p)[name];
}

Json::Value* getPathValue(Json::Value& root, const std::string& path)
{
    size_t i = 0;
    size_t prev = 0;
    Json::Value* p = &root;
    while (i < path.size()) {
        if (path[i] == '.') {
            auto name = path.substr(prev, i - prev);
            if (!p->isMember(name)) {
                return nullptr;
            }
            p = &((*p)[name]);
            prev = i + 1;
        }
        i++;
    }
    auto name = path.substr(prev);
    if (!p->isMember(name)) {
        return nullptr;
    }
    p = &((*p)[name]);
    return p;
}

// original and additional are always object or array
void addIfNotExists(Json::Value& original, const Json::Value& additional)
{
    if (original.isArray() and additional.isArray()) {
        // always added, never fuse
        for (auto a : additional) {
            original.append(a);
        }
    } else if (original.isObject() and additional.isObject()) {
        // fuse
        for (auto name : additional.getMemberNames()) {
            if (original.isMember(name)) {
                // fuse
                addIfNotExists(original[name], additional[name]);
            } else {
                // add
                original[name] = additional[name];
            }
        }
    } else {
        if (original.empty()) {
            original = additional;
        }
        // do nothing
    }
}

BlackBoard::BlackBoard(int _path_components_used)
    : base(Json::objectValue)
{
    base["blackboard"] = Json::Value(Json::objectValue);
    base["$defs"] = Json::Value(Json::objectValue);
    root = &(base["blackboard"]);
    pth.reserve(20);
    pth.push_back(root);
    defs = &(base["$defs"]);
    path_components_used = _path_components_used;
}

void BlackBoard::addSchema(const Json::Value& schema, const std::string ctx)
{
    if (!schema["$id"]) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("{} : schema does not have an $id member", ctx));
    }
    if (!schema["$id"].isString()) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("schema in file '{}' should have an $id member with a string value", ctx));
    }
    std::string id = keepLastNPartsOfPath(schema["$id"].asString(), path_components_used);
    (*defs)[id] = schema;
}

void BlackBoard::addSchema(const std::string& fname)
{
    Json::Value schema = loadJSONFile(fname);
    addSchema(schema, fname);
}

void BlackBoard::addSchemaFromDir(const std::string& dirname)
{
    namespace fs = std::filesystem;
    std::string dn = etasl::string_interpolate(dirname);
    for (auto& p : fs::directory_iterator(dn)) {
        Json::Value schema = loadJSONFile(p.path());
        addSchema(schema, p.path());
    }
}

void BlackBoard::setSearchPath(const std::string& spth)
{
    searchpath = SearchPath(spth);
}

Json::Value BlackBoard::loadSchemaReference(const std::string& raw_ref, const std::string& ctx, const Json::Value schema = Json::Value())
{
    auto p = resolveSchemaReference(raw_ref);
    if (schema) {
        auto schema_id = resolveSchemaReference(schema["$id"].asString());
        if (schema_id != p) {
            throw etasl_error(etasl_error::SCHEMA_REFERENCE_NOT_FOUND,
                "{}: request to load schema with $id='{}', but expected schema with $id='{}'",
                ctx, p.c_str(), schema_id.c_str());
        }
    }
    if (defs->isMember(p.c_str())) {
        return (*defs)[p.c_str()];
    }
    auto fp = searchpath.find(p);
    if (fp.empty()) {
        throw etasl_error(etasl_error::SCHEMA_REFERENCE_NOT_FOUND,
            "{}: Could not resolve the reference '{}' using '{}' parts of the path, using the searchpath '{}'",
            ctx, raw_ref, path_components_used, searchpath);
    }
    std::cout << "loading reference " << fp.c_str() << std::endl;
    Json::Value newschema = loadJSONFile(fp.c_str());
    auto new_id = resolveSchemaReference(newschema["$id"].asString());
    if (new_id != p) {
        throw etasl_error(etasl_error::SCHEMA_REFERENCE_NOT_FOUND,
            "{}: raw reference '{}' leading to local reference $id='{}' , I found file '{}' but the file contains another reference with local $id='{}'.",
            ctx, raw_ref, p.c_str(), fp.c_str(), new_id.c_str());
    }
    return newschema;
}

std::filesystem::path BlackBoard::resolveSchemaReference(const std::string& raw_ref)
{
    // std::cout << "raw reference " << raw_ref << std::endl;
    URI uri(raw_ref);
    std::filesystem::path extract_path;
    if (uri.matched()) {
        extract_path = uri.path();
    } else {
        extract_path = raw_ref;
    }
    // std::cout << "part corresponding to path of reference " << extract_path << std::endl;
    return keepLastNPartsOfPath(extract_path, path_components_used);
}

void BlackBoard::load(const std::string& filename)
{
    std::string fn = etasl::string_interpolate(filename);
    Json::Value result = loadJSONFile(fn);
    if (!result.isObject()) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("{} should describe JSON object", fn));
    }
    if (!result.isMember("blackboard") || !result["blackboard"].isObject()) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("{} should contain a member object with key 'blackboard'", fn));
    }
    if (!result.isMember("$defs") || !result["$defs"].isObject()) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("{} should contain a member object with key '$defs'", fn));
    }
    base = std::move(result);
    root = &(base["blackboard"]);
    pth.clear();
    pth.push_back(root);
    defs = &(base["$defs"]);
}

void BlackBoard::load_process_and_validate(const std::string& filename)
{
    load(filename);
    base = process_and_validate(base, "Blackboard loaded from " + filename);
    root = &(base["blackboard"]);
    pth.clear();
    pth.push_back(root);
    defs = &(base["$defs"]);
}

void BlackBoard::save(const std::string& filename)
{
    std::string fn = etasl::string_interpolate(filename);
    saveJSONFILE(base, fn);
}

void BlackBoard::saveSchemas(const std::string& filename)
{
    std::string fn = etasl::string_interpolate(filename);
    saveJSONFILE(getDefinitions(), fn);
}

Json::Value& BlackBoard::getRoot()
{
    return *root;
}

Json::Value& BlackBoard::getLocal()
{
    return *pth.back();
}

Json::Value BlackBoard::getDefinitions()
{
    Json::Value result(Json::objectValue);
    result["$defs"] = *defs;
    return std::move(result);
}

void BlackBoard::changeLocal(const std::string& property)
{
    if (!pth.back()->isMember(property)) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("{} is not a member of local node", property));
    }
    if (!(*pth.back())[property].isObject()) {
        throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("{} subnode is not an object you can change local to ", property));
    }
    pth.push_back(&(*pth.back())[property]);
}

void BlackBoard::changeLocalUp()
{
    if (pth.size() > 1) {
        pth.pop_back();
    }
}

Json::Value& BlackBoard::getPath(const std::string& path, bool create)
{
    size_t i = 0;
    size_t prev = 0;
    std::vector<Json::Value*> localpth;
    if (path[i] == '/') {
        localpth.push_back(root);
        prev = 1;
        i = 1;
    } else {
        localpth = pth;
    }
    while (i <= path.size()) {
        // std::cout << "i " << i << " value " << path[i] << std::endl;
        if (i == path.size() || path[i] == '/') {
            std::string name = path.substr(prev, i - prev);
            // std::cout << "name : " << name << std::endl;
            if (name == "..") {
                if (localpth.size() == 1) {
                    throw etasl_error(etasl_error::JSON_PARSE_ERROR, fmt::format("'..' cannot go up in path '{}' ", path));
                }
                localpth.pop_back();
            } else {
                if (localpth.back()->isMember(name)) {
                    localpth.push_back(&(*localpth.back())[name]);
                } else {
                    if (create) {
                        Json::Value obj(Json::objectValue);
                        (*localpth.back())[name] = obj;
                        localpth.push_back(&(*localpth.back())[name]);
                    } else {
                        return empty;
                    }
                }
            }
            prev = i + 1;
        }
        i++;
    }
    return (*localpth.back());
}

Json::Value BlackBoard::process_and_validate(Json::Value& schema,
    Json::Value& data, const std::string& ctx)
{
    if (data.isObject()) {
        // if data is a reference to the blackboard, resolve it:s
        if (data.isMember("$ref")) {
            /// @todo implement

            Json::Value r = data["$ref"];
            std::cout << "data : value of $ref " << r.asString() << std::endl;
            if (!r.isString()) {
                throw std::logic_error(fmt::format("{}: reference to a blackboard using '$ref' should be of string type ", ctx));
            }
            Json::Value p = getPath(r.asString(), false);
            std::cout << "whole path of /default : " << getPath("/default", false) << std::endl;
            std::cout << "$ref points to " << p << std::endl;
            //Json::Path pth(".default.frequency");
            //Json::Value obj = pth.resolve(*root);
            //std::cout << "path using jsoncpp " << obj << std::endl;
            // std::cout << "root : " << *root << std::endl;
            // std::cout << "resolved data reference " << p << std::endl;
            if (!p) {
                throw etasl_error(etasl_error::JSON_PARSE_ERROR, "could not find blackboard reference '{}'", r.asString());
            }
            std::cout << "schema " << schema << std::endl;
            Json::Value retval = process_and_validate(schema, p, ctx + "." + "default");
            std::cout << "after validation " << retval << std::endl;
            return retval;
        }
        if (data.isMember("$schema")) {
            // override existing schema if possible:
            schema = loadSchemaReference(data["$schema"].asString(), ctx, schema);
        }
    }
    if (!schema.isObject()) {
        schema = Json::Value(Json::objectValue);
        return data;
        // throw std::logic_error(fmt::format("{} : schema should not be zero and should be an JSON object", ctx));
    }
    if (schema.isMember("$defs")) {
        // if schema contains $def declaration, add it to the defs :
        if (!schema["$defs"].isObject()) {
            throw std::logic_error(fmt::format("{}: type of '$defs' member in schema should be JSON object ", ctx));
        }
        for (auto id : schema["$defs"].getMemberNames()) {
            (*defs)[id] = schema["$defs"][id];
        }
    }
    // and expand $ref in schema :
    if (schema.isMember("$ref")) {
        Json::Value ref = schema["$ref"];
        if (!ref.isString()) {
            throw std::logic_error(fmt::format("{} : $ref should be a string", ctx));
        }
        schema = loadSchemaReference(ref.asString(), ctx + " schema reference ");
        return process_and_validate(schema, data, ctx);
    }
    // expand $ref in data using the blackboard
    // if (data.isMember("$ref"))

    if (schema.isMember("oneOf")) {
        Json::Value& oneof = schema["oneOf"];
        if (!oneof.isArray()) {
            throw std::logic_error(fmt::format("{} : oneOf key should point to an array of possiblly matching schema", ctx));
        }
        std::stringstream ss;
        ss << "data:\n"
           << data <<"\n";
        for (auto p : oneof) {
            Json::Value localdata = data;
            try {
                localdata = process_and_validate(p, localdata, ctx);
                return localdata;
            } catch (std::exception& e) {
                ss << "  schema:\n"
                   << p << "\nwas rejected because of " << e.what() << std::endl;
            }
        }
        throw std::logic_error(fmt::format("{} : none of the schema's indicatd by oneOf match the data :\n", ctx) + ss.str());
    }

    Json::Value typev = schema["type"];
    if (!typev || !typev.isString()) {
        std::cout << schema << std::endl;
        throw std::logic_error(fmt::format("{} : schema should have a member 'type' that is a string", ctx));
    }
    std::string type = typev.asString();
    Json::Value retval;
    if (type == "object") {
        // check if required properties are present:
        Json::Value required = schema["required"];
        if (required) {
            if (!required.isArray()) {
                throw std::logic_error(fmt::format("{} : 'required' member of schema should be an array of strings", ctx));
            }
            for (auto s : required) {
                if (!s.isString()) {
                    throw std::logic_error(fmt::format("{} : 'required' member of schema should be an array of strings", ctx));
                }
                if (!data[s.asString()]) {
                    throw std::logic_error(fmt::format("{} : required member of schema '{}' is missing", ctx, s.asString()));
                }
            }
        }
        // define some aux. variables:
        Json::Value properties = schema["properties"];
        Json::Value addprop = schema["additionalProperties"];
        retval = Json::Value(Json::objectValue);
        // schema should have a properties member:
        if (!properties) {
            //throw std::logic_error(fmt::format("{} : schema of type object should have a properties member", ctx));
            properties = Json::Value(Json::objectValue);
        }
        // validate all data properties :
        for (auto n : data.getMemberNames()) {
            if (properties.isMember(n)) {
                retval[n] = process_and_validate(properties[n], data[n], ctx + "." + n);
            } else {
                if (!addprop || (addprop.isBool() && !addprop.asBool())) {
                    if (n != "$schema") {
                        throw std::logic_error(fmt::format("{} : member '{}' encountered but schema does not allow additional properties ", ctx, n));
                    }
                } else {
                    retval[n] = process_and_validate(properties[n], data[n], ctx + "." + n);
                }
            }
        }
        // fill in default values if they are available:
        for (auto n : properties.getMemberNames()) {
            Json::Value member = properties[n];
            if (member.isMember("default") && !retval.isMember(n)) {
                // fmt::print("default encountered for property {} with schema {} and value {}", n, member, member["default"]);
                retval[n] = process_and_validate(member, member["default"], ctx + "." + n);
            }
            // if (member.isMember("default")) {
            //      //std::cout << "retval[n] before " << retval[n] << std::endl;
            //      //std::cout << "schema " << schema << std::endl;
            //     //std::cout << "default " << member["default"] << std::endl;
            //      addIfNotExists(retval[n], member["default"]);
            //      //std::cout << "retval[n] " << retval[n] << std::endl;
            //      // fmt::print("default encountered for property {} with schema {} and value {}", n, member, member["default"]);
            //      retval[n] = process_and_validate(member, retval[n], ctx + "." + n);
            //  }
        }
        // check number of properties:
        if (schema.isMember("maxProperties")) {
            Json::Value maxprop = schema["maxProperties"];
            if (!maxprop.isNumeric()) {
                throw std::logic_error(fmt::format("{} : maxProperties property in schema should be numeric", ctx));
            }
            if (data.size() > maxprop.asInt()) {
                throw std::logic_error(fmt::format("{} : number of properties ({}) is larger than maxProperties ({}) ",
                    ctx, data.size(), maxprop.asInt()));
            }
        }
        if (schema.isMember("minPropeties")) {
            Json::Value minprop = schema["minProperties"];
            if (!minprop.isNumeric()) {
                throw std::logic_error(fmt::format("{} : minProperties property in schema should be numeric", ctx));
            }
            if (data.size() < minprop.asInt()) {
                throw std::logic_error(fmt::format("{} : number of properties ({}) is larger than maxProperties ({}) ",
                    ctx, data.size(), minprop.asInt()));
            }
        }
    } else if (type == "string") {
        if (!data.isString()) {
            throw std::logic_error(fmt::format("{} : should be a string", ctx));
        }
        Json::Value interp = schema["interpolate"];
        retval = Json::Value(data.asString());
        if (interp) {
            if (!interp.isBool()) {
                throw std::logic_error(fmt::format("{} : interpolate property of schema should be boolean", ctx));
            }
            if (interp.asBool()) {
                retval = Json::Value(etasl::string_interpolate(data.asString()));
            }
        }
    } else if (type == "number") {
        if (!data.isDouble()) {
            throw std::logic_error(fmt::format("{} : should be a number", ctx));
        }

        if (schema.isMember("minimum")) {
            Json::Value minv = schema["minimum"];
            if (!minv.isNumeric()) {
                throw std::logic_error(fmt::format("{} : minimum property in schema should be numeric", ctx));
            }
            if (data.asDouble() < minv.asDouble()) {
                throw std::logic_error(fmt::format("{} : number shuould be larger than minimum ({})", ctx, minv.asDouble()));
            }
        }

        if (schema.isMember("maximum")) {
            Json::Value maxv = schema["maximum"];
            if (!maxv.isNumeric()) {
                throw std::logic_error(fmt::format("{} : maximum property in schema should be numeric", ctx));
            }
            if (data.asDouble() > maxv.asDouble()) {
                throw std::logic_error(fmt::format("{} : number shuould be smaller than maximum ({})", ctx, maxv.asDouble()));
            }
        }
        retval = data;
    } else if (type == "boolean") {
        if (!data.isBool()) {
            throw std::logic_error(fmt::format("{} : should be a boolean", ctx));
        }
        retval = data;
    } else if (type == "array") {
        // check type of data:
        if (!data.isArray()) {
            throw std::logic_error(fmt::format("{} : should be an array", ctx));
        }
        // check number of array elements:
        if (schema.isMember("minItems")) {
            Json::Value minv = schema["minItems"];
            if (!minv.isNumeric()) {
                throw std::logic_error(fmt::format("{} : minItems property of schema should be numeric", ctx));
            }
            if (data.size() < minv.asInt()) {
                throw std::logic_error(fmt::format("{} : array should have at least {} items", ctx, minv.asInt()));
            }
        }
        if (schema.isMember("maxItems")) {
            Json::Value maxv = schema["maxItems"];
            if (!maxv.isNumeric()) {
                throw std::logic_error(fmt::format("{} : maxItems property of schema should be numeric", ctx));
            }
            if (data.size() > maxv.asInt()) {
                throw std::logic_error(fmt::format("{} : array should have at most {} items", ctx, maxv.asInt()));
            }
        }
        // check type of array elements:
        int size = 0;
        retval = Json::Value(Json::arrayValue);
        for (auto val : data) {
            if (schema.isMember("items")) {
                retval[size] = process_and_validate(schema["items"], val, ctx + "[" + std::to_string(size) + "]");
            } else {
                retval[size] = val;
            }
            size++;
        }
    }
    return retval;
}

Json::Value BlackBoard::process_and_validate(const std::string& schema_id, const std::string& datafn)
{
    Json::Value schema;
    if (schema_id != "") {
        schema = Json::Value(Json::objectValue);
        schema["$ref"] = schema_id;
    }
    Json::Value data = loadJSONFile(etasl::string_interpolate(datafn));
    return process_and_validate(schema, data, datafn);
}

Json::Value BlackBoard::process_and_validate(const std::string& schema_id, Json::Value& data, const std::string& ctx)
{
    Json::Value schema;
    if (schema_id != "") {
        schema = Json::Value(Json::objectValue);
        schema["$ref"] = schema_id;
    }
    return process_and_validate(schema, data, ctx);
}

Json::Value BlackBoard::process_and_validate(
    Json::Value& data, const std::string& ctx)
{
    Json::Value empty;
    return process_and_validate(empty, data, ctx);
}

Json::Value BlackBoard::process_and_validate(const std::string& datafn)
{
    Json::Value empty;
    Json::Value data = loadJSONFile(etasl::string_interpolate(datafn));
    return process_and_validate(empty, data, datafn);
}

} // namespace etasl