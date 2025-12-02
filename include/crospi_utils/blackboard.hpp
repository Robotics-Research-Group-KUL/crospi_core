//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Erwin Aertbeliën
//  email: <erwin.aertbelien@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#pragma once

#include "jsoncpp/json/json.h"
// #include "rclcpp/rclcpp.hpp"
// #include <expressiongraph/solver_registry.hpp>
#include <stack>
#include "crospi_utils/string_interpolate.hpp"

namespace etasl {

/**
 * @brief loads a json file
 * @param fname std::string representing filename. string will be interpolated, i.e. you
 *  can use ${env var}, $(command) and $[rospackage]
 * @return 
 */
Json::Value loadJSONFile(const std::string& fname);

/**
 * @brief saves json data structure to a file
 * @param data Json::Value data structure containing json.
 * @param fn  std::string representing filename. string will be interpolated, i.e. you
 *  can use ${env var}, $(command) and $[rospackage]
 */
void saveJSONFILE(const Json::Value& data, const std::string& fn);


/**
 * @brief adds elements of additional if they don't exists in original.  For arrays the items
 * are appended, For objects, the non existing objects are added and the existing objects
 * are recursively processed.
 * @param original Json::Value JSON data structure
 * @param additional Json::Value JSON data structure
 * @return 
 */
void addIfNotExists(Json::Value& original, const Json::Value& additional);


/**
 * @brief get the node corresponding to the given path
 * @param root Json::Value root of the json document.
 * @param path Path the a value (separated with '.')
 * @return reference to the node specified by the path. This routine will create missing nodes
 *         such that you can assign to the result.
 */
Json::Value& getPath(Json::Value& root, const std::string& path);

/**
 * @brief gets a pointer to the node of the given path or null if it does not exist.
 * @param root Json::Value root of the json document
 * @param path Path the the given value (separated with '.')
 * @return pointer to the node corresponding to the path or null if not exists.
 */
Json::Value* getPathValue(Json::Value& root, const std::string& path);

/**
 * The Blackboard contains all schema definitions, global parameters and parameters
 * you want to transfer.
 *
 * The actual parameters of eTaSL routines are outside the blackboard in a Json::Value
 * object. But you can make references to the blackboard in this structure.
 *
 * All data is maintained in a JSON object: 
 *  \code{JSON}
 *      {
 *          "blackboard" : {}, 
 *          "$defs": { 
 *              "my-schema-id": {}
 *          } 
 *      }
 *  \endcode
 * 
 * Implements a subset of **JSON-SCHEMA** (see https://json-schema.org/ ) and a number of extensions.
 * Extensions are "default" tag to indicate default value, "interpolate" to perform string interpolation
 * on a string type (i.e. ${..} for environment variables, $(...) for shell execution and $[...] for ROS2/ament package lookup)
 *
 * you refer to an element in the *blackboard* using a JSON object with only a $ref member, e.g. {"$ref":"/solver/initial/convergence_rate"}
 * for an absolute reference and {"$ref":"initial/convergence_rate"} for a relative reference.
 *
 * 
 * $defs member in the data will overwrite the definition loaded beforehand in the blackboards.  The blackboard remembers $defs encountered in
 * previous data.
 */
class BlackBoard {
    Json::Value base; // root of JSON document
    Json::Value empty; // an empty value to return as a reference in some cases.
    Json::Value* root; // root of the blackboard ( everything under the "blackboard" key)
    std::vector<Json::Value*> pth;
    Json::Value* defs; // schema definitions
    SearchPath searchpath;
    int path_components_used;

public:
    /**
     * @brief constructor
     * @param path_components_used each reference is only identified by the last 'path_components_used'
     * parts of the reference.
     */
    BlackBoard(int path_components_used);

    /**
     * @brief addSchema
     * @param filename filename of the schema to add.
     * @details string interpolation will be applied to the filename.  The schema will be stored
     * with its "$id" tag as key.
     */
    void addSchema(const std::string& filename);

    /**
     * @brief addSchema
     * @param schema schema to add
     * @details  The schema will be stored with its "$id" tag as key.
     */
    void addSchema(const Json::Value& schema, const std::string ctx = "");

    /**
     * @brief add a schema for each .json file in the given directory.
     * @param dirname string (interpolated) indicating the directory name
     */
    void addSchemaFromDir(const std::string& dirname);

    /**
     * @brief Set the search path.
     * @param spth std::string representing the search path. Will have string interpolation,i.e.
     *   ${env. var}, $(command), $[rospackage] are recognized.
     */
    void setSearchPath(const std::string& spth);

    /**
     * @brief load Schema Reference. Resolves the reference to a local $id. First is checked whether already a schema was loaded and if not,
     * the reference is looked up using the searchpath and loaded. An error is thrown if unsuccesful.
     * @param rawref
     * @param ctx
     * @param schema Json::Value if not empty, this indicate the schema that should be obeyed (even if rawref
     *           points to another schema).  If empty, use the schema as indicated by rawref.
     * @return
     */
    Json::Value loadSchemaReference(const std::string& raw_ref, const std::string& ctx, const Json::Value schema);

    /**
     * @brief resolves a reference to a schema and returns a local $id. Only the 'path_components_used' last parts of the
     * reference are used.
     * @param raw_ref std::string reference to a schema
     * @return Json::Value representing the schema that was refered to.
     */
    std::filesystem::path resolveSchemaReference(const std::string& raw_ref);

    /**
     * @brief load from a file
     * @param filename json file to load
     * @details should have a blackboard and $defs key.  string interpolation will be
     *          used for filename.
     */
    void load(const std::string& filename);


    /**
     * @brief load from a file and processes and validates it
     * @param filename json file to load
     * @details should have a blackboard and $defs key.  string interpolation will be
     *          used for filename.
     */
    void load_process_and_validate(const std::string& filename);

    /**
     * @brief save to a file
     * @param filename filename of the file to save
     * @details will have a blackboard and $defs key.  string interpolation will be
     *          used for filename.
     */
    void save(const std::string& filename);

    /**
     * @brief saves all the schema's to a file, (including the "$defs" key)
     * @param filename file name, string interpolation will be used for the filename.
     */
    void saveSchemas(const std::string& filename);

    /**
     * @brief gets the root JSON object of the blackboard
     * @return Json::Value& reference to the root, i.e. the object that is referred to with the "blackboard" key.
     */
    Json::Value& getRoot();

    /**
     * @brief gets the local JSON node of the blackboard
     * @return Json::Value& reference to the local node of the blackboard
     * @details equivalent to local directory of a file system
     */
    Json::Value& getLocal();

    /**
     * @brief gets schema definitions that are loaded or gather during process_and_validate
     * @return Json::Value containing a schema with only a '$defs' member.
     */
    Json::Value getDefinitions();

    /**
     * @brief move the local node to a subnode
     * @param property name of the subnode
     * @details similar to 'cd subdir' in a file system
     */
    void changeLocal(const std::string& property);

    /**
     * @brief moves the local node up.
     * @details similar to 'cd ..' in a file system
     */
    void changeLocalUp();

    // /**
    //  * @brief moves the local node to an absolute path
    //  * @param path path
    //  * @details similar to 'cd /a/b/c' in a file system
    //  */
    // void moveLocalPath(const std::string& path);

    /**
     * @brief get the node corresponding to the path
     * @param path string indicating the path to the node (absolute or relative) (notation similar to linux filesystem)
     * @param create bool create the node if it does not exist.
     * @return a JSON node that is null if create was false and path not found.  If create=true and not found,
     * an empty object is created in the tree and returned.
     */
    Json::Value& getPath(const std::string& path, bool create);

    /**
     * @brief processes and validates the data given a specific json-schema.
     * @param schema JSON schema
     * @param data JSON data structure to validate and process
     * @param ctx ontext that can be given to be used in error messages
     * @return adapted data structure that is validated against the schema, with default
     * values filled in, strings interpolated and blackboard references resolved.
     */
    Json::Value process_and_validate(Json::Value& schema,
        Json::Value& data, const std::string& ctx = "");

    /**
     * @brief processes and validates the data given a specific json-schema.
     * @param schema_id string, id of the Json Schema
     * @param datafn file name of the json file to be validated (interpolated string)
     * @return adapted data structure that is validated against the schema, with default
     * values filled in, strings interpolated and blackboard references resolved.
     */
    Json::Value process_and_validate(const std::string& schema_id, const std::string& datafn);
    /**
     * @brief processes and validates the data given a specific json-schema.
     * @param schema_id string, id of the Json Schema
     * @param data JSON object describing data
     * @return adapted data structure that is validated against the schema, with default
     * values filled in, strings interpolated and blackboard references resolved.
     **/
    Json::Value process_and_validate(const std::string& schema_id, Json::Value& data, const std::string& ctx = "");

    /**
     * @brief processes and validates the data given a json-schema specified in the data itself using
     *        the $schema tag.
     * @param data JSON data structure to validate and process
     * @param ctx ontext that can be given to be used in error messages
     * @return adapted data structure that is validated against the schema, with default
     * values filled in, strings interpolated and blackboard references resolved.
     */
    Json::Value process_and_validate(
        Json::Value& data, const std::string& ctx = "");

    /**
     * @brief processes and validates the data given a json-schema specified in the data itself using
     *        the $schema tag.
     * @param datafn ile name of the json file to be validated (interpolated string)
     * @return adapted data structure that is validated against the schema, with default
     * values filled in, strings interpolated and blackboard references resolved.
     */
    Json::Value process_and_validate(const std::string& datafn);
};

} // namespace etasl
