// Some ament utility functions for lua
//
//#include "luabind/luabind.hpp"
//s#include <luabind/class_info.hpp>

#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_packages_with_prefixes.hpp"
#include <fmt/format.h>

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <mutex>
#include <stdexcept>

/**
 * Singleton classes that caches the ament package index and does not need
 * to access the filesystem to get a package prefix
 * 
 * more or less a drop-in-replacement for the routines of ament_index_cpp that access
 * directory each time.
 * 
 * (This is good and thread-safe implementation, guaranteed from C++11 )
 */
class AmentIndex
{    
    private:
        std::map<std::string,std::string> package_map;

        AmentIndex() {  
          package_map = ament_index_cpp::get_packages_with_prefixes();
          // for (auto it = package_map.begin(); it!= package_map.end(); it++) {
          //   std::cout << it->first << ":" << it->second << std::endl;
          // }
        } 

    public:
        /// @brief creates a single instance of the AmentIndex (singleton). This loads the dictionary of all packages and their prefix.
        /// @return a reference to the AmentIndex object.
        static AmentIndex& getInstance()
        {
            static AmentIndex  instance;
            return instance;
        }    
        AmentIndex(AmentIndex const&)               = delete;  // singleton, avoid creating multiple instances
        void operator=(AmentIndex const&)           = delete;  // singleton, avoid creating multiple instances

        /// @brief gets the prefix of package
        /// @param pkg string with package name
        /// @return string with prefix
        std::string get_package_prefix(const std::string& pkg) {
            auto loc = package_map.find(pkg);
            if ( loc!=package_map.end() ) {
                return loc->second;
            } else {
                throw ament_index_cpp::PackageNotFoundError(pkg);
            }
        }


        /// @brief gets the share directory of a package
        /// @param pkg string with package name
        /// @return string with the share directory of the package
        std::string get_package_share_directory(const std::string& pkg) {
          return get_package_prefix(pkg) + "/share/" + pkg;
        }
};

// debugging routine:
// static void stackDump (lua_State *L) {
//       int i;
//       int top = lua_gettop(L);
//       for (i = 1; i <= top; i++) {  /* repeat for each level */
//         int t = lua_type(L, i);
//         switch (t) {
    
//           case LUA_TSTRING:  /* strings */
//             printf("`%s'", lua_tostring(L, i));
//             break;
    
//           case LUA_TBOOLEAN:  /* booleans */
//             printf(lua_toboolean(L, i) ? "true" : "false");
//             break;
    
//           case LUA_TNUMBER:  /* numbers */
//             printf("%g", lua_tonumber(L, i));
//             break;
    
//           default:  /* other values */
//             printf("%s", lua_typename(L, t));
//             break;
    
//         }
//         printf("  ");  /* put a separator */
//       }
//       printf("\n");  /* end the listing */
//     }




static int l_get_package_share_directory (lua_State *L) {
      std::string package = luaL_checkstring(L, 1);
      try {
        std::string sharedir = AmentIndex::getInstance().get_package_share_directory(package);
        lua_pushstring(L,sharedir.c_str());
        return 1;  /* number of results*/
      } catch( ament_index_cpp::PackageNotFoundError& err) {
        luaL_error(L,fmt::format("package {} not found",err.package_name).c_str()); 
        // the above statement returns the function
        return 0;
      }
    }


static int l_get_package_prefix (lua_State *L) {
      std::string package = luaL_checkstring(L, 1);
      try {
        std::string prefix = AmentIndex::getInstance().get_package_prefix(package);
        lua_pushstring(L,prefix.c_str());
        return 1;  /* number of results*/
      } catch( ament_index_cpp::PackageNotFoundError& err) {
        luaL_error(L,fmt::format("package {} not found",err.package_name).c_str()); 
        // the above statement returns the function
        return 0;
      }
    }

static const struct luaL_reg ros2lua[] = {
    {"get_package_share_directory", l_get_package_share_directory},
    {"get_package_prefix", l_get_package_prefix},
    //{"create_ros_node", l_create_ros_node},
    //{"get_nr_of_rosnodes",l_get_nr_of_rosnodes},
    {NULL, NULL}  /* sentinel */
};


extern "C" int luaopen_libamentlua(lua_State* L)
{
    //lua_pushcfunction(l, get_package_share_directory);
    //lua_setglobal(l, "get_package_share_directory");
    luaL_openlib(L, "ros2lua", ros2lua, 0);
    return 1;
}
