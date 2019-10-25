/*
 * utils.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_UTILS_H_
#define INCLUDE_NOETHER_FILTERING_UTILS_H_

#include <typeinfo>
#include <memory>
#include <string>

namespace noether_filtering
{
namespace utils
{
template <class C>
static std::string getClassName() {

    int status = -4; // some arbitrary value to eliminate the compiler warning
    const char* mangled_name = typeid(C).name();

    // enable c++11 by passing the flag -std=c++11 to g++
    std::unique_ptr<char, void(*)(void*)> res {
        abi::__cxa_demangle(mangled_name, NULL, NULL, &status),
        std::free
    };
    return (status==0) ? res.get() : mangled_name ;
}

}
}


#endif /* INCLUDE_NOETHER_FILTERING_UTILS_H_ */
