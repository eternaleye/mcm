#ifndef _LIBMCM_ERROR_HPP_
#define _LIBMCM_ERROR_HPP_

#include <stdexcept>  // for logic_error
#include <string>     // for string

namespace libmcm {
    class unimplemented_error : public std::logic_error {
        public:
            unimplemented_error(const std::string& what_arg)
                // TODO<C++17>: Use s"" string
                : std::logic_error(std::string("Calling unimplemented function ") + what_arg) {}

            unimplemented_error(const char* what_arg) : unimplemented_error(std::string(what_arg)) {}
    };
}

#endif // _LIBMCM_ERROR_HPP_

