#ifndef __BASE_HPP__
#define __BASE_HPP__

#include <utils/string.hpp>
#include <utils/type.hpp>

namespace visopt {
    class Base {
        public:
            virtual const std::string getName() const {
                std::string className = instant::Utils::Type::GetTypeName(this);
                return instant::Utils::String::Replace(className, "visopt::", "");
            }
    };
}

#endif //__BASE_HPP__
