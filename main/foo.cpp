#include <iostream>

#include "foo.hpp"

int main(int argc, char* argv[]) {

    Foo foo;
    std::cout << foo.bar() << std::endl;

    return 0;
}
