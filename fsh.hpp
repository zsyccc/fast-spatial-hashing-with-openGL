#pragma once
#ifndef FSH_HPP
#define FSH_HPP

#include <string>
#include <functional>
#include <iostream>
#include <vector>
#include "point.hpp"
#include "util.hpp"

#define VALUE(x) std::cout << #x "=" << x << std::endl

namespace fsh {
    template <uint d, class T, class PosInt, class HashInt>
    class map {
    public:
        struct data_t {
            point<d, PosInt> location;
            point<d, PosInt> normal;
            T contents;
        };
    };
}  // namespace fsh

#endif