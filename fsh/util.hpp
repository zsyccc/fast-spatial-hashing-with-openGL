#pragma once

#ifndef FSH_UTIL_HPP
#define FSH_UTIL_HPP

#include <cmath>
#include "point.hpp"
#include <iostream>
namespace fsh {
    namespace {
        // these functions convert between multidimensional (points) and linear
        // (index) coordinates

        // helper struct to allow partial specialization of function templates
        template <uint d, class Int>
        struct point_helpers {
            static constexpr Int point_to_index(const point<d, Int>& p,
                                                const point<d, Int>& bounding,
                                                Int max) {
                Int index = 0;
                Int mul = 1;
                for (uint i = 0; i < d; i++) {
                    uint s = d - 1 - i;
                    index += p[s] * mul;
                    mul *= bounding[s];
                }
                return index % max;
            }

            static constexpr point<d, Int> index_to_point(
                Int index, const point<d, Int>& bounding, Int max) {
                point<d, Int> output;
                for (uint i = 0; i < d - 1; i++) {
                    uint s = d - 1 - i;
                    Int mul = bounding[s];
                    output[s] = index % mul;
                    index /= mul;
                }
                output[0] = index;
                return output;
            }
        };

        // template <class Int>
        // struct point_helpers<2, Int> {
        //     static constexpr Int point_to_index(const point<2, Int>& p,
        //                                         const point<d, Int>&
        //                                         bounding, Int max) {
        //         return (width * p[0] + p[1]) % max;
        //     }

        //     static constexpr point<2, Int> index_to_point(
        //         Int index, const point<d, Int>& bounding, Int max) {
        //         return point<2, Int>{index / width, index % width};
        //     }
        // };

        // template <class Int>
        // struct point_helpers<3, Int> {
        //     static constexpr Int point_to_index(const point<3, Int>& p,
        //                                         const point<d, Int>&
        //                                         bounding, Int max) {
        //         return (p[2] + width * p[1] + width * width * p[0]) % max;
        //     }

        //     static constexpr point<3, Int> index_to_point(
        //         Int index, const point<d, Int>& bounding, Int max) {
        //         return point<3, Int>{index / (width * width),
        //                              (index % (width * width)) / width,
        //                              (index % (width * width)) % width};
        //     }
        // };
    }  // namespace

    template <uint d, class Int>
    constexpr Int point_to_index(const point<d, Int>& p,
                                 const point<d, Int>& bounding, Int max) {
        // static_assert(sizeof(IntS) <= sizeof(IntL),
        //               "IntS must be smaller or equal to IntL");
        return point_helpers<d, Int>::point_to_index(point<d, Int>(p), bounding,
                                                     max);
    }

    template <uint d, class IntS, class IntL>
    constexpr IntL point_to_index(const point<d, IntL>& p,
                                  const point<d, IntS>& bouding, IntL max) {
        static_assert(sizeof(IntS) <= sizeof(IntL),
                      "IntS must be smaller or equal to IntL");
        return point_helpers<d, IntL>::point_to_index(
            point<d, IntL>(p), point<d, IntL>(bouding), max);
    }

    template <uint d, class IntS, class IntL>
    constexpr IntL point_to_index(const point<d, IntS>& p,
                                  const point<d, IntS>& bouding, IntL max) {
        static_assert(sizeof(IntS) <= sizeof(IntL),
                      "IntS must be smaller or equal to IntL");
        return point_helpers<d, IntL>::point_to_index(
            point<d, IntL>(p), point<d, IntL>(bouding), max);
    }

    template <uint d, class IntS, class IntL>
    constexpr point<d, IntS> index_to_point(IntL index,
                                            const point<d, IntS>& bouding,
                                            IntL max) {
        static_assert(sizeof(IntS) <= sizeof(IntL),
                      "IntS must be smaller or equal to IntL");
        return point<d, IntS>(point_helpers<d, IntL>::index_to_point(
            index, point<d, IntL>(bouding), max));
    }
}  // namespace fsh

#endif