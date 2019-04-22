#pragma once
#ifndef FSH_HPP
#define FSH_HPP

#include <string>
#include <functional>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "point.hpp"
#include "util.hpp"
#include "bitset.hpp"

#define VALUE(x) std::cout << #x "=" << x << std::endl

namespace fsh {
    // creates a perfect hash for a predefined data set
    // d is the dimensionality, T is the data type
    // PosInt is the integer type used for positions
    template <uint d, class T, class PosInt, class NorInt>
    class map {
        // private:
    public:
        using IndexInt = size_t;

        // bouding box
        point<d, PosInt> box;

        // data size
        IndexInt n;

        // global offset for data
        PosInt offset;

        // normal table indices
        std::vector<bitset> normal_indices[d];

        // normal table
        std::vector<point<d, NorInt>> normals;

        class entry;
        class redirct_entry;

        // hash table
        std::vector<entry> H;

        std::vector<redirct_entry> phi;

    public:
        struct data_t {
            point<d, PosInt> location;
            point<d, NorInt> normal;
            T contents;
        };
        using data_function = std::function<data_t(IndexInt)>;

        map(const data_function& data, IndexInt n) : n(n), offset(0) {
            for (int i = 0; i < d; i++) {
                box[i] = 0;
            }
            for (int i = 0; i < n; i++) {
                const data_t& it = data(i);
                for (int i = 0; i < d; i++) {
                    box[i] = std::max(box[i], it.location[i]);
                }
            }
            create_normal_table(data);
            size_t fail = 0;
            while (true) {
                if (create(data)) {
                    break;
                }
                fail++;
                for (int i = 0; i < d; i++) {
                    box[i] += 2 * d;
                }
                offset += d;
            }
        }

        // private:
    public:
        class entry {
            point<d, NorInt> normal;
            size_t redirct_index;
        };

        class redirct_entry {
            std::vector<PosInt> redirect;
        };

        void create_normal_table(const data_function& data) {
            std::unordered_map<point<d, NorInt>, size_t> m;
            int nnormal = 0;
            for (size_t i = 0; i < n; i++) {
                const data_t& it = data(i);
                if (!m.count(it.normal)) {
                    m[it.normal] = nnormal++;
                }
            }
            for (int i = 0; i < d; i++) {
                point<d, PosInt> bound = box + (PosInt)1;
                normal_indices[i].clear();
                normal_indices[i].reserve(bound[i]);
                std::fill_n(std::back_inserter(normal_indices[i]), bound[i],
                            bitset(nnormal));
            }
            for (size_t i = 0; i < n; i++) {
                const data_t& it = data(i);
                for (uint j = 0; j < d; j++) {
                    normal_indices[j][it.location[j]].add(m[it.normal]);
                }
            }
        }

        void move_to_box(point<d, PosInt>& v, const point<d, NorInt>& vn) {
            double len = std::max(box[0], std::max(box[1], box[2]));
            v += offset;
            for (uint i = 0; i < d; i++) {
                double move = len;
                if (vn[i] > 0) {
                    move = 1.0 * (box[i] - v[i]) / vn[i];
                } else if (vn[i] < 0) {
                    move = 1.0 * (0 - v[i]) / vn[i];
                }
                len = std::min(len, move);
            }
            for (uint i = 0; i < d; i++) {
                v[i] += std::round(len * vn[i]);
            }
        }
        size_t h(const point<d, PosInt>& v) {
            size_t ret = -1;
            size_t surface_offset[d];
                }
        bool create(const data_function& data) {
            // move to surface
            std::vector<data_t> suface_data(n);
            for (size_t i = 0; i < n; i++) {
                data_t t = data(i);
                move_to_box(t.location, t.normal);
                suface_data[i] = std::move(t);
            }
            // begin hash
            std::vector<data_t> collision;
            std::vector<entry> H_hat;
            size_t sz = 1;
            for (uint i = 0; i < d; i++) {
                sz *= box[i];
            }
            H_hat.reserve(sz);
        }
    };
}  // namespace fsh

#endif