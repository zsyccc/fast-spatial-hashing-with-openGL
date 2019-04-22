#pragma once
#ifndef FSH_HPP
#define FSH_HPP

#include <string>
#include <functional>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "point.hpp"
#include "util.hpp"
#include "bitset.hpp"

#define VALUE(x) std::cout << #x "=" << x << std::endl

namespace fsh {
    // creates a perfect hash for a predefined data set
    // d is the dimensionality, T is the data type
    // PosInt is the integer type used for positions
    template <uint d, class T, class PosInt, class NorInt, class HashInt>
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
            while (hash_table_size() < n) {
                box = box + (PosInt)(2 * d);
                offset += d;
            }
            create_normal_table(data);
            create(data);
        }

        // private:
    public:
        size_t hash_table_size() const {
            size_t mul = 1;
            for (uint i = 0; i < d; i++) {
                mul *= (size_t)box[i];
            }
            size_t sz = 0;
            for (uint i = 0; i < d; i++) {
                sz += mul / box[i] * 2;
            }
            return sz;
        }
        struct entry {
            point<d, NorInt> normal;
            size_t redirct_index;
            entry() : normal({0, 0, 0}), redirct_index(0) {}
        };

        struct entry_large : public entry {
            point<d, PosInt> location;
        };

        struct redirct_entry {
            // using HashInt = uint_8;
            std::vector<PosInt> redirect;
            HashInt k;
            redirct_entry() : k(1) {}
            HashInt h(const point<d, NorInt>& p) const { return h(p, k); }
            static HashInt h(const point<d, NorInt>& p, HashInt k) {
                PosInt u = 0;
                PosInt mul = 1;
                for (uint i = 0; i < d; i++) {
                    u += p[i] * mul;
                    mul *= NorInt(-1);
                }
                return u % k;
            }
        };
        struct redirct_entry_large : public redirct_entry {
            std::unordered_map<point<d, NorInt>, size_t> normal_redircts;
            size_t index;
            redirct_entry_large() {}
            redirct_entry_large(size_t index) : redirct_entry(), index(index) {}
        };

        void create_normal_table(const data_function& data) {
            std::unordered_map<point<d, NorInt>, size_t> m;
            int nnormal = 0;
            for (size_t i = 0; i < n; i++) {
                const data_t& it = data(i);
                if (!m.count(it.normal)) {
                    m[it.normal] = nnormal++;
                    assert((it.normal != point<d, NorInt>::point_zero()));
                }
            }
            for (int i = 0; i < d; i++) {
                point<d, PosInt> bound = box + (PosInt)1;
                normal_indices[i].clear();
                normal_indices[i].resize(bound[i], bitset(nnormal));
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
            v = v + offset;
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
        constexpr size_t find_surface(const point<d, PosInt>& v) const {
            int ret = 0;
            for (uint i = 0; i < d; i++) {
                bool ok = true;
                for (uint j = 0; j < d; j++) {
                    if (i != j && (v[j] < 0 || v[j] >= box[j])) {
                        ok = false;
                        break;
                    }
                }
                if (ok && v[i] == 0) break;
                ret++;
                if (ok && v[i] == box[i]) break;
                ret++;
            }
            assert(ret != d * 2);
            return ret;
        }
        constexpr size_t h(const point<d, PosInt>& v) const {
            size_t surface_offset[d];
            size_t surface_addr[2 * d];
            size_t mul = 1;
            for (uint i = 0; i < d; i++) {
                mul *= (size_t)box[i];
            }
            for (uint i = 0; i < d; i++) {
                surface_offset[i] = mul / box[i];
            }
            surface_addr[0] = 0;
            for (uint i = 1; i < 2 * d; i++) {
                surface_addr[i] = surface_addr[i - 1] + surface_offset[i / 2];
            }
            size_t surface_id = find_surface(v);
            size_t ret = surface_addr[surface_id];
            point<d - 1, PosInt> p;
            point<d - 1, PosInt> bound;
            int cur = 0;
            for (uint i = 0; i < d; i++) {
                if (i != surface_id / 2) {
                    p[cur] = v[i];
                    bound[cur] = box[i];
                    cur++;
                }
            }
            ret += point_to_index(p, bound, uint(-1));
            return ret;
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
            std::queue<data_t> collision;
            std::vector<entry_large> H_hat;
            size_t table_size = hash_table_size();
            H_hat.resize(table_size, entry_large());
            for (const auto& it : suface_data) {
                auto index = h(it.location);
                if (H_hat[index].normal == point<d, NorInt>::point_zero()) {
                    H_hat[index].normal = it.normal;
                    H_hat[index].location = it.location;
                } else {
                    collision.push(it);
                }
            }
            std::unordered_map<size_t, redirct_entry_large> redirect_hat;
            for (size_t i = 0; i < table_size && !collision.empty(); i++) {
                if (H_hat[i].normal == point<d, NorInt>::point_zero()) {
                    auto it = collision.front();
                    collision.pop();
                    auto index = h(it.location);
                    if (!redirect_hat.count(index)) {
                        redirect_hat[index] = redirct_entry_large(index);
                        redirect_hat[index]
                            .normal_redircts[H_hat[index].normal] = index;
                        // std::cout << H_hat[index].location << std::endl;
                    }
                    // std::cout << it.location << std::endl;
                    redirect_hat[index].normal_redircts[it.normal] = i;
                }
            }
            // VALUE(redirect_hat.size());
            std::vector<redirct_entry> phi_hat;
            for (auto& it : redirect_hat) {
                const auto& index = it.first;
                auto& r = it.second;
                bool ok = getK(r, r.k);
                if (!ok) return false;
                r.redirect.resize(r.k);
                for (const auto& it : r.normal_redircts) {
                    r.redirect[r.h(it.first)] = it.second;
                }
                phi_hat.push_back(r);
                H_hat[index].redirct_index = phi_hat.size() - 1;
            }
            // done!
            phi = std::move(phi_hat);
            H.clear();
            H.reserve(H_hat.size());
            std::copy(H_hat.begin(), H_hat.end(), std::back_inserter(H));
            return true;
        }
        bool getK(const redirct_entry_large& r, HashInt& k) {
            std::set<HashInt> s;
            k = r.normal_redircts.size() - 1;
            do {
                k++;
                s.clear();
                for (const auto& it : r.normal_redircts) {
                    HashInt u = redirct_entry::h(it.first, k);
                    s.insert(u);
                }
            } while (s.size() != r.normal_redircts.size());
        }
    };
}  // namespace fsh

#endif