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

#define NOT_FOUND_EXCEPTION() \
    throw std::out_of_range("Element not found in map")

namespace fsh {
    // creates a perfect hash for a predefined data set
    // d is the dimensionality, T is the data type
    // PosInt is the integer type used for positions
    // NorInt is the integer type used for normal vectors
    // HashInt is the integer type used for handling collisions
    template <uint d, class T, class PosInt, class NorInt, class HashInt>
    class map {
    private:
        // public:
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

        // redirct table
        std::vector<redirct_entry> phi;

    public:
        struct data_t {
            point<d, PosInt> location;
            point<d, NorInt> normal;
            T contents;
        };
        struct data_t_large : public data_t {
            PosInt distance;
            data_t_large() {}
            data_t_large(const data_t& data) : data_t(data), distance() {}
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
            while (!create(data)) {
                box = box + (PosInt)(2 * d);
                offset += d;
                VALUE(box);
            }
        }

        const T& get(const point<d, PosInt>& p) const { return get(p); }

        T& get(const point<d, PosInt>& p) {
            for (uint i = 0; i < d; i++) {
                if (p[i] >= normal_indices[i].size()) {
                    NOT_FOUND_EXCEPTION();
                }
            }
            int index = get_normal_index(p);
            if (index == -1) {
                NOT_FOUND_EXCEPTION();
            }
            const point<d, NorInt>& vn = normals[index];
            point<d, PosInt> surface_point = p;
            PosInt dist = move_to_box(surface_point, vn);
            size_t H_index = h(surface_point);
            entry& en = H[H_index];
            if (en.redirct_index == 0) {
                if (en.redirected == false && en.equals(vn, dist)) {
                    return en.contents;
                } else {
                    NOT_FOUND_EXCEPTION();
                }
            } else {
                size_t R_index = en.redirct_index - 1;
                const redirct_entry& re = phi[R_index];
                H_index = re.redirect[re.h(vn, dist)];
                if (H_index == 0) NOT_FOUND_EXCEPTION();
                entry& en2 = H[H_index];
                if (en2.equals(vn, dist)) {
                    return en2.contents;
                } else {
                    NOT_FOUND_EXCEPTION();
                }
            }
        }

    private:
        size_t hash_table_size() const {
            size_t mul = 1;
            for (uint i = 0; i < d; i++) {
                mul *= (size_t)box[i] + 1;
            }
            size_t sz = 0;
            for (uint i = 0; i < d; i++) {
                sz += mul / box[i] * 2;
            }
            return sz;
        }
        struct entry_verify {
            point<d, NorInt> normal;
            PosInt distance;
            entry_verify() : normal({0, 0, 0}), distance(0) {}
            entry_verify(const point<d, NorInt>& normal, PosInt distance)
                : normal(normal), distance(distance) {}
            void add(const point<d, NorInt>& normal, PosInt distance) {
                this->normal = normal;
                this->distance = distance;
            }
            bool empty() const {
                return normal == decltype(normal)::point_zero();
            }
            bool operator<(const entry_verify& rhs) const {
                return normal < rhs.normal ||
                       (normal == rhs.normal && distance < rhs.distance);
            }
            bool operator==(const entry_verify& rhs) const {
                return !(*this < rhs) && !(rhs < *this);
            }
            friend class redirct_entry;
        };
        struct entry {
            entry_verify verify;
            size_t redirct_index;
            T contents;
            bool redirected;
            entry() : redirct_index(0), redirected(false) {}
            bool equals(const point<d, NorInt>& nor, PosInt dist) const {
                return equals(entry_verify(nor, dist));
            }
            bool equals(const entry_verify& v) const { return verify == v; }
        };

        // struct entry_large : public entry {
        //     point<d, PosInt> location;
        // };

        struct redirct_entry {
            std::vector<size_t> redirect;
            HashInt k;
            redirct_entry() : k(1) {}
            static HashInt h(const entry_verify& verify, HashInt k) {
                size_t u = 0;
                size_t mul = 1;
                for (uint i = 0; i < d; i++) {
                    u += verify.normal[i] * mul;
                    mul *= 3145739;
                }
                u += verify.distance * mul;
                return u % k;
            }
            static HashInt h(const point<d, NorInt>& normal, PosInt distance,
                             HashInt k) {
                return h(entry_verify(normal, distance), k);
            }
            HashInt h(const entry_verify& verify) const { return h(verify, k); }
            HashInt h(const point<d, NorInt>& normal, PosInt distance) const {
                return h(normal, distance, k);
            }
        };
        struct redirct_entry_large : public redirct_entry {
            std::map<entry_verify, size_t> redirect_table;
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
            normals.resize(nnormal);
            for (const auto& it : m) {
                normals[it.second] = it.first;
            }
        }
        bool on_box(point<d, PosInt>& v) const {
            bool ok = false;
            for (uint i = 0; i < d; i++) {
                if (v[i] == 0 || v[i] == box[i]) {
                    ok = true;
                    break;
                }
            }
            return ok;
        }
        void fix_to_box(point<d, PosInt>& v) const {
            PosInt minVal = v[0];
            uint cur = 0;
            uint minid = 0;
            for (uint i = 0; i < d; i++) {
                if (minVal > v[i]) {
                    minVal = v[i];
                    minid = cur;
                }
                cur++;
                if (minVal > box[i] - v[i]) {
                    minVal = box[i] - v[i];
                    minid = cur;
                }
                cur++;
            }
            v[minid / 2] = minid % 2 ? box[minid / 2] : 0;
        }
        PosInt move_to_box(point<d, PosInt>& v,
                           const point<d, NorInt>& vn) const {
            PosInt maxbox = std::max(box[0], std::max(box[1], box[2]));
            double len = maxbox;
            v += offset;
            for (uint i = 0; i < d; i++) {
                double move = len;
                if (vn[i] > 0) {
                    move = ((double)box[i] - (double)v[i]) / (double)vn[i];
                } else if (vn[i] < 0) {
                    move = (0.0 - (double)v[i]) / (double)vn[i];
                }
                len = std::min(len, move);
            }
            for (uint i = 0; i < d; i++) {
                v[i] += std::round(len * vn[i]);
            }
            assert(on_box(v));
            return (PosInt)std::round(len * ((unsigned)PosInt(-1) >> 4));
        }
        constexpr size_t find_surface(const point<d, PosInt>& v) const {
            int ret = 0;
            for (uint i = 0; i < d; i++) {
                bool ok = true;
                for (uint j = i + 1; j < d; j++) {
                    if (v[j] == box[j]) {
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
                surface_addr[i] =
                    surface_addr[i - 1] + surface_offset[(i - 1) / 2];
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
            std::vector<data_t_large> suface_data(n);
            for (size_t i = 0; i < n; i++) {
                data_t_large t = data(i);
                t.normal = normals[get_normal_index(t.location)];
                t.distance = move_to_box(t.location, t.normal);
                suface_data[i] = std::move(t);
            }
            // begin hash
            std::queue<data_t_large> collision;
            std::vector<entry> H_hat;
            const size_t table_size = hash_table_size();
            H_hat.resize(table_size, entry());
            for (const auto& it : suface_data) {
                auto index = h(it.location);
                assert(index < table_size);
                if (H_hat[index].verify.empty()) {
                    H_hat[index].verify.add(it.normal, it.distance);
                    H_hat[index].contents = it.contents;
                } else {
                    collision.push(it);
                }
            }
            std::unordered_map<size_t, redirct_entry_large> redirect_hat;
            for (size_t i = 1; i < table_size && !collision.empty(); i++) {
                if (H_hat[i].verify.empty()) {
                    auto it = collision.front();
                    collision.pop();
                    auto index = h(it.location);
                    if (!redirect_hat.count(index)) {
                        redirect_hat[index] = redirct_entry_large(index);
                        redirect_hat[index]
                            .redirect_table[H_hat[index].verify] = index;
                    }
                    // std::cout << it.location << std::endl;
                    entry_verify verify(it.normal, it.distance);
                    redirect_hat[index].redirect_table[verify] = i;
                    H_hat[i].verify = verify;
                    // H_hat[i].location = it.location;
                    H_hat[i].contents = it.contents;
                    H_hat[i].redirected = true;
                }
            }
            VALUE(redirect_hat.size());
            std::vector<redirct_entry> phi_hat;
            for (auto& it : redirect_hat) {
                const auto& index = it.first;
                auto& r = it.second;
                bool ok = getK(r, r.k);
                if (!ok) {
                    return false;
                }
                r.redirect.resize(r.k, 0);
                for (const auto& it : r.redirect_table) {
                    r.redirect[r.h(it.first)] = it.second;
                }
                phi_hat.push_back(r);
                H_hat[index].redirct_index = phi_hat.size();
            }
            // done
            phi = std::move(phi_hat);
            H = std::move(H_hat);
            return true;
        }
        bool getK(const redirct_entry_large& r, HashInt& k) {
            std::set<HashInt> s;
            k = r.redirect_table.size();
            assert(k >= 2);
            do {
                k++;
                if (k == 0) return false;
                s.clear();
                for (const auto& it : r.redirect_table) {
                    HashInt u = redirct_entry::h(it.first, k);
                    s.insert(u);
                }
            } while (s.size() != r.redirect_table.size());
            return true;
        }
        size_t get_normal_index(const point<d, PosInt>& p) const {
            bitset bit = normal_indices[0][p[0]];
            for (uint i = 1; i < d; i++) {
                bit &= normal_indices[i][p[i]];
            }
            return bit.find_fist();
        }
    };
}  // namespace fsh

#endif
