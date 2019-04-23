#pragma once
#ifndef FSH_BITSET_HPP
#define FSH_BITSET_HPP

#include <vector>
#include <iostream>

#define BIT_CAPACITY(type) (sizeof(type) * 8)

namespace fsh {
    class bitset {
    private:
        using scaler = uint8_t;
        size_t size;
        std::vector<scaler> data;
        size_t data_size;

    public:
        size_t memory_size() const {
            return sizeof(*this) + sizeof(typename decltype(data)::value_type) *
                                       data.capacity();
        }
        bitset() : size(0) {}
        bitset(size_t size)
            : size(size),
              data(std::ceil(1.0 * size / BIT_CAPACITY(scaler)), 0),
              data_size(data.size()) {
            // data_size = data.size();
        }
        bitset& add(size_t k) {
            if (k >= size) {
                std::cout << "error in bitset: add" << std::endl;
                return *this;
            }
            size_t pos = k / BIT_CAPACITY(scaler);
            size_t cur = k % BIT_CAPACITY(scaler);
            data[pos] |= ((scaler)1 << cur);
            return *this;
        }
        bitset& sub(size_t k) {
            if (k >= size) {
                std::cout << "error in bitset: sub" << std::endl;
                return *this;
            }
            size_t pos = k / BIT_CAPACITY(scaler);
            size_t cur = k % BIT_CAPACITY(scaler);
            data[pos] &= ((scaler)-1 ^ ((scaler)1 << cur));
            return *this;
        }
        bitset operator&(const bitset& rhs) const {
            bitset ret(size);
            if (size != rhs.size) {
                std::cout << "error in bitset: &" << std::endl;
                return ret;
            }
            for (size_t i = 0; i < data_size; i++) {
                ret.data[i] = data[i] & rhs.data[i];
            }
            return ret;
        }
        bitset operator&=(const bitset& rhs) { return *this = (*this & rhs); }
        int find_fist() const {
            int ret = -1;
            for (size_t i = 0; i < data_size && ret == -1; i++) {
                if (data[i]) {
                    for (size_t j = 0; j < BIT_CAPACITY(scaler); j++) {
                        if (data[i] & ((scaler)1 << j)) {
                            ret = i * BIT_CAPACITY(scaler) + j;
                            break;
                        }
                    }
                }
            }
            return ret;
        }
        int find_last() const {
            int ret = -1;
            for (size_t i = 0; i < data_size && ret == -1; i++) {
                size_t s = data_size - 1 - i;
                if (data[s]) {
                    for (size_t j = 0; j < BIT_CAPACITY(scaler); j++) {
                        size_t ss = BIT_CAPACITY(scaler) - j - 1;
                        if (data[s] & ((scaler)1 << ss)) {
                            ret = s * BIT_CAPACITY(scaler) + ss;
                            break;
                        }
                    }
                }
            }
            return ret;
        }
        void display() const {
            for (size_t i = 0; i < size; i++) {
                size_t pos = i / BIT_CAPACITY(scaler);
                size_t cur = i % BIT_CAPACITY(scaler);
                if (data[pos] & ((scaler)1 << cur)) {
                    std::cout << 1;
                } else {
                    std::cout << 0;
                }
            }
            std::cout << std::endl;
        }
    };
}  // namespace fsh

#endif