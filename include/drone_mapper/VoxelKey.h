#pragma once

#include <cstddef>
#include <functional>
#include <tuple>

namespace drone_mapper {

using VoxelKey = std::tuple<int, int, int>;

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const noexcept {
        auto h = std::hash<int>{};
        std::size_t s = h(std::get<0>(k));
        s ^= h(std::get<1>(k)) + 0x9e3779b9u + (s << 6) + (s >> 2);
        s ^= h(std::get<2>(k)) + 0x9e3779b9u + (s << 6) + (s >> 2);
        return s;
    }
};

} // namespace drone_mapper
