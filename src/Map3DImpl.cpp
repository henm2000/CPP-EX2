#include <drone_mapper/Map3DImpl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {

bool Map3DImpl::hasData(NpyArray& array) {
    return !array.Shape().empty();
}

bool Map3DImpl::hasBounds(const types::MappingBounds& bounds) {
    return bounds.max_x > bounds.min_x &&
           bounds.max_y > bounds.min_y &&
           bounds.max_height > bounds.min_height;
}

Map3DImpl::GridGeometry Map3DImpl::geometryOf(NpyArray& array, const types::MapConfig& config) {
    GridGeometry g;
    g.res_cm = config.resolution.force_numerical_value_in(cm);
    g.origin_x_cm = config.offset.x.force_numerical_value_in(cm);
    g.origin_y_cm = config.offset.y.force_numerical_value_in(cm);
    g.origin_z_cm = config.offset.z.force_numerical_value_in(cm);
    if (hasData(array)) {
        const auto& shape = array.Shape();
        g.nx = shape[0];
        g.ny = shape[1];
        g.nz = shape[2];
    }
    return g;
}

// World-space bounds: explicit boundaries when configured, otherwise the array
// footprint [offset, offset + shape * resolution). Half-open ranges.
bool Map3DImpl::withinBounds(const Position3D& pos,
                             const types::MapConfig& config,
                             const GridGeometry& g) {
    const double x_cm = pos.x.force_numerical_value_in(cm);
    const double y_cm = pos.y.force_numerical_value_in(cm);
    const double z_cm = pos.z.force_numerical_value_in(cm);

    if (hasBounds(config.boundaries)) {
        return x_cm >= config.boundaries.min_x.force_numerical_value_in(cm) &&
               x_cm < config.boundaries.max_x.force_numerical_value_in(cm) &&
               y_cm >= config.boundaries.min_y.force_numerical_value_in(cm) &&
               y_cm < config.boundaries.max_y.force_numerical_value_in(cm) &&
               z_cm >= config.boundaries.min_height.force_numerical_value_in(cm) &&
               z_cm < config.boundaries.max_height.force_numerical_value_in(cm);
    }
    return x_cm >= g.origin_x_cm && x_cm < g.origin_x_cm + static_cast<double>(g.nx) * g.res_cm &&
           y_cm >= g.origin_y_cm && y_cm < g.origin_y_cm + static_cast<double>(g.ny) * g.res_cm &&
           z_cm >= g.origin_z_cm && z_cm < g.origin_z_cm + static_cast<double>(g.nz) * g.res_cm;
}

// Input maps are uint8/bool Minecraft-style block grids (0 = air, any non-zero
// block id = solid). Our own saved output maps are int8 with the VoxelOccupancy
// sentinels. The dtype char decides which convention applies.
types::VoxelOccupancy Map3DImpl::occupancyAt(NpyArray& array, std::size_t flat) {
    if (array.Type() == 'i') {
        const int value = array.Data<std::int8_t>()[flat];
        switch (value) {
            case -1: return types::VoxelOccupancy::Unmapped;
            case -2: return types::VoxelOccupancy::OutOfBounds;
            case -3: return types::VoxelOccupancy::PotentiallyOccupied;
            case 0: return types::VoxelOccupancy::Empty;
            default: return types::VoxelOccupancy::Occupied; // 1 or any other block id
        }
    }
    const unsigned value = array.Data<std::uint8_t>()[flat];
    return (value == 0) ? types::VoxelOccupancy::Empty : types::VoxelOccupancy::Occupied;
}

void Map3DImpl::writeRaw(NpyArray& array, std::size_t flat, types::VoxelOccupancy value) {
    const auto raw = static_cast<std::int8_t>(value);
    if (array.Type() == 'i') {
        array.Data<std::int8_t>()[flat] = raw;
    } else {
        array.Data<std::uint8_t>()[flat] = static_cast<std::uint8_t>(raw);
    }
}

std::shared_ptr<NpyArray> Map3DImpl::allocateFromBounds(const types::MapConfig& config) {
    const double res_cm = config.resolution.force_numerical_value_in(cm);
    if (res_cm <= 0.0) {
        throw std::runtime_error("Map3DImpl: resolution must be positive to allocate a map");
    }

    const double origin_cm[3] = {
        config.offset.x.force_numerical_value_in(cm),
        config.offset.y.force_numerical_value_in(cm),
        config.offset.z.force_numerical_value_in(cm),
    };
    const double min_cm[3] = {
        config.boundaries.min_x.force_numerical_value_in(cm),
        config.boundaries.min_y.force_numerical_value_in(cm),
        config.boundaries.min_height.force_numerical_value_in(cm),
    };
    const double max_cm[3] = {
        config.boundaries.max_x.force_numerical_value_in(cm),
        config.boundaries.max_y.force_numerical_value_in(cm),
        config.boundaries.max_height.force_numerical_value_in(cm),
    };

    std::size_t cells[3] = {0, 0, 0};
    for (int axis = 0; axis < 3; ++axis) {
        // Cell indices cannot go negative, so the matrix origin must not sit
        // past the low edge of the requested boundaries.
        if (min_cm[axis] < origin_cm[axis]) {
            throw std::runtime_error("Map3DImpl: offset must not exceed boundary minimum");
        }
        cells[axis] = static_cast<std::size_t>(
            std::ceil((max_cm[axis] - origin_cm[axis]) / res_cm));
        if (cells[axis] == 0) {
            throw std::runtime_error("Map3DImpl: empty mapping bounds");
        }
    }

    const std::vector<std::size_t> shape{cells[0], cells[1], cells[2]};
    // Use the explicit-dtype constructor: it OWNS its buffer (the templated T*
    // constructor is only a non-owning view) and is tagged int8 ('i'), since
    // TinyNPY's type deduction otherwise mislabels int8_t as numpy bool.
    auto array = std::make_shared<NpyArray>(shape, sizeof(std::int8_t), 'i', false);
    array->Allocate();
    std::int8_t* out = array->Data<std::int8_t>();
    std::fill(out, out + array->NumValue(),
              static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return array;
}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
    if (hasData(*map_)) {
        if (map_->Shape().size() != 3) {
            throw std::runtime_error("Map3DImpl: expected a 3-D NPY array");
        }
        const char type = map_->Type();
        if (type != 'u' && type != 'b' && type != 'i') {
            throw std::runtime_error(
                std::string{"Map3DImpl: unsupported NPY dtype char '"} + type + "'");
        }
    } else if (hasBounds(config_.boundaries)) {
        map_ = allocateFromBounds(config_);
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    NpyArray& array = *map_;
    const GridGeometry g = geometryOf(array, config_);
    if (g.res_cm <= 0.0 || !hasData(array)) {
        return types::VoxelOccupancy::OutOfBounds;
    }
    if (!withinBounds(pos, config_, g)) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    const auto ix = static_cast<long>(
        std::floor((pos.x.force_numerical_value_in(cm) - g.origin_x_cm) / g.res_cm));
    const auto iy = static_cast<long>(
        std::floor((pos.y.force_numerical_value_in(cm) - g.origin_y_cm) / g.res_cm));
    const auto iz = static_cast<long>(
        std::floor((pos.z.force_numerical_value_in(cm) - g.origin_z_cm) / g.res_cm));
    if (ix < 0 || iy < 0 || iz < 0 ||
        static_cast<std::size_t>(ix) >= g.nx ||
        static_cast<std::size_t>(iy) >= g.ny ||
        static_cast<std::size_t>(iz) >= g.nz) {
        // Inside the configured boundaries but not covered by the file:
        // hidden-map semantics, nothing listed means free space.
        return types::VoxelOccupancy::Empty;
    }

    const std::size_t flat =
        (static_cast<std::size_t>(ix) * g.ny + static_cast<std::size_t>(iy)) * g.nz +
        static_cast<std::size_t>(iz);
    return occupancyAt(array, flat);
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    NpyArray& array = *map_;
    const GridGeometry g = geometryOf(array, config_);
    if (g.res_cm <= 0.0 || !hasData(array)) {
        return false;
    }
    return withinBounds(pos, config_, g);
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    NpyArray& array = *map_;
    const GridGeometry g = geometryOf(array, config_);
    if (g.res_cm <= 0.0 || !hasData(array)) {
        return;
    }
    if (!withinBounds(pos, config_, g)) {
        return;
    }

    const auto ix = static_cast<long>(
        std::floor((pos.x.force_numerical_value_in(cm) - g.origin_x_cm) / g.res_cm));
    const auto iy = static_cast<long>(
        std::floor((pos.y.force_numerical_value_in(cm) - g.origin_y_cm) / g.res_cm));
    const auto iz = static_cast<long>(
        std::floor((pos.z.force_numerical_value_in(cm) - g.origin_z_cm) / g.res_cm));
    if (ix < 0 || iy < 0 || iz < 0 ||
        static_cast<std::size_t>(ix) >= g.nx ||
        static_cast<std::size_t>(iy) >= g.ny ||
        static_cast<std::size_t>(iz) >= g.nz) {
        return;
    }

    const std::size_t flat =
        (static_cast<std::size_t>(ix) * g.ny + static_cast<std::size_t>(iy)) * g.nz +
        static_cast<std::size_t>(iz);
    writeRaw(array, flat, value);
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    if (!hasData(*map_)) {
        throw std::runtime_error("Map3DImpl::save: map has no data to save");
    }
    const char* error = map_->SaveNPY(output_path.string().c_str());
    if (error != nullptr) {
        throw std::runtime_error(std::string{"Map3DImpl::save: "} + error);
    }
}

} // namespace drone_mapper
