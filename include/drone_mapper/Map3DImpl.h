#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <cstddef>
#include <filesystem>
#include <memory>

namespace drone_mapper {

// Dense occupancy map backed by a TinyNPY array plus an offset-aware MapConfig.
// World -> index: i = floor((world - offset) / resolution) per axis; cell
// (0,0,0) sits at world `offset`; flat C-order index = (ix*ny + iy)*nz + iz.
class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    // Changed: added offset-aware construction for hidden maps loaded from NPY files.
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    // Changed: exposes boundaries, offset, and resolution as one map-owned configuration.
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    //Mutable map methods
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    // cm-space grid geometry derived from the NPY shape + MapConfig: cell
    // (0,0,0) sits at world `offset`, so index = floor((world - offset) / res).
    struct GridGeometry {
        double res_cm = 0.0;
        double origin_x_cm = 0.0;
        double origin_y_cm = 0.0;
        double origin_z_cm = 0.0;
        std::size_t nx = 0;
        std::size_t ny = 0;
        std::size_t nz = 0;
    };

    [[nodiscard]] static bool hasData(NpyArray& array);
    [[nodiscard]] static bool hasBounds(const types::MappingBounds& bounds);
    [[nodiscard]] static GridGeometry geometryOf(NpyArray& array, const types::MapConfig& config);
    // True iff `pos` is inside the configured boundaries (when set) or inside
    // the array footprint otherwise. Shared by atVoxel, set and isInBounds.
    [[nodiscard]] static bool withinBounds(const Position3D& pos,
                                           const types::MapConfig& config,
                                           const GridGeometry& geometry);
    // dtype-aware raw cell access: uint8 is reinterpreted as int8 so a stored
    // 255 reads back as Unmapped.
    [[nodiscard]] static int rawAt(NpyArray& array, std::size_t flat);
    static void writeRaw(NpyArray& array, std::size_t flat, types::VoxelOccupancy value);
    [[nodiscard]] static types::VoxelOccupancy occupancyFromRaw(int raw);
    // Builds the dense int8 grid for an output map over [offset, bounds max),
    // every cell initialised to Unmapped.
    static void allocateFromBounds(NpyArray& array, const types::MapConfig& config);

    // Changed: shared ownership supports the new pointer-based storage member.
    std::shared_ptr<NpyArray> map_;
    // Changed: replaces standalone resolution_ so all map geometry stays together.
    types::MapConfig config_;
};

} // namespace drone_mapper
