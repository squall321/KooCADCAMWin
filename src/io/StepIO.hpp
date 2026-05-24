#pragma once
// @lat: [[engine/parametric-templates#STEP Export]]

#include <TopoDS_Shape.hxx>

#include <filesystem>
#include <optional>
#include <string>

namespace koocadcam::io {

class StepIO
{
public:
    // Write `shape` to STEP file at `path`. Returns true on success.
    // On failure, populates `err` with a human-readable message.
    // Schema: AP214, units: MM.
    static bool write(const TopoDS_Shape& shape,
                      const std::filesystem::path& path,
                      std::string& err) noexcept;

    // Read a STEP file. Returns the loaded compound shape, or std::nullopt on failure.
    // On failure, populates `err`.
    static std::optional<TopoDS_Shape> read(const std::filesystem::path& path,
                                            std::string& err) noexcept;
};

}  // namespace koocadcam::io
