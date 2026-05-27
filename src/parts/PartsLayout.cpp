// @lat: [[engine/skills#Layer 6 Parts layout]]

#include "PartsLayout.hpp"

#include "io/StepIO.hpp"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <Bnd_Box.hxx>
#include <TopoDS_Shape.hxx>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace koocadcam::parts {

using nlohmann::json;

namespace {

// Compute the AABB of (placement * shape).  Returns false if the shape is
// null or the bbox is void (e.g. degenerate shape).
bool computeAabb(const TopoDS_Shape& shape,
                 const gp_Trsf& placement,
                 double& xMin, double& yMin, double& zMin,
                 double& xMax, double& yMax, double& zMax)
{
    if (shape.IsNull()) return false;
    try {
        // Apply placement (cheap; OCCT does not deep-copy the shape).  If
        // the trsf is identity, skip the transform step to save a copy.
        TopoDS_Shape placed = shape;
        if (placement.Form() != gp_Identity) {
            BRepBuilderAPI_Transform xf(shape, placement, /*Copy=*/false);
            placed = xf.Shape();
        }
        Bnd_Box box;
        // AddOptimal walks the surface (vs the NURBS control polygon
        // BRepBndLib::Add returns), matching the convention used by
        // src/engine/primitives/Bbox.hpp and Workpiece::boundingBox.
        BRepBndLib::AddOptimal(placed, box);
        if (box.IsVoid()) return false;
        box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        return true;
    } catch (const std::exception& e) {
        spdlog::warn("PartsLayout: AABB computation failed: {}", e.what());
        return false;
    }
}

// Encode a gp_Trsf as a 16-element row-major double array.
json trsfToJson(const gp_Trsf& t)
{
    json arr = json::array();
    // OCCT stores Trsf as 3x4; pad the last row to make a clean 4x4.
    for (int row = 1; row <= 3; ++row) {
        for (int col = 1; col <= 4; ++col) {
            arr.push_back(t.Value(row, col));
        }
    }
    // Bottom row of a homogeneous matrix: 0 0 0 1.
    arr.push_back(0.0);
    arr.push_back(0.0);
    arr.push_back(0.0);
    arr.push_back(1.0);
    return arr;
}

gp_Trsf jsonToTrsf(const json& arr)
{
    gp_Trsf t;
    if (!arr.is_array() || arr.size() != 16) return t;   // identity fallback
    try {
        t.SetValues(
            arr[0].get<double>(),  arr[1].get<double>(),  arr[2].get<double>(),  arr[3].get<double>(),
            arr[4].get<double>(),  arr[5].get<double>(),  arr[6].get<double>(),  arr[7].get<double>(),
            arr[8].get<double>(),  arr[9].get<double>(),  arr[10].get<double>(), arr[11].get<double>());
    } catch (const std::exception& e) {
        spdlog::warn("PartsLayout: trsf decode failed: {}", e.what());
        return gp_Trsf{};
    }
    return t;
}

}  // namespace

// ── Construction ─────────────────────────────────────────────────────────

PartsLayout PartsLayout::fromStepFiles(const std::vector<std::filesystem::path>& steps)
{
    PartsLayout layout;
    for (const auto& p : steps) {
        std::string err;
        auto shapeOpt = io::StepIO::read(p, err);
        if (!shapeOpt.has_value()) {
            spdlog::warn("PartsLayout::fromStepFiles: skipping {} — {}",
                          p.string(), err);
            continue;
        }
        Part part;
        part.id        = p.stem().string();
        part.step_path = p.string();
        part.shape     = *shapeOpt;
        layout.addPart(std::move(part));
    }
    return layout;
}

void PartsLayout::addPart(Part part)
{
    // If the caller did not pre-fill the AABB (all zeros) and the shape is
    // non-null, compute it now.  Pre-filled AABBs (e.g. from tests) are
    // trusted as-is.
    const bool aabbAlreadySet =
        !(part.xMin == 0.0 && part.yMin == 0.0 && part.zMin == 0.0 &&
          part.xMax == 0.0 && part.yMax == 0.0 && part.zMax == 0.0);

    if (!aabbAlreadySet && !part.shape.IsNull()) {
        computeAabb(part.shape, part.placement,
                    part.xMin, part.yMin, part.zMin,
                    part.xMax, part.yMax, part.zMax);
    }
    m_parts.push_back(std::move(part));
}

const Part* PartsLayout::findById(const std::string& id) const
{
    for (const auto& p : m_parts) {
        if (p.id == id) return &p;
    }
    return nullptr;
}

// ── Diff ─────────────────────────────────────────────────────────────────

std::vector<PartsLayout::PartDiff>
PartsLayout::diff(const PartsLayout& other) const
{
    std::vector<PartDiff> result;

    // Index both sides by id for O(N+M) lookup.
    std::unordered_map<std::string, const Part*> mineById, theirsById;
    for (const auto& p : m_parts)        mineById[p.id]   = &p;
    for (const auto& p : other.m_parts)  theirsById[p.id] = &p;

    // Removed: in this, not in other.
    for (const auto& p : m_parts) {
        if (theirsById.find(p.id) == theirsById.end()) {
            PartDiff d;
            d.part_id = p.id;
            d.kind    = "removed";
            d.details = {
                { "from_center", { p.centerX(), p.centerY(), p.centerZ() } },
            };
            result.push_back(std::move(d));
        }
    }

    // Added: in other, not in this.
    for (const auto& p : other.m_parts) {
        if (mineById.find(p.id) == mineById.end()) {
            PartDiff d;
            d.part_id = p.id;
            d.kind    = "added";
            d.details = {
                { "to_center", { p.centerX(), p.centerY(), p.centerZ() } },
            };
            result.push_back(std::move(d));
        }
    }

    // Common: check for moves and/or resizes.  Iterate in *this's* order so
    // the output is deterministic.
    for (const auto& mineP : m_parts) {
        auto it = theirsById.find(mineP.id);
        if (it == theirsById.end()) continue;
        const Part& theirsP = *it->second;

        const double dx = theirsP.centerX() - mineP.centerX();
        const double dy = theirsP.centerY() - mineP.centerY();
        const double dz = theirsP.centerZ() - mineP.centerZ();
        const double translation = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (translation > kMovedTolMm) {
            PartDiff d;
            d.part_id = mineP.id;
            d.kind    = "moved";
            d.details = {
                { "from_center", { mineP.centerX(),   mineP.centerY(),   mineP.centerZ()   } },
                { "to_center",   { theirsP.centerX(), theirsP.centerY(), theirsP.centerZ() } },
                { "delta",       { dx, dy, dz } },
                { "magnitude",   translation },
            };
            result.push_back(std::move(d));
        }

        const double dsx = theirsP.sizeX() - mineP.sizeX();
        const double dsy = theirsP.sizeY() - mineP.sizeY();
        const double dsz = theirsP.sizeZ() - mineP.sizeZ();
        if (std::abs(dsx) > kSizeTolMm ||
            std::abs(dsy) > kSizeTolMm ||
            std::abs(dsz) > kSizeTolMm) {
            PartDiff d;
            d.part_id = mineP.id;
            d.kind    = "resized";
            d.details = {
                { "from_size", { mineP.sizeX(),   mineP.sizeY(),   mineP.sizeZ()   } },
                { "to_size",   { theirsP.sizeX(), theirsP.sizeY(), theirsP.sizeZ() } },
                { "delta",     { dsx, dsy, dsz } },
            };
            result.push_back(std::move(d));
        }
    }

    return result;
}

// ── JSON I/O ─────────────────────────────────────────────────────────────

json PartsLayout::toJson() const
{
    json out = {
        { "version", 1 },
        { "parts",   json::array() },
    };
    for (const auto& p : m_parts) {
        json part = {
            { "id",        p.id },
            { "step_path", p.step_path },
            { "aabb", {
                { "xMin", p.xMin }, { "yMin", p.yMin }, { "zMin", p.zMin },
                { "xMax", p.xMax }, { "yMax", p.yMax }, { "zMax", p.zMax },
            }},
            { "placement", trsfToJson(p.placement) },
        };
        out["parts"].push_back(part);
    }
    return out;
}

PartsLayout PartsLayout::fromJson(const json& j)
{
    PartsLayout layout;
    if (!j.contains("parts") || !j["parts"].is_array()) {
        spdlog::warn("PartsLayout::fromJson: missing or non-array 'parts'");
        return layout;
    }
    for (const auto& partJ : j["parts"]) {
        Part part;
        if (partJ.contains("id") && partJ["id"].is_string()) {
            part.id = partJ["id"].get<std::string>();
        }
        if (partJ.contains("step_path") && partJ["step_path"].is_string()) {
            part.step_path = partJ["step_path"].get<std::string>();
        }
        if (partJ.contains("aabb") && partJ["aabb"].is_object()) {
            const auto& a = partJ["aabb"];
            if (a.contains("xMin")) part.xMin = a["xMin"].get<double>();
            if (a.contains("yMin")) part.yMin = a["yMin"].get<double>();
            if (a.contains("zMin")) part.zMin = a["zMin"].get<double>();
            if (a.contains("xMax")) part.xMax = a["xMax"].get<double>();
            if (a.contains("yMax")) part.yMax = a["yMax"].get<double>();
            if (a.contains("zMax")) part.zMax = a["zMax"].get<double>();
        }
        if (partJ.contains("placement")) {
            part.placement = jsonToTrsf(partJ["placement"]);
        }
        // We intentionally do NOT call addPart() here, because addPart()
        // would try to recompute the AABB; we want the JSON-stored AABB
        // to be honoured verbatim.
        layout.m_parts.push_back(std::move(part));
    }
    return layout;
}

}  // namespace koocadcam::parts
