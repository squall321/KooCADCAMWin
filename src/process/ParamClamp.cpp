// @lat: [[engine/skills#param-clamp]]

#include "ParamClamp.hpp"

#include "skills/Workpiece.hpp"

#include <algorithm>

namespace koocadcam::process {

using nlohmann::json;

const std::unordered_map<std::string, ClampFn>& clampTable()
{
    static const std::unordered_map<std::string, ClampFn> table = [] {
        std::unordered_map<std::string, ClampFn> t;

        // hollow_cavity — a recovered wall_thickness / depth from a foreign STEP
        // can exceed the stock so the remaining cavity is non-positive, which
        // throws in apply() ("wall_thickness >= half of workpiece XY extent").
        // Clamp wall < 0.45 * min(XY extent) (strictly inside the 0.5 boundary)
        // and depth < Z span, honouring the 0.4 mm manufacturable wall floor.
        t["hollow_cavity"] = [](const json& p, const skill::Workpiece& stock) {
            json out = p;
            double xMin, yMin, zMin, xMax, yMax, zMax;
            stock.boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
            const double xyMin = std::min(xMax - xMin, yMax - yMin);
            const double zSpan = zMax - zMin;

            if (out.contains("wall_thickness_mm") &&
                out["wall_thickness_mm"].is_number()) {
                double w = out["wall_thickness_mm"].get<double>();
                const double wMax = 0.45 * xyMin;
                if (w > wMax) w = wMax;
                if (w < 0.4 && wMax >= 0.4) w = 0.4;     // DFM-001 floor
                out["wall_thickness_mm"] = w;
            }
            if (out.contains("depth_mm") && out["depth_mm"].is_number()) {
                double d = out["depth_mm"].get<double>();
                const double dMax = zSpan - 0.4;          // keep a bottom wall
                if (dMax > 0.0 && d > dMax) d = dMax;
                out["depth_mm"] = d;
            }
            return out;
        };

        return t;
    }();
    return table;
}

nlohmann::json clampParams(const std::string& skill_id,
                           const nlohmann::json& params,
                           const skill::Workpiece& stock)
{
    const auto& t = clampTable();
    const auto it = t.find(skill_id);
    if (it == t.end()) return params;
    try {
        return it->second(params, stock);
    } catch (...) {
        return params;     // a clamp must never make the step worse than before
    }
}

}  // namespace koocadcam::process
