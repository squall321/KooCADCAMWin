// @lat: [[engine/skills#intermittent_weld_bead]]

#include "intermittent_weld_bead.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace koocadcam::skill::intermittent_weld_bead {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

double mag3(const std::array<double, 3>& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;
    (void)wp;

    if (in.bead_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "intermittent_weld_bead: bead_length_mm must be > 0");
    }
    if (in.bead_count <= 0) {
        r.add("DFM-INPUT", "error",
              "intermittent_weld_bead: bead_count must be > 0 (got " +
              std::to_string(in.bead_count) + ")");
    }
    if (in.gap_length_mm < 0.0) {
        r.add("DFM-INPUT", "error",
              "intermittent_weld_bead: gap_length_mm must be ≥ 0");
    }
    if (in.bead_height_mm <= 0.0) {
        r.add("DFM-INPUT", "error",
              "intermittent_weld_bead: bead_height_mm must be > 0");
    }
    if (mag3(in.bead_direction_xyz) < 1e-9) {
        r.add("DFM-INPUT", "error",
              "intermittent_weld_bead: bead_direction_xyz must be non-zero");
    }
    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "intermittent_weld_bead DFM failed:";
        for (const auto& f : dfm.findings)
            if (f.severity == "error")
                msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const double dmag = mag3(in.bead_direction_xyz);
    const std::array<double, 3> dir = {
        in.bead_direction_xyz[0] / dmag,
        in.bead_direction_xyz[1] / dmag,
        in.bead_direction_xyz[2] / dmag,
    };
    const gp_Dir gpDir(dir[0], dir[1], dir[2]);

    const double w = in.bead_height_mm;   // bead width = height (square cross-section)
    const double h = in.bead_height_mm;
    const double bL = in.bead_length_mm;
    const double pitch = in.bead_length_mm + in.gap_length_mm;

    // Build each bead as a box placed at the start of its slot.
    std::vector<TopoDS_Shape> beads;
    beads.reserve(static_cast<std::size_t>(in.bead_count));

    for (int i = 0; i < in.bead_count; ++i) {
        const double off = i * pitch;
        // Origin of the bead box at one corner.  box(axis, dx, dy, dz) treats
        // axis.Direction() as +Z and uses dx as the X extent.
        // We want the bead length aligned with `dir`, so we use a frame where
        // local +X (DX) points along dir.
        //
        // Simpler approach: orient the box with axis along the
        // bead-direction unit vector and use it as Z; then DZ = bead_length.
        // The cross-section becomes DX × DY = w × h.
        const gp_Pnt corner(
            in.bead_start_xyz[0] + dir[0] * off - 0.5 * w,  // shift -w/2 in X
            in.bead_start_xyz[1] + dir[1] * off - 0.5 * h,  // shift -h/2 in Y
            in.bead_start_xyz[2] + dir[2] * off);
        // For general dir we use gp_Ax2(origin, dir) so the box extrudes
        // along `dir` (= local +Z).  DX & DY are in the plane perpendicular
        // to dir — this is approximate but is fine for the intermittent
        // bead use-case (typically dir aligned with an edge).
        try {
            const gp_Ax2 ax(corner, gpDir);
            TopoDS_Shape b = pr::box(ax, w, h, bL);
            beads.push_back(b);
        } catch (const Standard_Failure& ex) {
            spdlog::debug("intermittent_weld_bead: box[{}] failed — {}", i, ex.what());
        }
    }

    if (beads.empty()) {
        throw SkillError("intermittent_weld_bead: no beads could be built");
    }

    // Fuse all beads onto the workpiece.
    const double volBefore = volumeOf(wp.shape());
    TopoDS_Shape newShape;
    if (wp.shape().IsNull()) {
        newShape = pr::fuseMany(beads.front(),
                                std::vector<TopoDS_Shape>(beads.begin() + 1,
                                                          beads.end()));
    } else {
        try {
            newShape = pr::fuseMany(wp.shape(), beads);
        } catch (const Standard_Failure&) {
            BRep_Builder builder;
            TopoDS_Compound comp;
            builder.MakeCompound(comp);
            builder.Add(comp, wp.shape());
            for (const auto& b : beads) builder.Add(comp, b);
            newShape = comp;
        }
    }

    const double volAfter = volumeOf(newShape);
    const double added    = volAfter - volBefore;
    const double expected = w * h * bL * static_cast<double>(beads.size());

    json params = {
        { "bead_start_xyz",     { in.bead_start_xyz[0],
                                  in.bead_start_xyz[1],
                                  in.bead_start_xyz[2] } },
        { "bead_direction_xyz", { in.bead_direction_xyz[0],
                                  in.bead_direction_xyz[1],
                                  in.bead_direction_xyz[2] } },
        { "bead_length_mm",     in.bead_length_mm  },
        { "gap_length_mm",      in.gap_length_mm   },
        { "bead_height_mm",     in.bead_height_mm  },
        { "bead_count",         in.bead_count      },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "profile_kind",        "bead" },
        { "bead_count",          static_cast<int>(beads.size()) },
        { "profile_dim_mm",      in.bead_height_mm },
        { "path_length",         pitch * static_cast<double>(beads.size()) },
        { "derived_volume_mm3",  added },
        { "expected_volume_mm3", expected },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "intermittent_weld";
    tooling.stock_removed_mm3 = -expected;
    tooling.est_cycle_time_s  = std::max(2.0,
                                         static_cast<double>(beads.size()) * 4.0);

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    for (const auto& f : wp.features()) wpNew->addFeature(f);
    wpNew->addFeature(sig);

    spdlog::debug("skill::intermittent_weld_bead: n={} bL={} gap={} added={}",
                  static_cast<int>(beads.size()),
                  in.bead_length_mm, in.gap_length_mm, added);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = f.params;
        rf.confidence       = 0.95;
        rf.matched_geometry = f.pattern;
        out.push_back(rf);
    }
    if (!out.empty()) return out;

    // Geometric: small periodic bumps → many planar faces.  Coarse check.
    if (wp.shape().IsNull()) return out;
    int planar = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFacePlanar(i)) ++planar;
    }
    if (planar >= 10) {
        RecognizedFeature rf;
        rf.skill_id         = kSkillId;
        rf.recovered_params = nlohmann::json::object();
        rf.confidence       = 0.30;
        rf.matched_geometry = nlohmann::json{ { "planar_face_count", planar } };
        out.push_back(rf);
    }
    return out;
}

}  // namespace koocadcam::skill::intermittent_weld_bead
