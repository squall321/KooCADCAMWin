// @lat: [[engine/skills#boolean_morph_chain]]

#include "boolean_morph_chain.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill::boolean_morph_chain {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── helpers ──────────────────────────────────────────────────────────────

namespace {

const char* shapeKindStr(ShapeKind k)
{
    switch (k) {
        case ShapeKind::Cylinder: return "cylinder";
        case ShapeKind::Box:      return "box";
        case ShapeKind::Cone:     return "cone";
    }
    return "cylinder";
}

const char* opKindStr(OpKind k)
{
    switch (k) {
        case OpKind::Cut:  return "cut";
        case OpKind::Fuse: return "fuse";
    }
    return "cut";
}

double volumeOf(const TopoDS_Shape& s)
{
    if (s.IsNull()) return 0.0;
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

TopoDS_Shape buildToolShape(const Op& op)
{
    // For Box, axis is the LOCAL Z direction (matches BRepPrimAPI_MakeBox(Ax2, dx, dy, dz)).
    // For Cylinder/Cone, axis is the cylinder/cone axis.  We always pick a
    // perpendicular Vx so gp_Ax2 is well-defined.
    const gp_Dir& Z = op.axis;
    gp_Dir Vx;
    {
        const gp_Dir seed =
            (std::abs(Z.Z()) < 0.9) ? gp_Dir(0, 0, 1) : gp_Dir(1, 0, 0);
        gp_Vec vx = gp_Vec(seed).Crossed(gp_Vec(Z));
        if (vx.Magnitude() < 1e-9) vx = gp_Vec(1, 0, 0);
        Vx = gp_Dir(vx);
    }
    const gp_Ax2 ax(op.origin, Z, Vx);

    switch (op.shape) {
        case ShapeKind::Cylinder:
            return pr::cylinder(ax, op.size_a, op.size_b);
        case ShapeKind::Box:
            return pr::box(ax, op.size_a, op.size_b, op.size_c);
        case ShapeKind::Cone:
            return pr::coneFrustum(ax, op.size_a, op.size_b, op.size_c);
    }
    throw Standard_Failure("boolean_morph_chain: unknown ShapeKind");
}

}  // namespace

// ── validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.operations.empty()) {
        r.add("DFM-INPUT", "error",
              "boolean_morph_chain: operations sequence is empty "
              "(no compound morph to perform)");
        return r;
    }
    if (in.operations.size() < 2) {
        r.add("DFM-INPUT", "error",
              "boolean_morph_chain: operations.size() < 2 — chain must have "
              "at least 2 ops to qualify as a compound morph");
    }
    for (size_t i = 0; i < in.operations.size(); ++i) {
        const Op& op = in.operations[i];
        switch (op.shape) {
            case ShapeKind::Cylinder:
                if (op.size_a <= 0.0 || op.size_b <= 0.0) {
                    r.add("DFM-INPUT", "error",
                          "boolean_morph_chain: op[" + std::to_string(i) +
                          "] cylinder needs radius>0 and height>0");
                }
                break;
            case ShapeKind::Box:
                if (op.size_a <= 0.0 || op.size_b <= 0.0 || op.size_c <= 0.0) {
                    r.add("DFM-INPUT", "error",
                          "boolean_morph_chain: op[" + std::to_string(i) +
                          "] box needs dx,dy,dz > 0");
                }
                break;
            case ShapeKind::Cone:
                if (op.size_a < 0.0 || op.size_b < 0.0 || op.size_c <= 0.0) {
                    r.add("DFM-INPUT", "error",
                          "boolean_morph_chain: op[" + std::to_string(i) +
                          "] cone needs r1,r2 ≥ 0 and h > 0");
                }
                break;
        }
    }
    (void)wp;
    return r;
}

// ── synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "boolean_morph_chain DFM failed:";
        for (const auto& f : dfm.findings)
            msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    TopoDS_Shape current = wp.shape();
    const double startVol = volumeOf(current);

    json opChain = json::array();
    double cumulativeRemoved = 0.0;
    double cumulativeAdded   = 0.0;

    for (size_t i = 0; i < in.operations.size(); ++i) {
        const Op& op = in.operations[i];
        TopoDS_Shape tool;
        try {
            tool = buildToolShape(op);
        } catch (const Standard_Failure& ex) {
            throw SkillError("boolean_morph_chain: op[" + std::to_string(i) +
                             "] tool build failed: " + ex.what());
        }
        const double volBefore = volumeOf(current);
        try {
            if (op.op == OpKind::Cut) {
                current = pr::cut(current, tool);
            } else {
                current = pr::fuse(current, tool);
            }
        } catch (const Standard_Failure& ex) {
            throw SkillError("boolean_morph_chain: op[" + std::to_string(i) +
                             "] Boolean failed: " + ex.what());
        }
        const double volAfter = volumeOf(current);
        if (volAfter < volBefore) cumulativeRemoved += (volBefore - volAfter);
        else                      cumulativeAdded   += (volAfter - volBefore);

        opChain.push_back({
            { "index",     i },
            { "op",        opKindStr(op.op) },
            { "shape",     shapeKindStr(op.shape) },
            { "origin",    { op.origin.X(), op.origin.Y(), op.origin.Z() } },
            { "axis",      { op.axis.X(),   op.axis.Y(),   op.axis.Z() } },
            { "size_a",    op.size_a },
            { "size_b",    op.size_b },
            { "size_c",    op.size_c },
            { "vol_delta", volAfter - volBefore },
        });
    }

    const double endVol = volumeOf(current);
    if (endVol < 1e-6) {
        throw SkillError("boolean_morph_chain DFM (post-build) failed:\n"
                         "  - DFM-CHAIN-VOLUME: chain consumed all material "
                         "(final volume ≈ 0)");
    }

    json params = {
        { "operation_count", static_cast<int>(in.operations.size()) },
        { "operations",      opChain },
    };
    json pattern = {
        { "kind",                kSkillId },
        { "is_compound",         true },
        { "subfeature_count",    static_cast<int>(in.operations.size()) },
        { "start_volume_mm3",    startVol },
        { "end_volume_mm3",      endVol },
        { "volume_removed_mm3",  cumulativeRemoved },
        { "volume_added_mm3",    cumulativeAdded },
    };
    ToolingMeta tooling;
    tooling.tool_type         = "morph_chain";
    tooling.tool_dia_mm       = 0.0;
    tooling.tool_length_mm    = 0.0;
    tooling.tool_material     = "varies";
    tooling.flute_count       = 0;
    tooling.cutting_speed_sfm = 0.0;
    tooling.feed_per_tooth_mm = 0.0;
    tooling.stock_removed_mm3 = cumulativeRemoved;
    tooling.est_cycle_time_s  = std::max(1.0,
        static_cast<double>(in.operations.size()) * 2.0);
    tooling.extra = {
        { "chain_signature",
          "morph_v1:" + std::to_string(in.operations.size()) + "ops" },
        { "op_summary", opChain },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(current, wp.material());
    wpNew->setFeatures(wp.features());
    wpNew->addFeature(sig);

    spdlog::debug("skill::boolean_morph_chain applied: {} ops, removed={:.2f}, added={:.2f}",
                  in.operations.size(), cumulativeRemoved, cumulativeAdded);

    return SkillOutput{ wpNew, sig };
}

// ── recognition ──────────────────────────────────────────────────────────
//
// A chain leaves an arbitrary fingerprint that cannot be reverse-engineered
// without history.  Primary path: feature-history replay (1.0).  Geometric
// fallback: count obviously non-monolithic topological subdivisions and
// emit a single low-confidence candidate.

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;
    for (const auto& f : wp.features()) {
        if (f.skill_id != kSkillId) continue;
        json rp = {
            { "operation_count",
              f.params.value("operation_count", 0) },
            { "operations",
              f.params.value("operations", json::array()) },
        };
        out.push_back(RecognizedFeature{
            kSkillId, rp, 1.0,
            json{ { "source", "feature_history" } }
        });
    }
    if (!out.empty()) return out;

    // Geometric fallback: many cylindrical / planar non-bbox faces hints at
    // a compound chain.  Confidence stays low to avoid stomping more
    // specific recognizers.
    int cyls = 0, planes = 0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (wp.isFaceCylinder(i)) ++cyls;
        else if (wp.isFacePlanar(i)) ++planes;
    }
    if (cyls + planes >= 12) {  // a non-trivial multi-feature workpiece
        json rp = {
            { "operation_count", cyls + planes / 4 },
            { "operations",      json::array() },
        };
        RecognizedFeature r;
        r.skill_id = kSkillId;
        r.recovered_params = rp;
        r.confidence = 0.55;
        r.matched_geometry = json{
            { "cylindrical_face_count", cyls },
            { "planar_face_count",      planes },
            { "note", "geometric chain guess; no history available" },
        };
        out.push_back(std::move(r));
    }
    return out;
}

}  // namespace koocadcam::skill::boolean_morph_chain
