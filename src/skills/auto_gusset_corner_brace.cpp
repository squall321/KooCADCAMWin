// @lat: [[engine/skills#auto_gusset_corner_brace]]

#include "auto_gusset_corner_brace.hpp"

#include "Workpiece.hpp"
#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace koocadcam::skill::auto_gusset_corner_brace {

namespace pr = koocadcam::engine::prim;
using nlohmann::json;

// ── Lookup table: ASME B31.3 gusset service classes ──────────────────────
namespace {

struct ServiceSpec {
    const char* key;
    double      tip_radius_mm;
    double      stress_factor_kt;   // theoretical Kt at tip
};

constexpr std::array<ServiceSpec, 3> kServiceTable {{
    { "light",  0.5, 1.4 },
    { "medium", 1.0, 1.2 },
    { "heavy",  2.0, 1.1 },
}};

const ServiceSpec* lookupService(const std::string& key)
{
    for (const auto& s : kServiceTable)
        if (key == s.key) return &s;
    return nullptr;
}

// Find the two faces sharing the given edge id.  Returns {-1, -1} if not
// exactly two faces are adjacent.
std::pair<int, int> facesSharingEdge(const Workpiece& wp, int edgeId)
{
    if (edgeId < 0 || edgeId >= wp.edgeCount()) return { -1, -1 };
    const TopoDS_Edge& target = wp.edge(edgeId);

    std::vector<int> shared;
    for (int f = 0; f < wp.faceCount(); ++f) {
        const TopoDS_Face& face = wp.face(f);
        for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
            if (exp.Current().IsSame(target)) {
                shared.push_back(f);
                break;
            }
        }
        if (shared.size() == 2) break;
    }
    if (shared.size() != 2) return { -1, -1 };
    return { shared[0], shared[1] };
}

}  // namespace

// ── Validation ───────────────────────────────────────────────────────────

DFMReport validate(const Workpiece& wp, const Input& in)
{
    DFMReport r;

    if (in.corner_edge_id < 0 || in.corner_edge_id >= wp.edgeCount()) {
        r.add("DFM-INPUT", "error",
              "corner_edge_id out of range (got " +
              std::to_string(in.corner_edge_id) +
              ", edges=" + std::to_string(wp.edgeCount()) + ")");
    }
    if (in.leg_length_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "leg_length_mm must be > 0");
    }
    if (in.gusset_thick_mm <= 0.0) {
        r.add("DFM-INPUT", "error", "gusset_thick_mm must be > 0");
    }
    const ServiceSpec* spec = lookupService(in.service_class);
    if (!spec) {
        r.add("DFM-GUSSET-SPEC", "error",
              "auto_gusset_corner_brace: unknown service_class '" +
              in.service_class + "' (supported: light/medium/heavy)");
        return r;
    }
    if (!r.passed) return r;

    auto [fA, fB] = facesSharingEdge(wp, in.corner_edge_id);
    if (fA < 0) {
        r.add("DFM-GUSSET-ANGLE", "error",
              "could not detect 2 faces sharing edge id " +
              std::to_string(in.corner_edge_id));
        return r;
    }

    // Face area check: leg_length must be < 0.5 × min(sqrt(area)).
    const double areaA = wp.faceArea(fA);
    const double areaB = wp.faceArea(fB);
    const double dimA  = std::sqrt(std::max(0.0, areaA));
    const double dimB  = std::sqrt(std::max(0.0, areaB));
    const double minDim = std::min(dimA, dimB);
    if (in.leg_length_mm > 0.5 * minDim) {
        r.add("DFM-GUSSET-LEG", "warning",
              "leg_length " + std::to_string(in.leg_length_mm) +
              " > 0.5 × min(face_dim) " + std::to_string(0.5 * minDim));
    }

    // Inter-face angle: gussets only useful when the two faces meet at
    // approximately 90° (between ~60° and ~100°).
    const gp_Dir nA = wp.faceNormal(fA);
    const gp_Dir nB = wp.faceNormal(fB);
    const double cosA = nA.X() * nB.X() + nA.Y() * nB.Y() + nA.Z() * nB.Z();
    const double interior = std::acos(std::clamp(cosA, -1.0, 1.0)) * 180.0 / M_PI;
    if (interior > 100.0) {
        r.add("DFM-GUSSET-ANGLE", "warning",
              "flanking faces meet at " + std::to_string(interior) +
              "° > 100°; gusset benefit limited");
    }

    return r;
}

// ── Synthesis ────────────────────────────────────────────────────────────

SkillOutput apply(const Workpiece& wp, const Input& in)
{
    DFMReport dfm = validate(wp, in);
    if (!dfm.passed) {
        std::string msg = "auto_gusset_corner_brace DFM failed:";
        for (const auto& f : dfm.findings) msg += "\n  - " + f.code + ": " + f.message;
        throw SkillError(msg);
    }

    const ServiceSpec* spec = lookupService(in.service_class);

    auto [fA, fB] = facesSharingEdge(wp, in.corner_edge_id);
    if (fA < 0) {
        throw SkillError("auto_gusset_corner_brace: could not resolve face pair");
    }

    // Use the corner edge midpoint as anchor.
    const gp_Pnt anchor = wp.edgeMidPoint(in.corner_edge_id);
    const gp_Dir nA = wp.faceNormal(fA);
    const gp_Dir nB = wp.faceNormal(fB);

    // Inward direction = -nA + -nB normalised — points into the corner pocket.
    gp_Vec dirIn(-(nA.X() + nB.X()) / 2.0,
                 -(nA.Y() + nB.Y()) / 2.0,
                 -(nA.Z() + nB.Z()) / 2.0);
    if (dirIn.Magnitude() < 1e-6) {
        // Fall back to a fixed direction; this won't happen for a real corner.
        dirIn = gp_Vec(0, 0, -1);
    }
    dirIn.Normalize();

    // Build the gusset as a square box of side leg_length, thickness =
    // gusset_thick_mm, placed at the corner.  This is a slice-1
    // approximation of the triangular wedge — the volume is then trimmed
    // by the surrounding solid via fuse().
    const double leg = in.leg_length_mm;
    const double t   = in.gusset_thick_mm;

    // gp_Ax2(origin, mainZ, xDir): mainZ = corner inward dir (DZ direction
    // along which thickness extends, DX = some perpendicular).
    gp_Dir mainZ(dirIn.X(), dirIn.Y(), dirIn.Z());
    gp_Dir xDir(1, 0, 0);
    // Ensure orthogonality.
    if (std::abs(xDir.Dot(mainZ)) > 0.95) xDir = gp_Dir(0, 1, 0);

    // Origin = anchor moved 1/2 leg back along edge and offset slightly.
    const gp_Pnt origin(
        anchor.X() - xDir.X() * leg / 2.0,
        anchor.Y() - xDir.Y() * leg / 2.0,
        anchor.Z() - xDir.Z() * leg / 2.0);

    const gp_Ax2 gusAx(origin, mainZ, xDir);
    // DX=leg (along edge), DY=leg (perpendicular into corner), DZ=t (out).
    const TopoDS_Shape gussetBox = pr::box(gusAx, leg, leg, t);

    // ── Sub-feature 2: weld fillet bead — small cylinder along corner edge.
    // Use the edge midpoint as the bead centre; bead axis = mainZ; radius
    // ≈ t/2, length = leg.  Fuse onto gusset before integrating.
    const gp_Pnt beadOrigin(
        anchor.X() - xDir.X() * leg / 2.0,
        anchor.Y() - xDir.Y() * leg / 2.0,
        anchor.Z() - xDir.Z() * leg / 2.0);
    const gp_Ax2 beadAx(beadOrigin, xDir);  // bead axis along the edge
    const TopoDS_Shape bead = pr::cylinder(beadAx, t * 0.4, leg);
    const TopoDS_Shape gussetWithBead = pr::fuse(gussetBox, bead);

    TopoDS_Shape afterFuse = pr::fuse(wp.shape(), gussetWithBead);

    // ── Sub-feature 3: tip relief cut — drill a small cylinder at the
    //   triangular tip to remove stress concentration.
    const double rRelief = spec->tip_radius_mm;
    const gp_Pnt tipPt(
        anchor.X() + dirIn.X() * leg * 0.8,
        anchor.Y() + dirIn.Y() * leg * 0.8,
        anchor.Z() + dirIn.Z() * leg * 0.8);
    const gp_Ax2 reliefAx(tipPt, xDir);
    const TopoDS_Shape reliefTool = pr::cylinder(reliefAx, rRelief, leg + 1.0);
    const TopoDS_Shape newShape = pr::cut(afterFuse, reliefTool);

    // ── Bookkeeping ─────────────────────────────────────────────────────
    const double vGusset = leg * leg * t;
    const double vBead   = M_PI * (t * 0.4) * (t * 0.4) * leg;
    const double vRelief = M_PI * rRelief * rRelief * (leg + 1.0);

    json params = {
        { "corner_edge_id",  in.corner_edge_id },
        { "leg_length_mm",   in.leg_length_mm },
        { "gusset_thick_mm", in.gusset_thick_mm },
        { "service_class",   in.service_class },
    };
    json pattern = {
        { "kind",              kSkillId },
        { "is_compound",       true },
        { "subfeature_count",  3 },
        { "service_class",     in.service_class },
        { "tip_radius_mm",     spec->tip_radius_mm },
        { "leg_length_mm",     leg },
        { "gusset_thick_mm",   t },
        { "auto_detected_faces", { fA, fB } },
        { "standard",          "ASME B31.3" },
    };

    ToolingMeta tooling;
    tooling.tool_type         = "weld;drill";
    tooling.tool_dia_mm       = 2.0 * rRelief;
    tooling.tool_material     = "carbide";
    tooling.stock_removed_mm3 = vRelief;
    tooling.est_cycle_time_s  = 30.0;
    tooling.extra = {
        { "gusset_volume_mm3",  vGusset },
        { "bead_volume_mm3",    vBead },
        { "relief_volume_mm3",  vRelief },
        { "added_volume_mm3",   vGusset + vBead },
        { "removed_volume_mm3", vRelief },
        { "net_delta_mm3",      (vGusset + vBead) - vRelief },
        { "stress_factor_kt",   spec->stress_factor_kt },
        { "standard",           "ASME B31.3" },
    };

    FeatureSignature sig{ kSkillId, params, pattern, tooling };

    auto wpNew = std::make_shared<Workpiece>(newShape, wp.material());
    wpNew->addFeature(sig);

    spdlog::debug("skill::auto_gusset_corner_brace: leg={} t={} svc={}",
                  leg, t, in.service_class);

    return SkillOutput{ wpNew, sig };
}

// ── Recognition ──────────────────────────────────────────────────────────

std::vector<RecognizedFeature> recognize(const Workpiece& wp)
{
    std::vector<RecognizedFeature> out;

    auto emitMetadataReplay = [&]() {
        for (const auto& f : wp.features()) {
            if (f.skill_id != kSkillId) continue;
            RecognizedFeature rf;
            rf.skill_id = kSkillId;
            rf.recovered_params = f.params;
            rf.confidence = 1.0;
            rf.matched_geometry = { { "via", "metadata_replay" } };
            out.push_back(rf);
            return;
        }
    };

    // Geometric heuristic: look for small triangular planar faces
    // (area < 200 mm² approximate) adjacent to two larger orthogonal
    // walls.  Slice-1 simplification: a planar face whose area is
    // 30–300 mm² flanked by 2 planar faces with ~90° normals.
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        double a = 0.0;
        try { a = wp.faceArea(i); } catch (...) { continue; }
        if (a < 20.0 || a > 400.0) continue;

        // Count adjacent planar faces with ~orthogonal normal.
        const gp_Dir nThis = wp.faceNormal(i);
        int orthCount = 0;
        for (int j = 0; j < wp.faceCount(); ++j) {
            if (i == j) continue;
            if (!wp.isFacePlanar(j)) continue;
            const gp_Dir nJ = wp.faceNormal(j);
            const double dot = std::abs(
                nThis.X() * nJ.X() + nThis.Y() * nJ.Y() + nThis.Z() * nJ.Z());
            if (dot < 0.3) ++orthCount;
        }
        if (orthCount < 2) continue;

        const gp_Pnt c = wp.faceCenter(i);

        json recovered = {
            { "corner_edge_id",  0 },     // can't recover precisely
            { "leg_length_mm",   std::sqrt(a) },
            { "gusset_thick_mm", 3.0 },
            { "service_class",   "medium" },
        };
        json matched = {
            { "via",      "geometric_primary" },
            { "face_idx", i },
            { "centre",   { c.X(), c.Y(), c.Z() } },
        };
        out.push_back(RecognizedFeature{ kSkillId, recovered, 0.55, matched });
    }

    if (out.empty()) emitMetadataReplay();
    return out;
}

}  // namespace koocadcam::skill::auto_gusset_corner_brace
