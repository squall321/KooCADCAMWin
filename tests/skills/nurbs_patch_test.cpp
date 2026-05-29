// @lat: [[process/test-strategy#skill round-trip]]
//
// nurbs_patch — surface patch built from a 4×4 control-point grid.
//
// Cases (6):
//   1. ApplyProducesNonNull shape.
//   2. ValidateRejectsBadInput — fewer than 4×4 control points.
//   3. SignatureRecordsKind — pattern.kind == "nurbs_patch".
//   4. RecognizeMetadataReplay — history replay confidence 1.0.
//   5. ValidateRejectsLowDegree — degree < 2.
//   6. ApplyAddsBSplineSurfaceFace — new shape contains a B-spline face.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/nurbs_patch.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

std::vector<std::vector<gp_Pnt>> makeFlat4x4(double z0)
{
    // A 4×4 grid of points on a slightly curved bump above z = z0.
    std::vector<std::vector<gp_Pnt>> g(4, std::vector<gp_Pnt>(4));
    for (int u = 0; u < 4; ++u) {
        for (int v = 0; v < 4; ++v) {
            const double x = 5.0 + 4.0 * u;
            const double y = 5.0 + 4.0 * v;
            // Gentle hump in the middle so the surface is not degenerate.
            const double bump = 0.2 *
                std::sin(3.14159 * u / 3.0) *
                std::sin(3.14159 * v / 3.0);
            g[u][v] = gp_Pnt(x, y, z0 + bump);
        }
    }
    return g;
}

}  // namespace

// ─── 1. ApplyProducesNonNull shape ────────────────────────────────────────
// Fixed in slice-9: GeomAPI_PointsToBSplineSurface with GeomAbs_C2 needs
// DegMax >= 2*Order+1 = 5; apply() now floors DegMin at 3 and lifts DegMax
// to 8 so the fit converges instead of throwing "WorkDegree too small".
TEST(SkillNurbsPatch, ApplyProducesNonNull)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);

    skill::nurbs_patch::Input in;
    in.control_grid = makeFlat4x4(10.0);
    in.degree_u = 3;
    in.degree_v = 3;

    auto out = skill::nurbs_patch::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput (too few control points) ──────────────────
TEST(SkillNurbsPatch, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);

    skill::nurbs_patch::Input in;
    // 2×2 grid is below the minimum of 4×4.
    in.control_grid = {
        { gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0) },
        { gp_Pnt(0, 10, 0), gp_Pnt(10, 10, 0) },
    };
    in.degree_u = 3;
    in.degree_v = 3;

    auto r = skill::nurbs_patch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::nurbs_patch::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind ──────────────────────────────────────────────
TEST(SkillNurbsPatch, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);

    skill::nurbs_patch::Input in;
    in.control_grid = makeFlat4x4(10.0);

    auto out = skill::nurbs_patch::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("nurbs_patch"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("nurbs_patch"));
    EXPECT_EQ(out.signature.pattern["nu"].get<int>(), 4);
    EXPECT_EQ(out.signature.pattern["nv"].get<int>(), 4);
}

// ─── 4. RecognizeMetadataReplay (confidence 1.0) ──────────────────────────
TEST(SkillNurbsPatch, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);

    skill::nurbs_patch::Input in;
    in.control_grid = makeFlat4x4(10.0);

    auto out = skill::nurbs_patch::apply(*stock, in);
    auto cands = skill::nurbs_patch::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("nurbs_patch"));
    EXPECT_DOUBLE_EQ(c.confidence, 1.0);
    EXPECT_EQ(c.recovered_params["nu"].get<int>(), 4);
    EXPECT_EQ(c.recovered_params["degree_u"].get<int>(), 3);
}

// ─── 5. ValidateRejectsLowDegree ──────────────────────────────────────────
TEST(SkillNurbsPatch, ValidateRejectsLowDegree)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);

    skill::nurbs_patch::Input in;
    in.control_grid = makeFlat4x4(10.0);
    in.degree_u = 1;     // below the min of 2
    in.degree_v = 3;

    auto r = skill::nurbs_patch::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-NURBS-DEG") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);
}

// ─── 6. ApplyAddsBSplineSurfaceFace ───────────────────────────────────────
TEST(SkillNurbsPatch, ApplyAddsBSplineSurfaceFace)
{
    auto stock = skill::createCuboidStock(30.0, 30.0, 5.0);

    skill::nurbs_patch::Input in;
    in.control_grid = makeFlat4x4(10.0);

    auto out = skill::nurbs_patch::apply(*stock, in);

    bool sawBSplineFace = false;
    for (int i = 0; i < out.workpiece->faceCount(); ++i) {
        BRepAdaptor_Surface s(out.workpiece->face(i));
        if (s.GetType() == GeomAbs_BSplineSurface ||
            s.GetType() == GeomAbs_BezierSurface) {
            sawBSplineFace = true;
            break;
        }
    }
    EXPECT_TRUE(sawBSplineFace) << "expected a free-form surface face after apply";
}
