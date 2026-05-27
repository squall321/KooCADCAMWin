// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/chamfer_edge.hpp"
#include "skills/face_milling_5ax.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// Build a wedge-shaped workpiece by chamfering one top rim of a cuboid.
// Result has an angled planar face whose normal is NOT along any global
// axis — ideal entry face for face_milling_5ax.
std::shared_ptr<skill::Workpiece> makeAngledStock()
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 20.0);

    skill::chamfer_edge::Input in;
    in.edge_selector  = skill::chamfer_edge::EdgesAtZBand{ 20.0, 1e-3 };
    in.chamfer_size_mm = 8.0;          // big chamfer for a clear angled face
    in.angle_deg       = 45.0;

    auto out = skill::chamfer_edge::apply(*stock, in);
    return out.workpiece;
}

}  // namespace

// ── 1. Apply: skim removes material on axis-aligned face (3-axis case) ───
TEST(SkillFaceMilling5ax, ApplyOnAxisAlignedFace)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::face_milling_5ax::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 1.0;

    auto out = skill::face_milling_5ax::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double removed = volBefore - volumeOf(out.workpiece->shape());
    EXPECT_NEAR(removed, 50.0 * 50.0 * 1.0, 2500.0 * 0.05);

    EXPECT_EQ(out.signature.skill_id, std::string("face_milling_5ax"));
    EXPECT_TRUE(out.signature.pattern["is_axis_aligned"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["requires_5axis"].get<bool>());
}

// ── 2. Apply: skim removes material on an ANGLED face (5-axis case) ──────
TEST(SkillFaceMilling5ax, ApplyOnAngledFace)
{
    auto stock = makeAngledStock();
    const double volBefore = volumeOf(stock->shape());

    skill::face_milling_5ax::Input in;
    // The chamfer face's outward normal is roughly (0, ?, ?) — at 45° to
    // both Y+ and Z+.  We target it by FaceByNormal aimed midway.
    in.entry_face = skill::FaceByNormal{
        gp_Dir(0.0, std::sqrt(0.5), std::sqrt(0.5)), 10.0, "largest" };
    in.depth_mm = 0.5;

    auto out = skill::face_milling_5ax::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_LT(volumeOf(out.workpiece->shape()), volBefore);

    EXPECT_FALSE(out.signature.pattern["is_axis_aligned"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["requires_5axis"].get<bool>());
}

// ── 3. Validate: zero depth rejected ─────────────────────────────────────
TEST(SkillFaceMilling5ax, ValidateRejectsZeroDepth)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::face_milling_5ax::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 0.0;

    auto r = skill::face_milling_5ax::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::face_milling_5ax::apply(*stock, in), skill::SkillError);
}

// ── 4. Validate: DFM-005 info always emitted ─────────────────────────────
TEST(SkillFaceMilling5ax, ValidateEmitsDFM005Info)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::face_milling_5ax::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.depth_mm   = 1.0;

    auto r = skill::face_milling_5ax::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // info findings don't fail validation
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-005" && f.severity == "info") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: finds the angled face but skips axis-aligned ones ──────
TEST(SkillFaceMilling5ax, RecognizeFindsAngledFace)
{
    auto stock = makeAngledStock();
    auto cands = skill::face_milling_5ax::recognize(*stock);
    ASSERT_FALSE(cands.empty());
    // All candidates must have non-axis-aligned normals.
    for (const auto& c : cands) {
        const auto& n = c.recovered_params["entry_face_normal"];
        const double ax = std::abs(n[0].get<double>());
        const double ay = std::abs(n[1].get<double>());
        const double az = std::abs(n[2].get<double>());
        EXPECT_FALSE(ax > 0.99 || ay > 0.99 || az > 0.99)
            << "axis-aligned face should have been skipped";
    }
}
