// @lat: [[process/test-strategy#skill round-trip]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/scan_pattern.hpp"

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

}  // namespace

// ── 1. Apply: grid pattern generates ~point_count probe points ───────────
TEST(SkillScanPattern, ApplyGeneratesGridPoints)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "grid";
    in.point_count      = 25;
    in.tolerance_mm     = 0.05;

    auto out = skill::scan_pattern::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    // Geometric no-op.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("scan_pattern"));
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_EQ(out.signature.pattern["pattern_type"].get<std::string>(),
              std::string("grid"));
    EXPECT_EQ(out.signature.pattern["actual_point_count"].get<int>(), 25);
    EXPECT_EQ(out.signature.pattern["points"].size(), 25u);
}

// ── 2. Apply: diagonal pattern generates exactly point_count points ──────
TEST(SkillScanPattern, ApplyGeneratesDiagonalPoints)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "diagonal";
    in.point_count      = 10;
    in.tolerance_mm     = 0.05;

    auto out = skill::scan_pattern::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["points"].size(), 10u);
    EXPECT_EQ(out.signature.pattern["pattern_type"].get<std::string>(),
              std::string("diagonal"));
}

// ── 3. DFM: unknown pattern rejected ─────────────────────────────────────
TEST(SkillScanPattern, ValidateRejectsUnknownPattern)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "zigzag";   // not supported
    in.point_count      = 25;
    in.tolerance_mm     = 0.05;

    auto r = skill::scan_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::scan_pattern::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-PROBE-AREA: tiny face rejected ────────────────────────────────
TEST(SkillScanPattern, ValidateRejectsTinyFace)
{
    auto stock = skill::createCuboidStock(5.0, 5.0, 10.0);   // top face = 25 mm² < 100
    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "grid";
    in.point_count      = 25;
    in.tolerance_mm     = 0.05;

    auto r = skill::scan_pattern::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PROBE-AREA") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize is always empty (metadata-only skill) ───────────────────
TEST(SkillScanPattern, RecognizeAlwaysEmpty)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto cands = skill::scan_pattern::recognize(*stock);
    EXPECT_TRUE(cands.empty());

    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "spiral";
    in.point_count      = 16;
    in.tolerance_mm     = 0.05;
    auto out = skill::scan_pattern::apply(*stock, in);
    EXPECT_TRUE(skill::scan_pattern::recognize(*out.workpiece).empty());
}

// ── 6. Probe points are on the entry face plane ──────────────────────────
TEST(SkillScanPattern, ProbePointsLieOnEntryFace)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "grid";
    in.point_count      = 9;
    in.tolerance_mm     = 0.05;

    auto out = skill::scan_pattern::apply(*stock, in);
    // Top face is z = 10; all probe Z coords should equal 10.
    for (const auto& p : out.signature.pattern["points"]) {
        EXPECT_NEAR(p[2].get<double>(), 10.0, 1e-6);
    }
}

// ── 7. Spiral pattern yields requested point_count exactly ───────────────
TEST(SkillScanPattern, SpiralYieldsExactCount)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    skill::scan_pattern::Input in;
    in.entry_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pattern          = "spiral";
    in.point_count      = 30;
    in.tolerance_mm     = 0.05;

    auto out = skill::scan_pattern::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["points"].size(), 30u);
}
