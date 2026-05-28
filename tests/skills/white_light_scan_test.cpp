// @lat: [[process/test-strategy#skill round-trip]]
//
// white_light_scan — INSPECTION skill: non-contact interferometry.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/white_light_scan.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Dir.hxx>

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

// ── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillWhiteLightScan, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::white_light_scan::Input in;
    in.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.point_density_per_mm2 = 100.0;
    in.measured_rms_nm       = 25.0;
    in.rms_tolerance_nm      = 50.0;

    auto out = skill::white_light_scan::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: bad density / tolerance / negative meas ──
TEST(SkillWhiteLightScan, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::white_light_scan::Input badDensity;
    badDensity.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badDensity.point_density_per_mm2 = 0.0;
    badDensity.rms_tolerance_nm      = 50.0;
    auto r1 = skill::white_light_scan::validate(*stock, badDensity);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::white_light_scan::apply(*stock, badDensity),
                 skill::SkillError);

    skill::white_light_scan::Input badTol;
    badTol.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badTol.point_density_per_mm2 = 100.0;
    badTol.rms_tolerance_nm      = 0.0;
    auto r2 = skill::white_light_scan::validate(*stock, badTol);
    EXPECT_FALSE(r2.passed);

    skill::white_light_scan::Input badMeas;
    badMeas.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badMeas.point_density_per_mm2 = 100.0;
    badMeas.rms_tolerance_nm      = 50.0;
    badMeas.measured_rms_nm       = -1.0;
    auto r3 = skill::white_light_scan::validate(*stock, badMeas);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillWhiteLightScan, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::white_light_scan::Input in;
    in.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.point_density_per_mm2 = 200.0;
    in.measured_rms_nm       = 30.0;
    in.rms_tolerance_nm      = 50.0;

    auto out = skill::white_light_scan::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("white_light_scan"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("white_light_scan"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("white_light_interferometer"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillWhiteLightScan, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::white_light_scan::Input in;
    in.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.point_density_per_mm2 = 50.0;
    in.rms_tolerance_nm      = 50.0;
    auto out = skill::white_light_scan::apply(*stock, in);

    auto cands = skill::white_light_scan::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("white_light_scan"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::white_light_scan::recognize(raw).empty());
}

// ── 5. Area + estimated point count roundtrip through pattern ────────────
TEST(SkillWhiteLightScan, AreaAndPointCountRoundtrip)
{
    // 50 × 50 cuboid top face = 2500 mm² × 100 pt/mm² = 250 000 pts.
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::white_light_scan::Input in;
    in.target_face_datum     = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.point_density_per_mm2 = 100.0;
    in.measured_rms_nm       = 40.0;
    in.rms_tolerance_nm      = 50.0;

    auto out = skill::white_light_scan::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["area_mm2"].get<double>(), 2500.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["point_density_per_mm2"].get<double>(),
                100.0, 1e-12);
    EXPECT_EQ(out.signature.pattern["estimated_point_count"].get<long long>(),
              250000LL);
    EXPECT_TRUE(out.signature.pattern["conforms"].get<bool>());
    EXPECT_NEAR(out.signature.pattern["measured_rms_nm"].get<double>(),
                40.0, 1e-12);
}
