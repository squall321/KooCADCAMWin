// @lat: [[process/test-strategy#skill round-trip]]
//
// surface_roughness_test — INSPECTION skill: Ra / Rz stylus measurement.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/surface_roughness_test.hpp"

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
TEST(SkillSurfaceRoughnessTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore = volumeOf(stock->shape());

    skill::surface_roughness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_ra_um       = 1.6;
    in.measured_ra_um     = 1.2;
    in.cutoff_length_mm   = 0.8;

    auto out = skill::surface_roughness_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ── 2. ValidateRejectsBadInput: zero/negative target_ra / cutoff ─────────
TEST(SkillSurfaceRoughnessTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::surface_roughness_test::Input badRa;
    badRa.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badRa.target_ra_um       = 0.0;
    badRa.cutoff_length_mm   = 0.8;
    auto r1 = skill::surface_roughness_test::validate(*stock, badRa);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::surface_roughness_test::apply(*stock, badRa),
                 skill::SkillError);

    skill::surface_roughness_test::Input badCut;
    badCut.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badCut.target_ra_um       = 1.6;
    badCut.cutoff_length_mm   = 0.0;
    auto r2 = skill::surface_roughness_test::validate(*stock, badCut);
    EXPECT_FALSE(r2.passed);

    skill::surface_roughness_test::Input badMeas;
    badMeas.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badMeas.target_ra_um       = 1.6;
    badMeas.measured_ra_um     = -0.5;   // negative impossible
    badMeas.cutoff_length_mm   = 0.8;
    auto r3 = skill::surface_roughness_test::validate(*stock, badMeas);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Signature records kind + tooling metadata ─────────────────────────
TEST(SkillSurfaceRoughnessTest, SignatureRecordsKindAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::surface_roughness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_ra_um       = 0.8;
    in.measured_ra_um     = 0.7;
    in.cutoff_length_mm   = 0.8;

    auto out = skill::surface_roughness_test::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("surface_roughness_test"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("surface_roughness_test"));
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("profilometer_stylus"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("diamond"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillSurfaceRoughnessTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::surface_roughness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.target_ra_um       = 1.6;
    in.measured_ra_um     = 1.4;
    in.cutoff_length_mm   = 0.8;
    auto out = skill::surface_roughness_test::apply(*stock, in);

    auto cands = skill::surface_roughness_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("surface_roughness_test"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::surface_roughness_test::recognize(raw).empty());
}

// ── 5. target/measured Ra + conformance bool round-trip ──────────────────
TEST(SkillSurfaceRoughnessTest, RaAndConformanceRoundtrip)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // measured < target → conforms
    skill::surface_roughness_test::Input ok;
    ok.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ok.target_ra_um       = 0.8;
    ok.measured_ra_um     = 0.5;
    ok.cutoff_length_mm   = 0.8;
    auto outOk = skill::surface_roughness_test::apply(*stock, ok);
    EXPECT_NEAR(outOk.signature.pattern["target_ra_um"].get<double>(),
                0.8, 1e-12);
    EXPECT_NEAR(outOk.signature.pattern["measured_ra_um"].get<double>(),
                0.5, 1e-12);
    EXPECT_TRUE(outOk.signature.pattern["conformance"].get<bool>());
    EXPECT_TRUE(outOk.signature.pattern["has_measurement"].get<bool>());

    // measured > target → does NOT conform (still valid — warning only)
    skill::surface_roughness_test::Input bad;
    bad.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    bad.target_ra_um       = 0.8;
    bad.measured_ra_um     = 1.5;
    bad.cutoff_length_mm   = 0.8;
    auto outBad = skill::surface_roughness_test::apply(*stock, bad);
    EXPECT_FALSE(outBad.signature.pattern["conformance"].get<bool>());
}
