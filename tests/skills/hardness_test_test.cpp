// @lat: [[process/test-strategy#skill round-trip]]
//
// hardness_test — INSPECTION skill: single-point indentation hardness
// (Rockwell / Vickers / Brinell).  Metadata-only.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hardness_test.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Dir.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

}  // namespace

// ── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillHardnessTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");
    const double volBefore = volumeOf(stock->shape());

    skill::hardness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.scale              = "HRC";
    in.indenter_type      = "diamond_brale";
    in.load_kgf           = 150.0;
    in.measured_hardness  = 55.0;

    auto out = skill::hardness_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    // Design choice locked: single-point, not face-average.
    EXPECT_EQ(out.signature.pattern["measurement_mode"].get<std::string>(),
              std::string("single_point_indentation"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: bad scale / indenter / load / measured ───
TEST(SkillHardnessTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::hardness_test::Input badScale;
    badScale.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badScale.scale             = "HRG";    // not in {HRC, HRB, HV, HB}
    badScale.indenter_type     = "diamond_brale";
    badScale.load_kgf          = 150.0;
    badScale.measured_hardness = 0.0;
    auto r1 = skill::hardness_test::validate(*stock, badScale);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::hardness_test::apply(*stock, badScale),
                 skill::SkillError);

    skill::hardness_test::Input badInd = badScale;
    badInd.scale         = "HRC";
    badInd.indenter_type = "wood";          // not allowed
    auto r2 = skill::hardness_test::validate(*stock, badInd);
    EXPECT_FALSE(r2.passed);

    skill::hardness_test::Input badLoad = badScale;
    badLoad.scale    = "HRC";
    badLoad.load_kgf = 0.0;
    auto r3 = skill::hardness_test::validate(*stock, badLoad);
    EXPECT_FALSE(r3.passed);

    skill::hardness_test::Input badMeas = badScale;
    badMeas.scale             = "HRC";
    badMeas.measured_hardness = -5.0;      // physically impossible
    auto r4 = skill::hardness_test::validate(*stock, badMeas);
    EXPECT_FALSE(r4.passed);
}

// ── 3. Scale ↔ indenter mismatch → info only (lab practice tolerated) ────
TEST(SkillHardnessTest, ScaleIndenterMismatchIsInfoOnly)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::hardness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.scale              = "HRC";              // expects diamond_brale
    in.indenter_type      = "tungsten_carbide_ball";  // mismatched
    in.load_kgf           = 150.0;
    in.measured_hardness  = 50.0;

    auto r = skill::hardness_test::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // mismatch is info-only
    EXPECT_TRUE(hasFinding(r, "DFM-HARD-MATCH"));
    EXPECT_NO_THROW(skill::hardness_test::apply(*stock, in));
}

// ── 4. Signature records all hardness parameters + Vickers tooling ───────
TEST(SkillHardnessTest, SignatureRecordsParameters)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::hardness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.scale              = "HV";
    in.indenter_type      = "diamond_pyramid";
    in.load_kgf           = 10.0;
    in.measured_hardness  = 220.0;

    auto out = skill::hardness_test::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("hardness_test"));
    EXPECT_EQ(out.signature.pattern["scale"].get<std::string>(),
              std::string("HV"));
    EXPECT_EQ(out.signature.pattern["indenter_type"].get<std::string>(),
              std::string("diamond_pyramid"));
    EXPECT_NEAR(out.signature.pattern["load_kgf"].get<double>(), 10.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["measured_hardness"].get<double>(),
                220.0, 1e-9);
    EXPECT_TRUE(out.signature.pattern["has_measurement"].get<bool>());
    EXPECT_EQ(out.signature.pattern["ndt_category"].get<std::string>(),
              std::string("hardness"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("hardness_tester"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("diamond"));
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 5. Recognize metadata replay round-trip ──────────────────────────────
TEST(SkillHardnessTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::hardness_test::Input in;
    in.target_face_datum  = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.scale              = "HB";
    in.indenter_type      = "tungsten_carbide_ball";
    in.load_kgf           = 3000.0;
    in.measured_hardness  = 180.0;

    auto out = skill::hardness_test::apply(*stock, in);
    auto cands = skill::hardness_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["scale"].get<std::string>(),
              std::string("HB"));
    EXPECT_NEAR(cands[0].recovered_params["measured_hardness"].get<double>(),
                180.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::hardness_test::recognize(raw).empty());
}
