// @lat: [[process/test-strategy#skill round-trip]]
//
// ecm — Electrochemical Machining skill round-trip tests.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ecm.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply removes material in a cylinder + bottom fillet ──────────────
TEST(SkillEcm, ApplyRemovesMaterial)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);

    skill::ecm::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 10.0;
    in.depth_mm      = 5.0;
    in.corner_r_mm   = 0.8;

    auto out = skill::ecm::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // The fillet shaves a small wedge off the simple π r² × depth box; allow
    // 10 % slack to absorb the toroidal correction.
    const double naive = M_PI * 5.0 * 5.0 * 5.0;
    const double removed = volumeOf(stock->shape()) - volumeOf(out.workpiece->shape());
    EXPECT_GT(removed, naive * 0.85);
    EXPECT_LT(removed, naive * 1.05);
}

// ── 2. Validate rejects bad input (corner_r >= diameter/2) ───────────────
TEST(SkillEcm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);

    skill::ecm::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 5.0;
    in.depth_mm      = 3.0;
    in.corner_r_mm   = 3.0;   // > diameter/2 (= 2.5)

    auto r = skill::ecm::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ECM-CORNER" && f.severity == "error") {
            found = true; break;
        }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::ecm::apply(*stock, in), skill::SkillError);
}

// ── 3. Signature records kind + tool_type ────────────────────────────────
TEST(SkillEcm, SignatureRecordsKindAndToolType)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);

    skill::ecm::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 8.0;
    in.depth_mm      = 4.0;
    in.corner_r_mm   = 0.5;

    auto out = skill::ecm::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("ecm"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ecm"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("ecm_cathode"));
    EXPECT_EQ(out.signature.pattern["toroidal_face_count"].get<int>(), 1);
}

// ── 4. Recognize replays metadata only ───────────────────────────────────
TEST(SkillEcm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 15.0);

    skill::ecm::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30.0;
    in.position_y_mm = 30.0;
    in.diameter_mm   = 8.0;
    in.depth_mm      = 4.0;
    in.corner_r_mm   = 0.5;

    auto out = skill::ecm::apply(*stock, in);
    auto cands = skill::ecm::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.9);
    EXPECT_NEAR(cands[0].recovered_params["diameter_mm"].get<double>(), 8.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["corner_r_mm"].get<double>(), 0.5, 1e-6);
}

// ── 5. Specific: depth > 25 mm emits the ECM-depth warning ───────────────
TEST(SkillEcm, ValidateWarnsOnDeepCavity)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 40.0);

    skill::ecm::Input in;
    in.entry_face    = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 40.0;
    in.position_y_mm = 40.0;
    in.diameter_mm   = 10.0;
    in.depth_mm      = 30.0;     // > 25 → warning
    in.corner_r_mm   = 0.5;

    auto r = skill::ecm::validate(*stock, in);
    EXPECT_TRUE(r.passed);   // warnings don't fail validation
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-ECM-DEPTH" && f.severity == "warning") {
            found = true; break;
        }
    EXPECT_TRUE(found);
}
