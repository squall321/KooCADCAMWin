// @lat: [[process/test-strategy#medical bone screw pilot compound]]
//
// bone_screw_pilot_compound — pilot + countersink + chamfer compound.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bone_screw_pilot_compound.hpp"

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

// ─── 1. apply produces valid compound shape ──────────────────────────────
TEST(SkillBoneScrewPilot, ApplyProducesValidCompoundShape)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double stockVol = volumeOf(stock->shape());
    const int    stockFC  = stock->faceCount();

    skill::bone_screw_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1), 5.0, "largest" };
    in.position_x_mm      = 20.0;
    in.position_y_mm      = 20.0;
    in.axis_dir           = gp_Dir(0, 0, -1);
    in.screw_size_id      = "3.5";    // AO/ASIF cortical 3.5 → pilot 2.5 mm
    in.depth_mm           = 8.0;
    in.countersink_dia_mm = 5.6;
    in.chamfer_dia_mm     = 6.2;

    auto out = skill::bone_screw_pilot_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Material removed > pilot cylinder alone (cone + chamfer add more).
    const double removed  = stockVol - volumeOf(out.workpiece->shape());
    const double pilotR   = 2.5 / 2.0;
    const double pilotVol = M_PI * pilotR * pilotR * 8.0;
    EXPECT_GT(removed, pilotVol);
    EXPECT_LT(removed, pilotVol * 10.0);   // sanity upper

    // More faces after the compound cut.
    EXPECT_GT(out.workpiece->faceCount(), stockFC);

    // Signature
    EXPECT_EQ(out.signature.skill_id, std::string("bone_screw_pilot_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_EQ(out.signature.pattern.at("medical_feature_type").get<std::string>(),
              std::string("cortical_bone_screw_pilot"));
    EXPECT_EQ(out.signature.pattern.at("screw_size").get<std::string>(),
              std::string("3.5"));
    EXPECT_NEAR(out.signature.pattern.at("pilot_dia_mm").get<double>(), 2.5, 1e-6);
}

// ─── 2. validate rejects non-AO/ASIF size ────────────────────────────────
TEST(SkillBoneScrewPilot, ValidateRejectsNonStandardSize)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::bone_screw_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 20.0;
    in.position_y_mm      = 20.0;
    in.screw_size_id      = "5.0";   // NOT in AO/ASIF cortical set
    in.depth_mm           = 5.0;
    in.countersink_dia_mm = 6.0;
    in.chamfer_dia_mm     = 7.0;

    auto r = skill::bone_screw_pilot_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::bone_screw_pilot_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects bad geometry chain ──────────────────────────────
TEST(SkillBoneScrewPilot, ValidateRejectsBadGeometry)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    // (a) countersink ≤ pilot
    {
        skill::bone_screw_pilot_compound::Input in;
        in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm      = 20.0; in.position_y_mm = 20.0;
        in.screw_size_id      = "4.0";   // pilot 2.9 mm
        in.depth_mm           = 5.0;
        in.countersink_dia_mm = 2.8;     // < pilot → reject
        in.chamfer_dia_mm     = 6.0;
        auto r = skill::bone_screw_pilot_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-BONESCREW-COUNTERSINK") { found = true; break; }
        EXPECT_TRUE(found);
    }
    // (b) chamfer ≤ countersink
    {
        skill::bone_screw_pilot_compound::Input in;
        in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm      = 20.0; in.position_y_mm = 20.0;
        in.screw_size_id      = "4.0";
        in.depth_mm           = 5.0;
        in.countersink_dia_mm = 6.0;
        in.chamfer_dia_mm     = 5.5;     // < CSK → reject
        auto r = skill::bone_screw_pilot_compound::validate(*stock, in);
        EXPECT_FALSE(r.passed);
        bool found = false;
        for (const auto& f : r.findings)
            if (f.code == "DFM-BONESCREW-CHAMFER") { found = true; break; }
        EXPECT_TRUE(found);
    }
}

// ─── 4. recognize replays metadata at confidence 1.0 ─────────────────────
TEST(SkillBoneScrewPilot, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::bone_screw_pilot_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm      = 20.0; in.position_y_mm = 20.0;
    in.screw_size_id      = "2.7";
    in.depth_mm           = 6.0;
    in.countersink_dia_mm = 4.5;
    in.chamfer_dia_mm     = 5.0;

    auto out = skill::bone_screw_pilot_compound::apply(*stock, in);
    auto cands = skill::bone_screw_pilot_compound::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("screw_size_id").get<std::string>(),
              std::string("2.7"));
}

// ─── 5. AO/ASIF pilot table matches reference values ─────────────────────
TEST(SkillBoneScrewPilot, PilotDiameterMatchesAoAsifTable)
{
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("2.0"), 1.5, 1e-6);
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("2.4"), 1.8, 1e-6);
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("2.7"), 2.0, 1e-6);
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("3.5"), 2.5, 1e-6);
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("4.0"), 2.9, 1e-6);
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("4.5"), 3.2, 1e-6);
    EXPECT_NEAR(skill::bone_screw_pilot_compound::pilotDiaFor("6.5"), 4.5, 1e-6);
    EXPECT_DOUBLE_EQ(skill::bone_screw_pilot_compound::pilotDiaFor("M3"), 0.0);
}
