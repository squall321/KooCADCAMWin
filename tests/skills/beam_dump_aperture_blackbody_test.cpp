// @lat: [[process/test-strategy#beam_dump_aperture_blackbody]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/beam_dump_aperture_blackbody.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFace(const skill::Workpiece& wp)
{
    int    bestId   = -1;
    double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}

skill::beam_dump_aperture_blackbody::Input good(int faceId)
{
    skill::beam_dump_aperture_blackbody::Input in;
    in.face_id                   = faceId;
    in.center_x_mm               = 30.0;
    in.center_y_mm               = 30.0;
    in.aperture_dia_mm           = 5.0;
    in.aperture_depth_mm         = 3.0;
    in.internal_chamber_dia_mm   = 15.0;
    in.internal_chamber_depth_mm = 18.0;
    in.cone_angle_deg            = 12.0;
    return in;
}

}  // namespace

// ─── 1. apply removes positive volume ────────────────────────────────────
TEST(SkillBeamDumpApertureBlackbody, ApplyRemovesVolume)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);
    auto in = good(topId);
    const double v0 = volumeOf(stock->shape());
    auto out = skill::beam_dump_aperture_blackbody::apply(*stock, in);
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_GT(v0 - v1, 1000.0) << "expected at least 1 cm^3 removed";
    EXPECT_EQ(out.signature.skill_id, std::string("beam_dump_aperture_blackbody"));
}

// ─── 2. validate rejects aperture ≥ chamber ──────────────────────────────
TEST(SkillBeamDumpApertureBlackbody, ValidateRejectsBadAperture)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.aperture_dia_mm = in.internal_chamber_dia_mm;
    auto r = skill::beam_dump_aperture_blackbody::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-APERTURE") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::beam_dump_aperture_blackbody::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects cone_angle out of [5, 30] ───────────────────────
TEST(SkillBeamDumpApertureBlackbody, ValidateRejectsBadConeAngle)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.cone_angle_deg = 2.0;        // < 5
    auto r = skill::beam_dump_aperture_blackbody::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-CONE-ANGLE") found = true;
    EXPECT_TRUE(found);

    in.cone_angle_deg = 35.0;       // > 30
    r = skill::beam_dump_aperture_blackbody::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. signature carries cone_angle_deg + is_compound ───────────────────
TEST(SkillBeamDumpApertureBlackbody, SignatureCarriesConicalChamber)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::beam_dump_aperture_blackbody::apply(*stock, in);
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["has_conical_chamber"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["optical_feature_type"].get<std::string>(),
              std::string("beam_dump_blackbody"));
    EXPECT_NEAR(out.signature.pattern["cone_angle_deg"].get<double>(),
                in.cone_angle_deg, 1e-6);
}

// ─── 5. recognize replays history ────────────────────────────────────────
TEST(SkillBeamDumpApertureBlackbody, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 80.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::beam_dump_aperture_blackbody::apply(*stock, in);
    auto cands = skill::beam_dump_aperture_blackbody::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id, std::string("beam_dump_aperture_blackbody"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
