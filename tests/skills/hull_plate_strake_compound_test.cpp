// @lat: [[process/test-strategy#skill round-trip]]
//
// hull_plate_strake_compound (slice 16) — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hull_plate_strake_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

skill::hull_plate_strake_compound::Input goodInput()
{
    skill::hull_plate_strake_compound::Input in;
    in.entry_face         = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.strake_axis_origin = gp_Pnt(20.0, 50.0, 0.0);
    in.strake_axis_dir    = gp_Dir(1, 0, 0);
    in.drill_axis_dir     = gp_Dir(0, 0, 1);
    in.strake_length_mm   = 200.0;
    in.rivet_pitch_mm     = 40.0;
    in.rivet_dia_mm       = 8.0;
    in.rivet_count        = 5;
    in.edge_radius_mm     = 1.5;
    in.plate_thickness_mm = 10.0;
    return in;
}

}  // namespace

TEST(SkillHullPlateStrake, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(300.0, 120.0, 10.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::hull_plate_strake_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;

    const double rR = in.rivet_dia_mm / 2.0;
    const double vRivetsMin = static_cast<double>(in.rivet_count) *
        M_PI * rR * rR * in.plate_thickness_mm * 0.9;
    EXPECT_GT(removed, vRivetsMin);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore + in.rivet_count - 1);
}

TEST(SkillHullPlateStrake, ValidateRejectsTooTightRivetPitch)
{
    auto stock = skill::createCuboidStock(300.0, 120.0, 10.0);

    auto in = goodInput();
    in.rivet_pitch_mm = 2.0 * in.rivet_dia_mm;  // < 3×dia

    auto r = skill::hull_plate_strake_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MARINE-RIVET"));
    EXPECT_THROW(skill::hull_plate_strake_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillHullPlateStrake, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(300.0, 120.0, 10.0);

    auto in  = goodInput();
    auto out = skill::hull_plate_strake_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("hull_plate_strake_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string{}),
              std::string("hull_strake_rivet_line"));
    EXPECT_EQ(out.signature.params.value("rivet_count", -1), in.rivet_count);
    EXPECT_NEAR(out.signature.params.value("rivet_pitch_mm", 0.0),
                in.rivet_pitch_mm, 1e-6);
}

TEST(SkillHullPlateStrake, RecognizeFindsRivetLine)
{
    auto stock = skill::createCuboidStock(300.0, 120.0, 10.0);

    auto in  = goodInput();
    auto out = skill::hull_plate_strake_compound::apply(*stock, in);
    auto cands = skill::hull_plate_strake_compound::recognize(*out.workpiece);

    ASSERT_FALSE(cands.empty());
    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.75) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

TEST(SkillHullPlateStrake, RecoveredRivetCountMatches)
{
    auto stock = skill::createCuboidStock(300.0, 120.0, 10.0);

    auto in  = goodInput();
    in.rivet_count = 4;

    auto out = skill::hull_plate_strake_compound::apply(*stock, in);
    auto cands = skill::hull_plate_strake_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        if (c.recovered_params.value("rivet_count", -1) == in.rivet_count) {
            found = true; break;
        }
    }
    EXPECT_TRUE(found);
}
