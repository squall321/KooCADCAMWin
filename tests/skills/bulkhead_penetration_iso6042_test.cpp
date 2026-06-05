// @lat: [[process/test-strategy#skill round-trip]]
//
// bulkhead_penetration_iso6042 (slice 16) — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bulkhead_penetration_iso6042.hpp"

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

skill::bulkhead_penetration_iso6042::Input goodInput()
{
    skill::bulkhead_penetration_iso6042::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm           = 100.0;
    in.center_y_mm           = 100.0;
    in.axis_dir              = gp_Dir(0, 0, 1);
    in.penetration_dia_mm    = 32.0;
    in.bulkhead_thickness_mm = 12.0;
    in.fire_seal_ring_dia_mm = 60.0;
    in.fire_seal_depth_mm    = 4.0;
    return in;
}

}  // namespace

TEST(SkillBulkheadPenetration, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 12.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::bulkhead_penetration_iso6042::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;

    const double rBore = in.penetration_dia_mm / 2.0;
    const double vBore = M_PI * rBore * rBore * in.bulkhead_thickness_mm;
    EXPECT_GT(removed, vBore * 0.9);
    EXPECT_GT(out.workpiece->faceCount(), facesBefore);
}

TEST(SkillBulkheadPenetration, ValidateRejectsThinBulkhead)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 12.0);

    auto in = goodInput();
    in.bulkhead_thickness_mm = 2.0;  // < 3 mm

    auto r = skill::bulkhead_penetration_iso6042::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MARINE-PLATE"));
    EXPECT_THROW(skill::bulkhead_penetration_iso6042::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillBulkheadPenetration, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 12.0);

    auto in  = goodInput();
    auto out = skill::bulkhead_penetration_iso6042::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("bulkhead_penetration_iso6042"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string{}),
              std::string("bulkhead_fire_penetration"));
    EXPECT_EQ(out.signature.pattern.value("subfeature_count", 0), 3);
}

TEST(SkillBulkheadPenetration, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 12.0);

    auto in  = goodInput();
    auto out = skill::bulkhead_penetration_iso6042::apply(*stock, in);
    auto cands = skill::bulkhead_penetration_iso6042::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.8) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

TEST(SkillBulkheadPenetration, ValidateRejectsTooNarrowFireSeal)
{
    auto stock = skill::createCuboidStock(200.0, 200.0, 12.0);

    auto in = goodInput();
    in.fire_seal_ring_dia_mm = in.penetration_dia_mm + 2.0;  // < +4 mm

    auto r = skill::bulkhead_penetration_iso6042::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FIRE-SEAL"));
}
