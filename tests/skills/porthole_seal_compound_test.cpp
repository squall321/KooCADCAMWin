// @lat: [[process/test-strategy#skill round-trip]]
//
// porthole_seal_compound (slice 16) — REAL geometric verification.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/porthole_seal_compound.hpp"

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

skill::porthole_seal_compound::Input goodInput()
{
    skill::porthole_seal_compound::Input in;
    in.entry_face          = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.center_x_mm         = 250.0;
    in.center_y_mm         = 250.0;
    in.axis_dir            = gp_Dir(0, 0, 1);
    in.porthole_dia_mm     = 300.0;
    in.o_ring_size_key     = "-325";
    in.bolt_count          = 12;
    in.bolt_pcd_mm         = 360.0;
    in.bolt_thread_size_key = "M8";
    in.plate_thickness_mm  = 12.0;
    return in;
}

}  // namespace

TEST(SkillPortholeSealCompound, ApplyRemovesExpectedVolume)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 12.0);
    ASSERT_FALSE(stock->shape().IsNull());

    const double volBefore = volumeOf(stock->shape());
    const int facesBefore  = stock->faceCount();

    auto in  = goodInput();
    auto out = skill::porthole_seal_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double volAfter = volumeOf(out.workpiece->shape());
    const double removed  = volBefore - volAfter;

    // Expect at least the porthole bore volume removed.
    const double rBore = in.porthole_dia_mm / 2.0;
    const double vBoreMin = M_PI * rBore * rBore * in.plate_thickness_mm;
    EXPECT_GT(removed, vBoreMin * 0.9) << "bore alone should consume ~"
                                       << vBoreMin << " mm³";

    EXPECT_GT(out.workpiece->faceCount(), facesBefore + in.bolt_count);
}

TEST(SkillPortholeSealCompound, ValidateRejectsBadAS568Key)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 12.0);

    auto in = goodInput();
    in.o_ring_size_key = "-999";  // not in central AS568 table

    auto r = skill::porthole_seal_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-AS568"));
    EXPECT_THROW(skill::porthole_seal_compound::apply(*stock, in),
                 skill::SkillError);
}

TEST(SkillPortholeSealCompound, SignatureIsCompound)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 12.0);

    auto in  = goodInput();
    auto out = skill::porthole_seal_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("porthole_seal_compound"));
    EXPECT_TRUE(out.signature.pattern.value("is_compound", false));
    EXPECT_EQ(out.signature.pattern.value("marine_feature_type", std::string{}),
              std::string("porthole_with_seal"));
    const int subCount = out.signature.pattern.value("subfeature_count", 0);
    EXPECT_EQ(subCount, 2 + in.bolt_count);
    EXPECT_EQ(out.signature.params.value("bolt_count", -1), in.bolt_count);
}

TEST(SkillPortholeSealCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 12.0);

    auto in  = goodInput();
    auto out = skill::porthole_seal_compound::apply(*stock, in);
    auto cands = skill::porthole_seal_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool ok = false;
    for (const auto& c : cands) {
        if (c.confidence >= 0.85) { ok = true; break; }
    }
    EXPECT_TRUE(ok);
}

TEST(SkillPortholeSealCompound, BoltCountRoundTripsViaParams)
{
    auto stock = skill::createCuboidStock(500.0, 500.0, 12.0);

    auto in  = goodInput();
    in.bolt_count = 8;
    in.bolt_pcd_mm = 360.0;

    auto out = skill::porthole_seal_compound::apply(*stock, in);
    auto cands = skill::porthole_seal_compound::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());

    bool found = false;
    for (const auto& c : cands) {
        if (c.recovered_params.value("bolt_count", -1) == in.bolt_count) {
            found = true; break;
        }
    }
    EXPECT_TRUE(found);
}
