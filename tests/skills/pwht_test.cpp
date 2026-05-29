// @lat: [[process/test-strategy#skill round-trip]]
//
// pwht — post-weld heat treatment for heavy hull welds.  Covers:
//   1. apply preserves geometry verbatim
//   2. validate rejects bad input (non-positive hold_temp / time)
//   3. signature records kind + weld_id + hold_temp_c + hold_time_h
//   4. recognize replays metadata
//   5. out-of-range hold_temp emits DFM-PWHT-TEMP info but does NOT block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pwht.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillPwht, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::pwht::Input in;
    in.weld_id     = "WLD-77";
    in.hold_temp_c = 620.0;     // in band
    in.hold_time_h = 2.0;

    auto out = skill::pwht::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillPwht, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::pwht::Input in;
    in.weld_id     = "WLD-bad";
    in.hold_temp_c = 0.0;     // invalid
    in.hold_time_h = 2.0;

    auto r = skill::pwht::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::pwht::apply(*stock, in), skill::SkillError);

    // Non-positive hold_time also blocks.
    in.hold_temp_c = 620.0;
    in.hold_time_h = 0.0;
    auto r2 = skill::pwht::validate(*stock, in);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. signature records kind + key params ──────────────────────────────
TEST(SkillPwht, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::pwht::Input in;
    in.weld_id     = "WLD-X9";
    in.hold_temp_c = 650.0;
    in.hold_time_h = 3.0;

    auto out = skill::pwht::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("pwht"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pwht"));
    EXPECT_EQ(out.signature.pattern["weld_id"].get<std::string>(),
              std::string("WLD-X9"));
    EXPECT_NEAR(out.signature.pattern["hold_temp_c"].get<double>(),
                650.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["hold_time_h"].get<double>(),
                3.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["residual_stress"].get<std::string>(),
              std::string("low"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillPwht, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::pwht::Input in;
    in.weld_id     = "WLD-recog";
    in.hold_temp_c = 600.0;
    in.hold_time_h = 1.5;
    auto out = skill::pwht::apply(*stock, in);

    auto cands = skill::pwht::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_id"].get<std::string>(),
              std::string("WLD-recog"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::pwht::recognize(raw).empty());
}

// ─── 5. Out-of-range hold_temp emits info but does NOT block ─────────────
TEST(SkillPwht, OutOfBandTempEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "marine_steel");

    skill::pwht::Input in;
    in.weld_id     = "WLD-cold";
    in.hold_temp_c = 400.0;     // below 550 °C band
    in.hold_time_h = 2.0;

    auto r = skill::pwht::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "out-of-band hold_temp should emit info, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-PWHT-TEMP"));
    EXPECT_NO_THROW(skill::pwht::apply(*stock, in));
}
