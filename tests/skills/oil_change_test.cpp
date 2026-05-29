// @lat: [[process/test-strategy#skill round-trip]]
//
// oil_change skill — 5-case maintenance sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/oil_change.hpp"

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

skill::oil_change::Input goodInput()
{
    skill::oil_change::Input in;
    in.oil_grade    = "ISO VG 46";
    in.volume_l     = 10.0;
    in.drain_temp_c = 60.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillOilChange, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::oil_change::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("oil_change"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillOilChange, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.volume_l = 0.0;                     // non-positive
    auto r = skill::oil_change::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::oil_change::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.oil_grade = "";                    // empty grade
    auto r2 = skill::oil_change::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillOilChange, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.oil_grade    = "5W-30";
    in.volume_l     = 4.5;
    in.drain_temp_c = 75.0;

    auto out = skill::oil_change::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("oil_change"));
    EXPECT_EQ(out.signature.pattern["oil_grade"].get<std::string>(),
              std::string("5W-30"));
    EXPECT_NEAR(out.signature.pattern["volume_l"].get<double>(), 4.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["drain_temp_c"].get<double>(), 75.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillOilChange, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::oil_change::apply(*stock, goodInput());

    auto cands = skill::oil_change::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["oil_grade"].get<std::string>(),
              std::string("ISO VG 46"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::oil_change::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "oil_change cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: out-of-window drain temperature → INFO, not error ──────
TEST(SkillOilChange, OutOfWindowDrainTempIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.drain_temp_c = 15.0;                // cold drain → slow

    auto r = skill::oil_change::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "cold drain temperature should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-OIL-TEMP"));
    EXPECT_NO_THROW(skill::oil_change::apply(*stock, in));
}
