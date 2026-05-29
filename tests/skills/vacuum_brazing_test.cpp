// @lat: [[process/test-strategy#skill round-trip]]
//
// vacuum_brazing — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range peak_temp, metadata-only recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vacuum_brazing.hpp"

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

skill::vacuum_brazing::Input goodInput()
{
    skill::vacuum_brazing::Input in;
    in.filler_metal = "BAg-8";
    in.vacuum_torr  = 1.0e-5;
    in.peak_temp_c  = 815.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillVacuumBrazing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::vacuum_brazing::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("vacuum_brazing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (peak_temp 500 C — soldering, not brazing)
TEST(SkillVacuumBrazing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.peak_temp_c = 500.0;   // <= 600 C is soldering, not brazing
    auto r = skill::vacuum_brazing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BRAZE-TEMP"));
    EXPECT_THROW(skill::vacuum_brazing::apply(*stock, in), skill::SkillError);

    // empty filler ⇒ DFM-INPUT error
    auto in2 = goodInput();
    in2.filler_metal = "";
    auto r2 = skill::vacuum_brazing::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillVacuumBrazing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.filler_metal = "BNi-2";
    in.vacuum_torr  = 5.0e-6;
    in.peak_temp_c  = 1010.0;

    auto out = skill::vacuum_brazing::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("vacuum_brazing"));
    EXPECT_EQ(out.signature.pattern["filler_metal"].get<std::string>(),
              std::string("BNi-2"));
    EXPECT_NEAR(out.signature.pattern["vacuum_torr"].get<double>(),
                5.0e-6, 1e-15);
    EXPECT_NEAR(out.signature.pattern["peak_temp_c"].get<double>(),
                1010.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["joint_quality"].get<std::string>(),
              std::string("leak_tight"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillVacuumBrazing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::vacuum_brazing::apply(*stock, goodInput());

    auto cands = skill::vacuum_brazing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["filler_metal"].get<std::string>(),
              std::string("BAg-8"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::vacuum_brazing::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "vacuum_brazing cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: poor vacuum (> 1e-3 torr) emits warning, still proceeds ─
TEST(SkillVacuumBrazing, PoorVacuumWarnsButProceeds)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.vacuum_torr = 1.0e-2;   // > 1e-3 torr — warning regime
    auto r = skill::vacuum_brazing::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BRAZE-VAC"));
    EXPECT_NO_THROW(skill::vacuum_brazing::apply(*stock, in));
}
