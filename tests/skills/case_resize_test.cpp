// @lat: [[process/test-strategy#skill round-trip]]
//
// case_resize skill — small-arms case resize + neck anneal no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/case_resize.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillCaseResize, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::case_resize::Input in;
    in.case_dia_neck_mm = 9.55;
    in.case_length_mm   = 19.15;
    in.anneal_temp_c    = 425.0;

    auto out = skill::case_resize::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: zero case length → error ────────────────
TEST(SkillCaseResize, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::case_resize::Input in;
    in.case_dia_neck_mm = 9.55;
    in.case_length_mm   = 0.0;        // invalid
    in.anneal_temp_c    = 425.0;

    auto r = skill::case_resize::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::case_resize::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillCaseResize, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::case_resize::Input in;
    in.case_dia_neck_mm = 6.45;
    in.case_length_mm   = 51.18;       // .308 Win
    in.anneal_temp_c    = 482.0;

    auto out = skill::case_resize::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("case_resize"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("case_resize"));
    EXPECT_NEAR(out.signature.pattern["case_dia_neck_mm"].get<double>(),
                6.45, 1e-6);
    EXPECT_NEAR(out.signature.pattern["case_length_mm"].get<double>(),
                51.18, 1e-6);
    EXPECT_NEAR(out.signature.pattern["anneal_temp_c"].get<double>(),
                482.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCaseResize, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::case_resize::Input in;
    in.case_dia_neck_mm = 9.55;
    in.case_length_mm   = 19.15;
    in.anneal_temp_c    = 425.0;

    auto out  = skill::case_resize::apply(*stock, in);
    auto cands = skill::case_resize::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["case_length_mm"].get<double>(),
                19.15, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::case_resize::recognize(raw).empty());
}

// ─── 5. Skill-specific: out-of-range anneal temp = info only ─────────────
TEST(SkillCaseResize, AnnealTempOutOfRangeIsInfoNotError)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::case_resize::Input in;
    in.case_dia_neck_mm = 9.55;
    in.case_length_mm   = 19.15;
    in.anneal_temp_c    = 600.0;       // above 350–550 sweet spot

    auto r = skill::case_resize::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "out-of-range anneal_temp_c should not block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-AMMO-ANNEAL"));
    EXPECT_NO_THROW(skill::case_resize::apply(*stock, in));
}
