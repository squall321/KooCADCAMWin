// @lat: [[process/test-strategy#skill round-trip]]
//
// anchor_bolt_install skill — 5-case sweep covering identity-geometry
// synthesis, DFM rejection of insufficient embedment, metadata-only
// recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anchor_bolt_install.hpp"

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

skill::anchor_bolt_install::Input goodInput()
{
    skill::anchor_bolt_install::Input in;
    in.bolt_dia_mm  = 24.0;
    in.embedment_mm = 240.0;        // 10 × dia
    in.torque_nm    = 300.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillAnchorBoltInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::anchor_bolt_install::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("anchor_bolt_install"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (embedment 100 mm < 8 × 24 = 192 mm) ──
TEST(SkillAnchorBoltInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.embedment_mm = 100.0;        // < 8 × 24 = 192 mm
    auto r = skill::anchor_bolt_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-AB-EMBED"));
    EXPECT_THROW(skill::anchor_bolt_install::apply(*stock, in), skill::SkillError);

    // Diameter below M12 minimum
    auto in2 = goodInput();
    in2.bolt_dia_mm = 8.0;
    in2.embedment_mm = 100.0;       // ratio 12.5 → DFM-AB-DIA dominates
    auto r2 = skill::anchor_bolt_install::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-AB-DIA"));
}

// ─── 3. Signature records kind + dia/embed/torque + embed_ratio ──────────
TEST(SkillAnchorBoltInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bolt_dia_mm  = 30.0;
    in.embedment_mm = 360.0;        // ratio 12
    in.torque_nm    = 500.0;

    auto out = skill::anchor_bolt_install::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("anchor_bolt_install"));
    EXPECT_NEAR(out.signature.pattern["bolt_dia_mm"].get<double>(),
                30.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["embedment_mm"].get<double>(),
                360.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["torque_nm"].get<double>(),
                500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["embed_ratio"].get<double>(),
                12.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAnchorBoltInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::anchor_bolt_install::apply(*stock, goodInput());

    auto cands = skill::anchor_bolt_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["bolt_dia_mm"].get<double>(),
                24.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["torque_nm"].get<double>(),
                300.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::anchor_bolt_install::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "anchor_bolt_install cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: very deep embedment emits info, still passes ────────────
TEST(SkillAnchorBoltInstall, DeepEmbedmentEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto deepIn = goodInput();
    deepIn.bolt_dia_mm  = 20.0;
    deepIn.embedment_mm = 800.0;       // 40 × dia → info-only
    auto r = skill::anchor_bolt_install::validate(*stock, deepIn);
    EXPECT_TRUE(r.passed)
        << "deep embedment should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-AB-EMBED"));

    EXPECT_NO_THROW(skill::anchor_bolt_install::apply(*stock, deepIn));
}
