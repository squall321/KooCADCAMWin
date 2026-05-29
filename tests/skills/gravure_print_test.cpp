// @lat: [[process/test-strategy#skill round-trip]]
//
// gravure_print — rotogravure printing.  Metadata-only.
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndParams
//   4. RecognizeMetadataReplay
//   5. DeepCellEmitsInfoFinding

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gravure_print.hpp"

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

// ─── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillGravurePrint, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(1000.0, 500.0, 0.05);
    const double volBefore = volumeOf(stock->shape());
    const int faceBefore   = stock->faceCount();
    const int edgeBefore   = stock->edgeCount();

    skill::gravure_print::Input in;
    in.cylinder_dia_mm  = 250.0;
    in.line_speed_m_min = 300.0;
    in.depth_um         = 30.0;

    auto out = skill::gravure_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("gravure_print"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillGravurePrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(500.0, 300.0, 0.05);

    skill::gravure_print::Input badDia;
    badDia.cylinder_dia_mm  = 0.0;
    badDia.line_speed_m_min = 300.0;
    badDia.depth_um         = 30.0;
    {
        auto r = skill::gravure_print::validate(*stock, badDia);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::gravure_print::apply(*stock, badDia), skill::SkillError);

    skill::gravure_print::Input badDepth;
    badDepth.cylinder_dia_mm  = 250.0;
    badDepth.line_speed_m_min = 300.0;
    badDepth.depth_um         = -1.0;
    {
        auto r = skill::gravure_print::validate(*stock, badDepth);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::gravure_print::apply(*stock, badDepth), skill::SkillError);
}

// ─── 3. Signature records kind + key params ───────────────────────────────
TEST(SkillGravurePrint, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(800.0, 400.0, 0.05);

    skill::gravure_print::Input in;
    in.cylinder_dia_mm  = 400.0;
    in.line_speed_m_min = 450.0;
    in.depth_um         = 45.0;

    auto out = skill::gravure_print::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gravure_print"));
    EXPECT_NEAR(out.signature.pattern["cylinder_dia_mm"].get<double>(),
                400.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["line_speed_m_min"].get<double>(),
                450.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["depth_um"].get<double>(),
                45.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillGravurePrint, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(600.0, 400.0, 0.05);

    skill::gravure_print::Input in;
    in.cylinder_dia_mm  = 250.0;
    in.line_speed_m_min = 300.0;
    in.depth_um         = 30.0;

    auto out   = skill::gravure_print::apply(*stock, in);
    auto cands = skill::gravure_print::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["depth_um"].get<double>(),
                30.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["cylinder_dia_mm"].get<double>(),
                250.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::gravure_print::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: deep cell emits info finding (still passes) ─────────────
TEST(SkillGravurePrint, DeepCellEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(600.0, 400.0, 0.05);

    skill::gravure_print::Input in;
    in.cylinder_dia_mm  = 250.0;
    in.line_speed_m_min = 200.0;
    in.depth_um         = 80.0;       // > 60 → deep

    auto r = skill::gravure_print::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-DEPTH-DEEP"));
    EXPECT_NO_THROW(skill::gravure_print::apply(*stock, in));
}
