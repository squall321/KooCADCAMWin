// @lat: [[process/test-strategy#skill round-trip]]
//
// high_speed_machining skill — 5-case sweep covering:
//   1. apply() clones geometry unchanged (metadata-only wrapper)
//   2. validate() rejects radial_engagement_ratio > 0.10 (DFM-HSM-RAE)
//   3. signature records kind + HSM strategy + tool_type=end_mill_hsm
//   4. recognize() metadata replay
//   5. spindle_rpm < 10000 rejected (DFM-HSM-RPM)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/high_speed_machining.hpp"

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
TEST(SkillHighSpeedMachining, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::high_speed_machining::Input in;
    in.sub_skill_id            = "mill_rect_pocket";
    in.spindle_rpm             = 24000.0;
    in.radial_engagement_ratio = 0.05;
    in.feed_per_tooth_mm       = 0.06;
    in.axial_doc_mm            = 0.0;
    in.tool_dia_mm             = 6.0;

    auto out = skill::high_speed_machining::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("high_speed_machining"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects radial_engagement_ratio > 0.10 ──────────────────
TEST(SkillHighSpeedMachining, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::high_speed_machining::Input in;
    in.sub_skill_id            = "mill_rect_pocket";
    in.spindle_rpm             = 24000.0;
    in.radial_engagement_ratio = 0.30;       // > 0.10 → error
    in.feed_per_tooth_mm       = 0.06;
    in.tool_dia_mm             = 6.0;

    auto r = skill::high_speed_machining::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HSM-RAE"));
    EXPECT_THROW(skill::high_speed_machining::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records kind + HSM strategy + tool_type ────────────────
TEST(SkillHighSpeedMachining, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::high_speed_machining::Input in;
    in.sub_skill_id            = "mill_circular_pocket";
    in.spindle_rpm             = 30000.0;
    in.radial_engagement_ratio = 0.08;
    in.feed_per_tooth_mm       = 0.08;
    in.axial_doc_mm            = 0.0;
    in.tool_dia_mm             = 4.0;

    auto out = skill::high_speed_machining::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("high_speed_machining"));
    EXPECT_EQ(out.signature.pattern["strategy"].get<std::string>(),
              std::string("hsm"));
    EXPECT_EQ(out.signature.pattern["sub_skill_id"].get<std::string>(),
              std::string("mill_circular_pocket"));
    EXPECT_NEAR(out.signature.pattern["spindle_rpm"].get<double>(),
                30000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["radial_engagement_ratio"].get<double>(),
                0.08, 1e-9);

    EXPECT_EQ(out.signature.tooling.tool_type, std::string("end_mill_hsm"));
    EXPECT_GT(out.signature.tooling.cutting_speed_sfm, 1000.0)
        << "HSM produces a very high derived surface speed";
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillHighSpeedMachining, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::high_speed_machining::Input in;
    in.sub_skill_id            = "mill_rect_pocket";
    in.spindle_rpm             = 25000.0;
    in.radial_engagement_ratio = 0.05;
    in.feed_per_tooth_mm       = 0.06;
    in.tool_dia_mm             = 6.0;

    auto out   = skill::high_speed_machining::apply(*stock, in);
    auto cands = skill::high_speed_machining::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["spindle_rpm"].get<double>(),
                25000.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["sub_skill_id"].get<std::string>(),
              std::string("mill_rect_pocket"));

    // Metadata-free → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::high_speed_machining::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. spindle_rpm < 10000 rejected ─────────────────────────────────────
TEST(SkillHighSpeedMachining, ValidateRejectsLowRpm)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 10.0);

    skill::high_speed_machining::Input in;
    in.sub_skill_id            = "mill_rect_pocket";
    in.spindle_rpm             = 8000.0;       // < 10000 → error
    in.radial_engagement_ratio = 0.05;
    in.feed_per_tooth_mm       = 0.06;
    in.tool_dia_mm             = 6.0;

    auto r = skill::high_speed_machining::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-HSM-RPM"));
    EXPECT_THROW(skill::high_speed_machining::apply(*stock, in),
                 skill::SkillError);
}
