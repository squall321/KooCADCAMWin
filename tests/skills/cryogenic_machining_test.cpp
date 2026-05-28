// @lat: [[process/test-strategy#skill round-trip]]
//
// cryogenic_machining skill — 5-case sweep covering:
//   1. apply() clones geometry unchanged (metadata-only wrapper)
//   2. validate() rejects empty sub_skill_id (DFM-INPUT)
//   3. signature records kind + ln2 coolant + sub_skill reference
//   4. recognize() metadata replay
//   5. flow_rate_lpm ≤ 0 rejected (DFM-CRYO-FLOW)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/cryogenic_machining.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <nlohmann/json.hpp>

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
TEST(SkillCryogenicMachining, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "titanium_grade5");
    const double volBefore   = volumeOf(stock->shape());
    const int    faceBefore  = stock->faceCount();
    const int    edgeBefore  = stock->edgeCount();

    skill::cryogenic_machining::Input in;
    in.sub_skill_id     = "drill_hole";
    in.sub_skill_params = { { "diameter_mm", 3.0 }, { "depth_mm", 5.0 } };
    in.flow_rate_lpm    = 5.0;
    in.delivery         = "through_spindle";

    auto out = skill::cryogenic_machining::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("cryogenic_machining"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects empty sub_skill_id ──────────────────────────────
TEST(SkillCryogenicMachining, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cryogenic_machining::Input in;
    in.sub_skill_id  = "";          // invalid
    in.flow_rate_lpm = 5.0;

    auto r = skill::cryogenic_machining::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::cryogenic_machining::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records kind + LN2 coolant + sub_skill reference ───────
TEST(SkillCryogenicMachining, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "inconel_718");

    skill::cryogenic_machining::Input in;
    in.sub_skill_id     = "mill_circular_pocket";
    in.sub_skill_params = { { "diameter_mm", 6.0 }, { "depth_mm", 3.0 } };
    in.flow_rate_lpm    = 8.0;
    in.delivery         = "through_spindle";

    auto out = skill::cryogenic_machining::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("cryogenic_machining"));
    EXPECT_EQ(out.signature.pattern["coolant"].get<std::string>(),
              std::string("ln2"));
    EXPECT_EQ(out.signature.pattern["sub_skill_id"].get<std::string>(),
              std::string("mill_circular_pocket"));
    EXPECT_NEAR(out.signature.pattern["flow_rate_lpm"].get<double>(), 8.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["delivery"].get<std::string>(),
              std::string("through_spindle"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());

    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("cryogenic_delivery_nozzle"));
    ASSERT_TRUE(out.signature.tooling.extra.contains("cryogenic"));
    EXPECT_TRUE(out.signature.tooling.extra["cryogenic"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.extra["coolant_temp_c"].get<double>(),
                -196.0, 1e-9);
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillCryogenicMachining, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cryogenic_machining::Input in;
    in.sub_skill_id     = "drill_hole";
    in.sub_skill_params = { { "diameter_mm", 3.0 } };
    in.flow_rate_lpm    = 5.0;
    in.delivery         = "through_spindle";

    auto out   = skill::cryogenic_machining::apply(*stock, in);
    auto cands = skill::cryogenic_machining::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["sub_skill_id"].get<std::string>(),
              std::string("drill_hole"));
    EXPECT_NEAR(cands[0].recovered_params["flow_rate_lpm"].get<double>(),
                5.0, 1e-9);

    // Metadata-free → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::cryogenic_machining::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. flow_rate_lpm ≤ 0 rejected ───────────────────────────────────────
TEST(SkillCryogenicMachining, ValidateRejectsNonPositiveFlow)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::cryogenic_machining::Input in;
    in.sub_skill_id  = "drill_hole";
    in.flow_rate_lpm = 0.0;          // invalid

    auto r = skill::cryogenic_machining::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-CRYO-FLOW"));
    EXPECT_THROW(skill::cryogenic_machining::apply(*stock, in),
                 skill::SkillError);
}
