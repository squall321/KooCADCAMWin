// @lat: [[process/test-strategy#skill round-trip]]
//
// two_shot_molding — bicomponent injection molding (metadata-only).
//
// Cases (5):
//   1. apply handles input — geometry unchanged.
//   2. validate rejects bad input (empty material_1).
//   3. signature records kind + material_1 + material_2 + parting_geometry.
//   4. recognize() replays metadata.
//   5. specific assertion: identical material_1/material_2 yields an INFO
//      finding but still passes (and unknown parting_geometry → info too).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/two_shot_molding.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code,
                const std::string& sev)
{
    for (const auto& f : r.findings)
        if (f.code == code && f.severity == sev) return true;
    return false;
}

}  // namespace

// ─── 1. Apply handles input — geometric no-op ──────────────────────────────
TEST(SkillTwoShotMolding, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0, "abs");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::two_shot_molding::Input in;
    in.material_1       = "abs";
    in.material_2       = "tpe";
    in.parting_geometry = "planar";

    auto out = skill::two_shot_molding::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("two_shot_molding"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillTwoShotMolding, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::two_shot_molding::Input bad;
    bad.material_1       = "";       // missing
    bad.material_2       = "tpe";
    bad.parting_geometry = "planar";

    auto r = skill::two_shot_molding::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-2SHOT-MATERIAL", "error"));
    EXPECT_THROW(skill::two_shot_molding::apply(*stock, bad),
                 skill::SkillError);
}

// ─── 3. Signature records kind + materials + parting ───────────────────────
TEST(SkillTwoShotMolding, SignatureRecordsKindAndMaterials)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0, "abs");

    skill::two_shot_molding::Input in;
    in.material_1       = "polycarbonate";
    in.material_2       = "tpu";
    in.parting_geometry = "stepped";

    auto out = skill::two_shot_molding::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("two_shot_molding"));
    EXPECT_EQ(out.signature.pattern["material_1"].get<std::string>(),
              std::string("polycarbonate"));
    EXPECT_EQ(out.signature.pattern["material_2"].get<std::string>(),
              std::string("tpu"));
    EXPECT_EQ(out.signature.pattern["parting_geometry"].get<std::string>(),
              std::string("stepped"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_bicomponent"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("two_shot_mold"));
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillTwoShotMolding, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::two_shot_molding::Input in;
    in.material_1       = "abs";
    in.material_2       = "tpe";
    in.parting_geometry = "planar";

    auto out   = skill::two_shot_molding::apply(*stock, in);
    auto cands = skill::two_shot_molding::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["material_1"].get<std::string>(),
              std::string("abs"));
    EXPECT_EQ(cands[0].recovered_params["material_2"].get<std::string>(),
              std::string("tpe"));

    // Stripped metadata → no recognition (no geometric trace).
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::two_shot_molding::recognize(raw).empty());
}

// ─── 5. Identical materials + unknown parting → INFO (not error) ──────────
TEST(SkillTwoShotMolding, IdenticalMaterialsAndUnknownPartingInfoOnly)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::two_shot_molding::Input in;
    in.material_1       = "abs";
    in.material_2       = "abs";          // same → INFO
    in.parting_geometry = "spiral";       // unknown → INFO

    auto r = skill::two_shot_molding::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "identical materials + unknown parting are info-level, not errors";
    EXPECT_TRUE(hasFinding(r, "DFM-2SHOT-MATERIAL", "info"));
    EXPECT_TRUE(hasFinding(r, "DFM-2SHOT-PARTING",  "info"));

    EXPECT_NO_THROW(skill::two_shot_molding::apply(*stock, in));
}
