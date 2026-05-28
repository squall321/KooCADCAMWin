// @lat: [[process/test-strategy#skill round-trip]]
//
// multi_material_3d_print — like 3d_print but with per-region material map.
//
// Cases (5):
//   1. apply handles input — geometric no-op + signature carries region map.
//   2. validate rejects bad input (single region).
//   3. signature records kind + process_type + region_material_map.
//   4. recognize() replays metadata.
//   5. specific assertion: layer_pct sum > 100 yields error (over-spec).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/multi_material_3d_print.hpp"

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
                const std::string& sev = "")
{
    for (const auto& f : r.findings) {
        if (f.code != code) continue;
        if (!sev.empty() && f.severity != sev) continue;
        return true;
    }
    return false;
}

}  // namespace

// ─── 1. Apply handles input — geometric no-op ──────────────────────────────
TEST(SkillMultiMaterial3DPrint, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "pla");
    const double volBefore = volumeOf(stock->shape());

    skill::mmp3d_print::Input in;
    in.process_type    = "fdm";
    in.layer_height_mm = 0.2;
    in.region_material_map = {
        { "frame",  "pla",  60.0 },
        { "grip",   "tpu",  30.0 },
        { "label",  "petg", 10.0 },
    };

    auto out = skill::mmp3d_print::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("multi_material_3d_print"));
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_multi_material"].get<bool>());
}

// ─── 2. Validate rejects bad input (single region) ─────────────────────────
TEST(SkillMultiMaterial3DPrint, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mmp3d_print::Input singleRegion;
    singleRegion.process_type    = "fdm";
    singleRegion.layer_height_mm = 0.2;
    singleRegion.region_material_map = {
        { "only", "pla", 100.0 },
    };

    auto r = skill::mmp3d_print::validate(*stock, singleRegion);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MMP3D-REGIONS", "error"));
    EXPECT_THROW(skill::mmp3d_print::apply(*stock, singleRegion),
                 skill::SkillError);

    // Bad layer height.
    skill::mmp3d_print::Input badLayer;
    badLayer.process_type    = "fdm";
    badLayer.layer_height_mm = 0.8;     // > 0.5
    badLayer.region_material_map = {
        { "a", "pla", 50.0 },
        { "b", "tpu", 50.0 },
    };
    auto r2 = skill::mmp3d_print::validate(*stock, badLayer);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-MMP3D-LAYER", "error"));

    // Empty material on one region.
    skill::mmp3d_print::Input emptyMat;
    emptyMat.process_type    = "fdm";
    emptyMat.layer_height_mm = 0.2;
    emptyMat.region_material_map = {
        { "a", "pla", 50.0 },
        { "b", "",    50.0 },      // empty
    };
    auto r3 = skill::mmp3d_print::validate(*stock, emptyMat);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-MMP3D-MATERIAL", "error"));
}

// ─── 3. Signature records kind + process + region map ─────────────────────
TEST(SkillMultiMaterial3DPrint, SignatureRecordsKindAndMaterials)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mmp3d_print::Input in;
    in.process_type    = "polyjet";
    in.layer_height_mm = 0.05;
    in.region_material_map = {
        { "rigid",  "vero_white",      40.0 },
        { "soft",   "agilus_30",       30.0 },
        { "clear",  "vero_clear",      30.0 },
    };

    auto out = skill::mmp3d_print::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("multi_material_3d_print"));
    EXPECT_EQ(out.signature.pattern["process_type"].get<std::string>(),
              std::string("polyjet"));
    EXPECT_EQ(out.signature.pattern["region_count"].get<int>(), 3);

    const auto regsJ = out.signature.pattern["region_material_map"];
    ASSERT_EQ(regsJ.size(), 3u);
    EXPECT_EQ(regsJ[0]["material"].get<std::string>(),
              std::string("vero_white"));
    EXPECT_EQ(regsJ[1]["material"].get<std::string>(),
              std::string("agilus_30"));
    EXPECT_NEAR(regsJ[2]["layer_pct"].get<double>(), 30.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("polyjet_head"));
}

// ─── 4. Recognize via metadata replay ──────────────────────────────────────
TEST(SkillMultiMaterial3DPrint, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mmp3d_print::Input in;
    in.process_type    = "fdm";
    in.layer_height_mm = 0.2;
    in.region_material_map = {
        { "body",   "petg", 70.0 },
        { "seal",   "tpu",  30.0 },
    };

    auto out   = skill::mmp3d_print::apply(*stock, in);
    auto cands = skill::mmp3d_print::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["process_type"].get<std::string>(),
              std::string("fdm"));
    const auto regsJ = cands[0].recovered_params["region_material_map"];
    ASSERT_EQ(regsJ.size(), 2u);
    EXPECT_EQ(regsJ[0]["region_id"].get<std::string>(),
              std::string("body"));

    // Stripped metadata → no recognition.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::mmp3d_print::recognize(raw).empty());
}

// ─── 5. Specific assertion: layer_pct sum > 100 → error ────────────────────
TEST(SkillMultiMaterial3DPrint, PctSumOverHundredIsError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::mmp3d_print::Input overSpec;
    overSpec.process_type    = "fdm";
    overSpec.layer_height_mm = 0.2;
    overSpec.region_material_map = {
        { "a", "pla", 60.0 },
        { "b", "tpu", 60.0 },     // 60 + 60 = 120 > 100 → error
    };

    auto r = skill::mmp3d_print::validate(*stock, overSpec);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MMP3D-PCTSUM", "error"));
    EXPECT_THROW(skill::mmp3d_print::apply(*stock, overSpec),
                 skill::SkillError);

    // Sum < 50 → info-level only (still passes).
    skill::mmp3d_print::Input partialSpec;
    partialSpec.process_type    = "fdm";
    partialSpec.layer_height_mm = 0.2;
    partialSpec.region_material_map = {
        { "a", "pla", 20.0 },
        { "b", "tpu", 20.0 },     // sum = 40 → info
    };
    auto r2 = skill::mmp3d_print::validate(*stock, partialSpec);
    EXPECT_TRUE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-MMP3D-PCTSUM", "info"));
    EXPECT_NO_THROW(skill::mmp3d_print::apply(*stock, partialSpec));
}
