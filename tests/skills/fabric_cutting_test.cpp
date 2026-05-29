// @lat: [[process/test-strategy#skill round-trip]]
//
// fabric_cutting — 5-case sweep covering identity geometry, DFM rejection of
// empty pattern_id / non-positive cut_length_m / out-of-range waste_pct,
// signature recording of pattern_id + cut_length_m + waste_pct,
// metadata-replay recognition, and usable_length_m arithmetic.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fabric_cutting.hpp"

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

}  // namespace

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillFabricCutting, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::fabric_cutting::Input in;
    in.pattern_id   = "MARKER-2026-001";
    in.cut_length_m = 5.0;
    in.waste_pct    = 12.0;

    auto out = skill::fabric_cutting::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("fabric_cutting"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects empty pattern_id / bad length / out-of-range waste ────
TEST(SkillFabricCutting, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_cutting::Input emptyId;
    emptyId.pattern_id = ""; emptyId.cut_length_m = 5.0; emptyId.waste_pct = 10.0;
    EXPECT_FALSE(skill::fabric_cutting::validate(*stock, emptyId).passed);
    EXPECT_THROW(skill::fabric_cutting::apply(*stock, emptyId), skill::SkillError);

    skill::fabric_cutting::Input zeroLen;
    zeroLen.pattern_id = "M"; zeroLen.cut_length_m = 0.0; zeroLen.waste_pct = 10.0;
    EXPECT_FALSE(skill::fabric_cutting::validate(*stock, zeroLen).passed);

    skill::fabric_cutting::Input negWaste;
    negWaste.pattern_id = "M"; negWaste.cut_length_m = 5.0; negWaste.waste_pct = -5.0;
    EXPECT_FALSE(skill::fabric_cutting::validate(*stock, negWaste).passed);
    EXPECT_THROW(skill::fabric_cutting::apply(*stock, negWaste), skill::SkillError);

    skill::fabric_cutting::Input overWaste;
    overWaste.pattern_id = "M"; overWaste.cut_length_m = 5.0; overWaste.waste_pct = 150.0;
    EXPECT_FALSE(skill::fabric_cutting::validate(*stock, overWaste).passed);
}

// ── 3. Signature records kind + pattern_id + cut_length_m + waste_pct ────
TEST(SkillFabricCutting, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_cutting::Input in;
    in.pattern_id   = "MARKER-XL";
    in.cut_length_m = 12.5;
    in.waste_pct    = 8.0;

    auto out = skill::fabric_cutting::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fabric_cutting"));
    EXPECT_EQ(out.signature.pattern["pattern_id"].get<std::string>(),
              std::string("MARKER-XL"));
    EXPECT_NEAR(out.signature.pattern["cut_length_m"].get<double>(),
                12.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["waste_pct"].get<double>(),
                8.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("fabric_cutter"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFabricCutting, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_cutting::Input in;
    in.pattern_id   = "MARKER-77";
    in.cut_length_m = 3.2;
    in.waste_pct    = 15.0;

    auto out = skill::fabric_cutting::apply(*stock, in);
    auto cands = skill::fabric_cutting::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["pattern_id"].get<std::string>(),
              std::string("MARKER-77"));
    EXPECT_NEAR(cands[0].recovered_params["cut_length_m"].get<double>(),
                3.2, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["waste_pct"].get<double>(),
                15.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::fabric_cutting::recognize(raw).empty());
}

// ── 5. usable_length_m = cut_length_m × (1 − waste_pct/100) ──────────────
TEST(SkillFabricCutting, UsableLengthExcludesWaste)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_cutting::Input in;
    in.pattern_id   = "MARKER-A";
    in.cut_length_m = 10.0;
    in.waste_pct    = 25.0;

    auto out = skill::fabric_cutting::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["usable_length_m"].get<double>(),
                10.0 * (1.0 - 0.25), 1e-9);

    // Excessive waste_pct (> 20) → DFM-TEX-WASTE info finding fires
    auto r = skill::fabric_cutting::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool info = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-TEX-WASTE" && f.severity == "info") {
            info = true; break;
        }
    }
    EXPECT_TRUE(info);
}
