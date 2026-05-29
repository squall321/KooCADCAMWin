// @lat: [[process/test-strategy#skill round-trip]]
//
// paint_marine skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection of out-of-range dry-film thickness, signature-records-
// kind+params, metadata-only recognition, and the skill-specific
// assertion that the added coating volume is recorded as a negative
// stock_removed_mm3 equal to -(area_m2 × 1e6 × dft_um × 1e-3).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/paint_marine.hpp"

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

skill::paint_marine::Input goodInput()
{
    skill::paint_marine::Input in;
    in.coat_system = "zinc_epoxy_AF";
    in.dft_um      = 350.0;
    in.area_m2     = 100.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPaintMarine, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::paint_marine::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("paint_marine"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillPaintMarine, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.dft_um = 20.0;                          // < 50 μm minimum
    auto r = skill::paint_marine::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PAINT-DFT"));
    EXPECT_THROW(skill::paint_marine::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.coat_system = "";                      // empty
    auto r2 = skill::paint_marine::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    auto in3 = goodInput();
    in3.area_m2 = 0.0;                         // ≤ 0
    auto r3 = skill::paint_marine::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPaintMarine, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.coat_system = "PU_topcoat_HB";
    in.dft_um      = 500.0;
    in.area_m2     = 250.0;
    auto out = skill::paint_marine::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("paint_marine"));
    EXPECT_EQ(out.signature.pattern["coat_system"].get<std::string>(),
              std::string("PU_topcoat_HB"));
    EXPECT_NEAR(out.signature.pattern["dft_um"].get<double>(),  500.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["area_m2"].get<double>(), 250.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPaintMarine, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::paint_marine::apply(*stock, goodInput());

    auto cands = skill::paint_marine::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["coat_system"].get<std::string>(),
              std::string("zinc_epoxy_AF"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::paint_marine::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "paint_marine cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: coating volume = area × dft, recorded as negative mm3 ───
TEST(SkillPaintMarine, CoatingVolumeRecordedAsNegativeMm3)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.area_m2 = 200.0;        // 200 m² = 2e8 mm²
    in.dft_um  = 500.0;        // 500 μm = 0.5 mm
    // Coating volume = 2e8 × 0.5 = 1.0e8 mm³, recorded as -1.0e8.
    auto out = skill::paint_marine::apply(*stock, in);

    EXPECT_NEAR(out.signature.pattern["coating_volume_mm3"].get<double>(),
                1.0e8, 1.0);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, -1.0e8, 1.0);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("airless_spray_rig"));
}
