// @lat: [[process/test-strategy#compound macro]]
//
// box_section_with_endplate — COMPOUND structural skill: hollow RHS tube +
// 2 endplates + bolt-hole grid through both endplates.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/box_section_with_endplate.hpp"

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

skill::box_section_with_endplate::Input goodInput()
{
    skill::box_section_with_endplate::Input in;
    in.length_mm     = 600.0;
    in.width_mm      = 100.0;
    in.height_mm     = 80.0;
    in.wall_t_mm     = 5.0;
    in.endplate_t_mm = 10.0;
    in.bolt_grid     = { 2, 2 };
    in.bolt_dia_mm   = 12.0;
    in.edge_dist_mm  = 25.0;
    return in;
}

}  // namespace

// ─── 1. Apply: shape valid + face count >> stock ──────────────────────────
TEST(SkillBoxSectionWithEndplate, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    const int    faceBefore = stock->faceCount();

    auto in = goodInput();
    auto out = skill::box_section_with_endplate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Sub-feature count = 4 + 2 × bolts_per_plate = 4 + 8 = 12.
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 12);
    EXPECT_GT(out.workpiece->faceCount(), faceBefore);

    // Volume sanity: hollow box + 2 plates minus bolt holes; should be > 0 and < bbox.
    const double vol = volumeOf(out.workpiece->shape());
    EXPECT_GT(vol, 0.0);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillBoxSectionWithEndplate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    // Bad: wall_t below EN 10219 minimum (1.6 mm).
    auto in1 = goodInput();
    in1.wall_t_mm = 1.0;
    auto r1 = skill::box_section_with_endplate::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-BOX-WALL"));
    EXPECT_THROW(skill::box_section_with_endplate::apply(*stock, in1),
                 skill::SkillError);

    // Bad: bolt edge distance too small (< 2 × bolt_dia).
    auto in2 = goodInput();
    in2.edge_dist_mm = 5.0;       // 2 × 12 = 24 mm required
    auto r2 = skill::box_section_with_endplate::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BOX-BOLT-PITCH"));
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillBoxSectionWithEndplate, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::box_section_with_endplate::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("box_section_with_endplate"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("box_section_with_endplate"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    // Verbatim params present.
    EXPECT_NEAR(out.signature.params["length_mm"].get<double>(),   600.0, 1e-9);
    EXPECT_NEAR(out.signature.params["width_mm"].get<double>(),    100.0, 1e-9);
    EXPECT_NEAR(out.signature.params["wall_t_mm"].get<double>(),     5.0, 1e-9);
    EXPECT_NEAR(out.signature.params["bolt_dia_mm"].get<double>(),  12.0, 1e-9);
}

// ─── 4. Recognize: metadata replay returns confidence 1.0 ────────────────
TEST(SkillBoxSectionWithEndplate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::box_section_with_endplate::apply(*stock, goodInput());

    auto cands = skill::box_section_with_endplate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["bolt_dia_mm"].get<double>(),
                12.0, 1e-9);

    // Strip metadata → geometric fallback may still recognize the box section
    // (slice-9 work-2 added geom-walk recognize).  Contract: any candidate
    // returned must carry confidence < 1.0 (NOT a metadata replay) and the
    // recovered bolt_dia_mm must be within 1 mm of the true 12 mm.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsRaw = skill::box_section_with_endplate::recognize(raw);
    for (const auto& c : candsRaw) {
        EXPECT_LT(c.confidence, 1.0)
            << "raw-geometry candidate must NOT claim metadata-level confidence";
        EXPECT_NEAR(c.recovered_params["bolt_dia_mm"].get<double>(), 12.0, 1.0);
    }
}

// ─── 5. SPECIFIC: total bolt holes = 2 × nx × ny ──────────────────────────
TEST(SkillBoxSectionWithEndplate, TotalBoltHolesEqualsTwicePerPlateGrid)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    auto in = goodInput();
    in.bolt_grid = { 3, 4 };       // 12 per plate, 24 total
    auto out = skill::box_section_with_endplate::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["n_bolts_per_plate"].get<int>(), 12);
    EXPECT_EQ(out.signature.pattern["n_bolt_holes_total"].get<int>(), 24);
    EXPECT_EQ(out.signature.pattern["n_endplates"].get<int>(), 2);

    // sub-feature count = 4 + 2 × 12 = 28
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 28);
}
