// @lat: [[process/test-strategy#skill round-trip]]
//
// lance_n_form skill — lance (3-sided slot) + bend the freed tab.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lance_n_form.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
}  // namespace

// ── 1. Apply: lance removes a kerf + adds an upright tab ──────────────
TEST(SkillLanceNForm, ApplyChangesGeometryOrPreserves)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_n_form::Input in;
    in.tab_center_x_mm = 40.0;
    in.tab_center_y_mm = 40.0;
    in.tab_length_mm   = 10.0;     // ≥ 2 × 1.5 = 3.0
    in.tab_width_mm    = 6.0;
    in.form_height_mm  = 3.0;      // ≤ 4 × 1.5 = 6.0
    in.lance_kerf_mm   = 0.8;      // ≥ 0.5 × 1.5 = 0.75
    in.hinge_side      = skill::lance_n_form::HingeSide::PlusY;

    auto out = skill::lance_n_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume should differ from stock (lance kerfs removed + tab added).
    const double stockVol = volumeOf(stock->shape());
    const double newVol   = volumeOf(out.workpiece->shape());
    EXPECT_NE(newVol, stockVol);
}

// ── 2. Validate rejects bad input ─────────────────────────────────────
TEST(SkillLanceNForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    // Tab too small.
    skill::lance_n_form::Input in;
    in.tab_center_x_mm = 40.0;
    in.tab_center_y_mm = 40.0;
    in.tab_length_mm   = 2.0;       // < 2 × 1.5 = 3.0
    in.tab_width_mm    = 2.0;
    in.form_height_mm  = 3.0;
    in.lance_kerf_mm   = 0.8;

    auto r = skill::lance_n_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LNF-MIN-TAB") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::lance_n_form::apply(*stock, in), skill::SkillError);

    // Form height too tall.
    skill::lance_n_form::Input in2 = in;
    in2.tab_length_mm  = 10.0;
    in2.tab_width_mm   = 6.0;
    in2.form_height_mm = 10.0;      // > 4 × 1.5 = 6.0
    auto r2 = skill::lance_n_form::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    bool maxFormFound = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-LNF-MAX-FORM") { maxFormFound = true; break; }
    EXPECT_TRUE(maxFormFound);
}

// ── 3. Signature records kind, hinge side, form_height ────────────────
TEST(SkillLanceNForm, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_n_form::Input in;
    in.tab_center_x_mm = 40.0;
    in.tab_center_y_mm = 40.0;
    in.tab_length_mm   = 10.0;
    in.tab_width_mm    = 6.0;
    in.form_height_mm  = 3.0;
    in.lance_kerf_mm   = 0.8;
    in.hinge_side      = skill::lance_n_form::HingeSide::PlusX;

    auto out = skill::lance_n_form::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("lance_n_form"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("lance_n_form"));
    EXPECT_EQ(out.signature.pattern["hinge_side"].get<std::string>(),
              std::string("+X"));
    EXPECT_NEAR(out.signature.pattern["form_height_mm"].get<double>(), 3.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern["has_bent_tab"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("lance_form_die"));
}

// ── 4. Recognize replays signature from metadata ──────────────────────
TEST(SkillLanceNForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_n_form::Input in;
    in.tab_center_x_mm = 50.0;
    in.tab_center_y_mm = 40.0;
    in.tab_length_mm   = 12.0;
    in.tab_width_mm    = 8.0;
    in.form_height_mm  = 2.5;
    in.lance_kerf_mm   = 0.9;
    in.hinge_side      = skill::lance_n_form::HingeSide::MinusY;

    auto out = skill::lance_n_form::apply(*stock, in);
    auto cands = skill::lance_n_form::recognize(*out.workpiece);

    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["hinge_side"].get<std::string>(),
              std::string("-Y"));
    EXPECT_NEAR(cands[0].recovered_params["form_height_mm"].get<double>(), 2.5, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::lance_n_form::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. Specific: DFM-LNF-KERF rejects too-narrow kerf ─────────────────
TEST(SkillLanceNForm, ValidateRejectsTooNarrowKerf)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_n_form::Input in;
    in.tab_center_x_mm = 40.0;
    in.tab_center_y_mm = 40.0;
    in.tab_length_mm   = 10.0;
    in.tab_width_mm    = 6.0;
    in.form_height_mm  = 3.0;
    in.lance_kerf_mm   = 0.3;       // < 0.5 × 1.5 = 0.75
    in.hinge_side      = skill::lance_n_form::HingeSide::PlusY;

    auto r = skill::lance_n_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool kerfFound = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-LNF-KERF") { kerfFound = true; break; }
    EXPECT_TRUE(kerfFound);
}
