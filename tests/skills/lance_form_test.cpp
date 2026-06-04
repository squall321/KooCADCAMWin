// @lat: [[process/test-strategy#skill round-trip]]
//
// lance_form — lance-and-form (slit + raised tab).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/lance_form.hpp"

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

// ── 1. Apply: slit + raised tab on a 100×80×1.5 sheet ──────────────────
TEST(SkillLanceForm, ApplyProducesValidSolid)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_form::Input in;
    in.slit_start_xy  = { 30.0, 40.0 };
    in.slit_end_xy    = { 50.0, 40.0 };    // slit length 20 mm > 5·1.5
    in.form_height_mm = 3.0;               // 2·t
    in.form_width_mm  = 5.0;               // > 2·t

    auto out = skill::lance_form::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("lance_form"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["sheet_feature_type"].get<std::string>(),
              std::string("lance_and_form"));
    EXPECT_NEAR(out.signature.pattern["slit_length_mm"].get<double>(),
                20.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["form_height_mm"].get<double>(),
                3.0, 1e-6);
}

// ── 2. DFM-L-SLIT-LEN: too short slit rejected ─────────────────────────
TEST(SkillLanceForm, ValidateRejectsTooShortSlit)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_form::Input in;
    in.slit_start_xy  = { 30.0, 40.0 };
    in.slit_end_xy    = { 35.0, 40.0 };     // 5 mm = 5·t, not > 5·t
    in.form_height_mm = 3.0;
    in.form_width_mm  = 5.0;

    auto r = skill::lance_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-L-SLIT-LEN") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::lance_form::apply(*stock, in), skill::SkillError);
}

// ── 3. DFM-L-HEIGHT: form too tall ─────────────────────────────────────
TEST(SkillLanceForm, ValidateRejectsTooTallForm)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_form::Input in;
    in.slit_start_xy  = { 30.0, 40.0 };
    in.slit_end_xy    = { 50.0, 40.0 };
    in.form_height_mm = 10.0;        // > 4·1.5 = 6.0
    in.form_width_mm  = 5.0;

    auto r = skill::lance_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-L-HEIGHT") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 4. DFM-L-WIDTH: form width too narrow ──────────────────────────────
TEST(SkillLanceForm, ValidateRejectsTooNarrowForm)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_form::Input in;
    in.slit_start_xy  = { 30.0, 40.0 };
    in.slit_end_xy    = { 50.0, 40.0 };
    in.form_height_mm = 3.0;
    in.form_width_mm  = 2.0;          // < 2·1.5 = 3.0

    auto r = skill::lance_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-L-WIDTH") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: raised tab detected after apply ──────────────────────
TEST(SkillLanceForm, RecognizeDetectsRaisedTab)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::lance_form::Input in;
    in.slit_start_xy  = { 30.0, 40.0 };
    in.slit_end_xy    = { 50.0, 40.0 };
    in.form_height_mm = 3.0;
    in.form_width_mm  = 5.0;

    auto out = skill::lance_form::apply(*stock, in);
    auto cands = skill::lance_form::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    const auto& c = cands.front();
    EXPECT_EQ(c.skill_id, std::string("lance_form"));
    EXPECT_NEAR(c.recovered_params["form_height_mm"].get<double>(),
                3.0, 0.5);
}
