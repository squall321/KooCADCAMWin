// @lat: [[process/test-strategy#skill round-trip]]
//
// tube_end_form — special end form (bell, beaded, expanded_slot); metadata-only.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/tube_end_form.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

}  // namespace

// ─── 1. ApplyHandlesInput — metadata-only, geometry preserved ────────────
TEST(SkillTubeEndForm, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::tube_end_form::Input in;
    in.form_type   = "bell";
    in.length_mm   = 8.0;
    in.diameter_mm = 28.0;          // > OD = 20
    in.end         = "z_max";
    auto out = skill::tube_end_form::apply(*stock, in);

    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("tube_end_form"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. ValidateRejectsBadInput — unknown form_type ──────────────────────
TEST(SkillTubeEndForm, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::tube_end_form::Input in;
    in.form_type   = "swiss_army";    // not in {bell, beaded, expanded_slot}
    in.length_mm   = 8.0;
    in.diameter_mm = 28.0;
    in.end         = "z_max";

    auto r = skill::tube_end_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-INPUT") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::tube_end_form::apply(*stock, in), skill::SkillError);
}

// ─── 3. SignatureRecordsKind + key params (incl. form_type enum) ─────────
TEST(SkillTubeEndForm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::tube_end_form::Input in;
    in.form_type   = "beaded";
    in.length_mm   = 5.0;
    in.diameter_mm = 24.0;
    in.end         = "z_min";
    auto out = skill::tube_end_form::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("tube_end_form"));
    EXPECT_EQ(out.signature.pattern["form_type"].get<std::string>(),
              std::string("beaded"));
    EXPECT_NEAR(out.signature.pattern["length_mm"].get<double>(), 5.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["diameter_mm"].get<double>(), 24.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["end"].get<std::string>(),
              std::string("z_min"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("end_form_die"));
}

// ─── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillTubeEndForm, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);

    skill::tube_end_form::Input in;
    in.form_type   = "expanded_slot";
    in.length_mm   = 6.0;
    in.diameter_mm = 22.0;
    in.end         = "z_max";
    auto out = skill::tube_end_form::apply(*stock, in);

    auto cands = skill::tube_end_form::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["form_type"].get<std::string>(),
              std::string("expanded_slot"));

    // Stripped of metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::tube_end_form::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: bell form with diameter ≤ current OD → DFM-TUBE-EF-DIA
TEST(SkillTubeEndForm, ValidateRejectsBellWithTooSmallDiameter)
{
    auto stock = skill::createCylindricalStock(20.0, 40.0);  // OD = 20

    skill::tube_end_form::Input in;
    in.form_type   = "bell";
    in.length_mm   = 5.0;
    in.diameter_mm = 18.0;          // ≤ current OD — bell must expand
    in.end         = "z_max";

    auto r = skill::tube_end_form::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-TUBE-EF-DIA") { found = true; break; }
    EXPECT_TRUE(found);
}
