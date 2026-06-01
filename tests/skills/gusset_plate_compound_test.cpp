// @lat: [[process/test-strategy#compound macro]]
//
// gusset_plate_compound — COMPOUND skill: triangle body + bevel + 4 weld
// notches + 1 lift hole.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/gusset_plate_compound.hpp"

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

bool hasFinding(const skill::DFMReport& r, const std::string& code)
{
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}

skill::gusset_plate_compound::Input goodInput()
{
    skill::gusset_plate_compound::Input in;
    in.leg_a_mm         = 400.0;
    in.leg_b_mm         = 400.0;
    in.plate_t_mm       = 12.0;
    in.lift_hole_dia_mm = 25.0;
    in.bevel_mm         = 5.0;
    in.notch_r_mm       = 8.0;
    return in;
}

}  // namespace

// ─── 1. Apply: shape produced, volume substantially less than full triangle box ───
TEST(SkillGussetPlateCompound, ApplyProducesValidShapeWithMultipleSubFeatures)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    auto in = goodInput();
    auto out = skill::gusset_plate_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Triangle area ~ 0.5 × a × b; volume ~ area × t.  Lift hole removes
    // π × r² × t, notches remove more. Final volume must be POSITIVE and LESS
    // than half the bounding cuboid's volume.
    const double triVol = 0.5 * in.leg_a_mm * in.leg_b_mm * in.plate_t_mm;
    const double vol    = volumeOf(out.workpiece->shape());
    EXPECT_GT(vol, 0.0);
    EXPECT_LT(vol, triVol);

    // Compound flag and at least 2 sub-features.
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillGussetPlateCompound, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);

    // Bad: plate_t too thin.
    auto in1 = goodInput();
    in1.plate_t_mm = 4.0;
    auto r1 = skill::gusset_plate_compound::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-GUSSET-THK"));
    EXPECT_THROW(skill::gusset_plate_compound::apply(*stock, in1),
                 skill::SkillError);

    // Bad: lift hole below shackle minimum.
    auto in2 = goodInput();
    in2.lift_hole_dia_mm = 10.0;
    auto r2 = skill::gusset_plate_compound::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-GUSSET-LIFT-D"));
}

// ─── 3. Signature records compound kind ───────────────────────────────────
TEST(SkillGussetPlateCompound, SignatureRecordsCompoundKind)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::gusset_plate_compound::apply(*stock, goodInput());

    EXPECT_EQ(out.signature.skill_id, std::string("gusset_plate_compound"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("gusset_plate_compound"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_GE(out.signature.pattern["subfeature_count"].get<int>(), 2);

    EXPECT_NEAR(out.signature.params["leg_a_mm"].get<double>(),         400.0, 1e-9);
    EXPECT_NEAR(out.signature.params["leg_b_mm"].get<double>(),         400.0, 1e-9);
    EXPECT_NEAR(out.signature.params["lift_hole_dia_mm"].get<double>(),  25.0, 1e-9);
}

// ─── 4. Recognize: metadata replay returns confidence 1.0 ────────────────
TEST(SkillGussetPlateCompound, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto out = skill::gusset_plate_compound::apply(*stock, goodInput());

    auto cands = skill::gusset_plate_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);

    // Strip metadata → geometric fallback may still recognize the gusset
    // (slice-9 work-2 added geom-walk recognize).  Contract: any candidate
    // returned must carry confidence < 1.0 (NOT a metadata replay) and the
    // recovered lift_hole_dia_mm must be within 1 mm of the true 25 mm.
    skill::Workpiece raw(out.workpiece->shape());
    auto candsRaw = skill::gusset_plate_compound::recognize(raw);
    for (const auto& c : candsRaw) {
        EXPECT_LT(c.confidence, 1.0)
            << "raw-geometry candidate must NOT claim metadata-level confidence";
        EXPECT_NEAR(c.recovered_params["lift_hole_dia_mm"].get<double>(),
                    25.0, 1.0);
    }
}

// ─── 5. SPECIFIC: hypotenuse = √(a² + b²) recorded in pattern ─────────────
TEST(SkillGussetPlateCompound, HypotenuseMatchesPythagoras)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 100.0);
    auto in = goodInput();
    in.leg_a_mm = 300.0;
    in.leg_b_mm = 400.0;
    auto out = skill::gusset_plate_compound::apply(*stock, in);

    const double hypExpected = std::sqrt(300.0 * 300.0 + 400.0 * 400.0); // 500
    EXPECT_NEAR(out.signature.pattern["hypotenuse_mm"].get<double>(),
                hypExpected, 1e-9);

    // notch_count present and > 0
    EXPECT_GT(out.signature.pattern["notch_count"].get<int>(), 0);
}
