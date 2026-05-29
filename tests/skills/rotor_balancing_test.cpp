// @lat: [[process/test-strategy#skill round-trip]]
//
// rotor_balancing skill — 5-case sweep covering identity-geometry synthesis,
// DFM error on non-positive imbalance, ISO 1940 grade lookup, metadata
// replay recognition.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/rotor_balancing.hpp"

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

skill::rotor_balancing::Input goodInput()
{
    skill::rotor_balancing::Input in;
    in.mass_imbalance_g_mm    = 5.0;
    in.balance_grade_iso_1940 = "G2.5";
    in.rotation_speed_rpm     = 10000.0;
    in.correction_plane_count = 2;
    in.rotor_mass_kg          = 5.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillRotorBalancing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "steel");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::rotor_balancing::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("rotor_balancing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (zero imbalance, zero rpm) ────────────
TEST(SkillRotorBalancing, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "steel");

    auto in = goodInput();
    in.mass_imbalance_g_mm = 0.0;       // not > 0
    auto r = skill::rotor_balancing::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::rotor_balancing::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.rotation_speed_rpm = 0.0;       // not > 0
    auto r2 = skill::rotor_balancing::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + imbalance / grade / rpm ─────────────────
TEST(SkillRotorBalancing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "steel");

    auto in = goodInput();
    in.mass_imbalance_g_mm    = 12.0;
    in.balance_grade_iso_1940 = "G6.3";
    in.rotation_speed_rpm     = 6000.0;
    in.correction_plane_count = 1;

    auto out = skill::rotor_balancing::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("rotor_balancing"));
    EXPECT_NEAR(out.signature.pattern["mass_imbalance_g_mm"].get<double>(),
                12.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["balance_grade_iso_1940"].get<std::string>(),
              std::string("G6.3"));
    EXPECT_NEAR(out.signature.pattern["rotation_speed_rpm"].get<double>(),
                6000.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["correction_plane_count"].get<int>(), 1);
    EXPECT_EQ(out.signature.pattern["balance_type"].get<std::string>(),
              std::string("static"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillRotorBalancing, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "steel");
    auto out = skill::rotor_balancing::apply(*stock, goodInput());

    auto cands = skill::rotor_balancing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["balance_grade_iso_1940"].get<std::string>(),
              std::string("G2.5"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::rotor_balancing::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "rotor_balancing cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: ISO 1940 grade envelope lookup populated ────────────────
TEST(SkillRotorBalancing, GradeEnvelopeIsCarriedInPattern)
{
    auto stock = skill::createCylindricalStock(80.0, 20.0, "steel");

    auto inFine = goodInput();
    inFine.balance_grade_iso_1940 = "G1";
    auto outFine = skill::rotor_balancing::apply(*stock, inFine);
    EXPECT_NEAR(outFine.signature.pattern["grade_envelope_mm_s"].get<double>(),
                1.0, 1e-9);

    auto inCoarse = goodInput();
    inCoarse.balance_grade_iso_1940 = "G16";
    auto outCoarse = skill::rotor_balancing::apply(*stock, inCoarse);
    EXPECT_NEAR(outCoarse.signature.pattern["grade_envelope_mm_s"].get<double>(),
                16.0, 1e-9);

    // Unknown grade emits a warning but does not block.
    auto inBogus = goodInput();
    inBogus.balance_grade_iso_1940 = "G_UNKNOWN";
    auto rep = skill::rotor_balancing::validate(*stock, inBogus);
    EXPECT_TRUE(rep.passed);
    EXPECT_TRUE(hasFinding(rep, "DFM-BAL-GRADE"));
}
