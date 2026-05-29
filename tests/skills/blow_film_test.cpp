// @lat: [[process/test-strategy#skill round-trip]]
//
// blow_film skill — 5-case sweep: identity-geometry, DFM rejection of BUR
// outside [1.5, 4.0], signature metadata, metadata-only recognition, and
// the die_dia = bubble_dia / BUR derivation check.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/blow_film.hpp"

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

skill::blow_film::Input goodInput()
{
    skill::blow_film::Input in;
    in.bubble_dia_mm    = 400.0;
    in.layflat_width_mm = 628.0;     // ~ π × 400 / 2
    in.bur              = 2.5;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillBlowFilm, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::blow_film::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("blow_film"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (BUR < 1.5, then > 4.0) ────────────────
TEST(SkillBlowFilm, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bur = 1.0;
    auto r = skill::blow_film::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-BLOW-BUR"));
    EXPECT_THROW(skill::blow_film::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.bur = 5.0;
    auto r2 = skill::blow_film::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BLOW-BUR"));

    auto in3 = goodInput();
    in3.bubble_dia_mm = 0.0;
    auto r3 = skill::blow_film::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-BLOW-DIA"));
}

// ─── 3. Signature records kind + dia + layflat + bur ──────────────────────
TEST(SkillBlowFilm, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bubble_dia_mm    = 800.0;
    in.layflat_width_mm = 1256.0;   // π × 800 / 2 ≈ 1257
    in.bur              = 3.2;

    auto out = skill::blow_film::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("blow_film"));
    EXPECT_NEAR(out.signature.pattern["bubble_dia_mm"].get<double>(),
                800.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["layflat_width_mm"].get<double>(),
                1256.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["bur"].get<double>(), 3.2, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillBlowFilm, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::blow_film::apply(*stock, goodInput());

    auto cands = skill::blow_film::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["bur"].get<double>(), 2.5, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::blow_film::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "blow_film cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: die_dia (tool_dia_mm) = bubble_dia / BUR ────────────────
TEST(SkillBlowFilm, DieDiameterDerivedFromBubbleAndBur)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bubble_dia_mm = 600.0;
    in.bur           = 3.0;
    // For BUR consistency the layflat must remain π × dia / 2 within 10%.
    in.layflat_width_mm = 3.14159265 * 600.0 / 2.0;

    auto out = skill::blow_film::apply(*stock, in);

    EXPECT_NEAR(out.signature.tooling.tool_dia_mm,
                in.bubble_dia_mm / in.bur, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("blown_film_annular_die"));
    // expected_layflat_mm is the same π·dia/2 derivation.
    EXPECT_NEAR(out.signature.pattern["expected_layflat_mm"].get<double>(),
                3.14159265358979323846 * 600.0 / 2.0, 1e-6);
}
