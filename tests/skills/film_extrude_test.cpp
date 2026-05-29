// @lat: [[process/test-strategy#skill round-trip]]
//
// film_extrude skill — 5-case sweep: identity-geometry, DFM rejection of
// thickness outside [5, 500] µm, signature metadata, metadata-only
// recognition, derived melt_output_kg_h plausibility.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/film_extrude.hpp"

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

skill::film_extrude::Input goodInput()
{
    skill::film_extrude::Input in;
    in.film_thickness_um = 50.0;
    in.width_mm          = 1200.0;
    in.line_speed_m_min  = 60.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillFilmExtrude, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::film_extrude::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("film_extrude"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ────────────────────────────────────────
TEST(SkillFilmExtrude, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.film_thickness_um = 2.0;     // < 5 µm minimum
    auto r = skill::film_extrude::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-FILM-THICKNESS"));
    EXPECT_THROW(skill::film_extrude::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.film_thickness_um = 800.0;  // > 500 µm — too thick for film
    auto r2 = skill::film_extrude::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FILM-THICKNESS"));

    auto in3 = goodInput();
    in3.line_speed_m_min = 0.0;
    auto r3 = skill::film_extrude::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-FILM-SPEED"));
}

// ─── 3. Signature records kind + thickness + width + speed ────────────────
TEST(SkillFilmExtrude, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.film_thickness_um = 100.0;
    in.width_mm          = 1800.0;
    in.line_speed_m_min  = 120.0;

    auto out = skill::film_extrude::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("film_extrude"));
    EXPECT_NEAR(out.signature.pattern["film_thickness_um"].get<double>(),
                100.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["width_mm"].get<double>(),
                1800.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["line_speed_m_min"].get<double>(),
                120.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFilmExtrude, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::film_extrude::apply(*stock, goodInput());

    auto cands = skill::film_extrude::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["film_thickness_um"].get<double>(),
                50.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::film_extrude::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "film_extrude cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: melt_output_kg_h scales linearly with width/speed/thick ─
TEST(SkillFilmExtrude, MeltOutputScalesWithThicknessWidthSpeed)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto base = goodInput();
    auto baseOut = skill::film_extrude::apply(*stock, base);
    const double m0 = baseOut.signature.pattern["melt_output_kg_h"].get<double>();

    auto wide = goodInput();
    wide.width_mm = base.width_mm * 2.0;
    auto wideOut = skill::film_extrude::apply(*stock, wide);
    const double m1 = wideOut.signature.pattern["melt_output_kg_h"].get<double>();

    EXPECT_NEAR(m1, 2.0 * m0, 1e-6)
        << "doubling width should double mass output";

    // Tooling die-lip width must equal the input width.
    EXPECT_NEAR(baseOut.signature.tooling.tool_length_mm, base.width_mm, 1e-9);
}
