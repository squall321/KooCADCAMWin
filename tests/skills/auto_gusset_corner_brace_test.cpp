// @lat: [[process/test-strategy#auto_gusset_corner_brace]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/auto_gusset_corner_brace.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
}

// Find an edge that has exactly 2 planar face neighbours — the first
// such edge will serve as the "corner".  For a cuboid, every edge has 2
// face neighbours; just take edge 0.
int firstCornerEdge(const skill::Workpiece& wp)
{
    return wp.edgeCount() > 0 ? 0 : -1;
}

skill::auto_gusset_corner_brace::Input goodInput(int eId)
{
    skill::auto_gusset_corner_brace::Input in;
    in.corner_edge_id  = eId;
    in.leg_length_mm   = 5.0;
    in.gusset_thick_mm = 3.0;
    in.service_class   = "medium";
    return in;
}

}  // namespace

// ─── T1: apply produces non-null shape with volume delta ──────────────────
TEST(SkillAutoGussetCornerBrace, ApplyAddsGusset)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    ASSERT_FALSE(stock->shape().IsNull());
    const double v0 = volumeOf(stock->shape());

    const int e = firstCornerEdge(*stock);
    ASSERT_GE(e, 0);

    auto in  = goodInput(e);
    auto out = skill::auto_gusset_corner_brace::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Gusset upper bound: leg² × t = 5×5×3 = 75 mm³.  Bead and relief are
    // small overlapping cylinders; total expected delta < 75 + bead − relief.
    const double leg = in.leg_length_mm;
    const double t   = in.gusset_thick_mm;
    const double vGusset = leg * leg * t;
    const double vBead   = M_PI * (t * 0.4) * (t * 0.4) * leg;
    const double rRelief = 1.0;
    const double vRelief = M_PI * rRelief * rRelief * (leg + 1.0);
    const double expected = vGusset + vBead - vRelief;

    const double actDelta = volumeOf(out.workpiece->shape()) - v0;
    // Loose bounds: actual could be smaller because of overlap with stock.
    EXPECT_LE(actDelta, expected * 1.20);
    EXPECT_GE(actDelta, -vRelief);
}

// ─── T2: validate rejects unknown service class; apply throws ─────────────
TEST(SkillAutoGussetCornerBrace, RejectsUnknownServiceClass)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    const int e = firstCornerEdge(*stock);
    auto in    = goodInput(e);
    in.service_class = "ultra";

    auto r = skill::auto_gusset_corner_brace::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GUSSET-SPEC") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::auto_gusset_corner_brace::apply(*stock, in),
                 skill::SkillError);
}

// ─── T3: signature carries service class + is_compound = true ────────────
TEST(SkillAutoGussetCornerBrace, SignatureCarriesServiceClassCompound)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    const int e = firstCornerEdge(*stock);
    auto in    = goodInput(e);
    auto out   = skill::auto_gusset_corner_brace::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("auto_gusset_corner_brace"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["service_class"].get<std::string>(),
              "medium");
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
}

// ─── T4: recognize returns ≥1 candidate ───────────────────────────────────
TEST(SkillAutoGussetCornerBrace, RecognizeAtLeastOne)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    const int e = firstCornerEdge(*stock);
    auto in    = goodInput(e);
    auto out   = skill::auto_gusset_corner_brace::apply(*stock, in);

    auto cands = skill::auto_gusset_corner_brace::recognize(*out.workpiece);
    ASSERT_FALSE(cands.empty());
    EXPECT_EQ(cands.front().skill_id,
              std::string("auto_gusset_corner_brace"));
    EXPECT_GT(cands.front().confidence, 0.0);
}

// ─── T5: context — flanking faces auto-detected, tip_radius from table ───
TEST(SkillAutoGussetCornerBrace, ContextDetectsFlankingFacesAndTipRadius)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 30.0);
    const int e = firstCornerEdge(*stock);
    auto in    = goodInput(e);
    auto out   = skill::auto_gusset_corner_brace::apply(*stock, in);

    ASSERT_TRUE(out.signature.pattern.contains("auto_detected_faces"));
    const auto faces = out.signature.pattern["auto_detected_faces"];
    EXPECT_TRUE(faces.is_array());
    EXPECT_EQ(faces.size(), 2u);
    // Service class "medium" → tip radius 1.0 mm per lookup table.
    EXPECT_NEAR(out.signature.pattern["tip_radius_mm"].get<double>(),
                1.0, 1e-6);
}
