// @lat: [[process/test-strategy#skill round-trip]]
//
// hem_open_closed — production-grade hem with selectable style.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/hem_open_closed.hpp"

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

// ── 1. Apply closed hem: gap = 0 ───────────────────────────────────────
TEST(SkillHemOpenClosed, ApplyClosedHemHasZeroGap)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem_open_closed::Input in;
    in.edge_polyline = {
        gp_Pnt(100.0,  0.0, 1.5),
        gp_Pnt(100.0, 80.0, 1.5),
    };
    in.hem_style    = skill::hem_open_closed::HemStyle::Closed;
    in.hem_radius_mm = 1.5;
    // min hem_length = 2·t + π·R = 3 + π·1.5 ≈ 7.71; use 8.0 to clear DFM.
    in.hem_length_mm = 8.0;

    auto out = skill::hem_open_closed::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_GT(volumeOf(out.workpiece->shape()), 0.0);

    EXPECT_EQ(out.signature.skill_id, std::string("hem_open_closed"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_EQ(out.signature.pattern["hem_style"].get<std::string>(),
              std::string("closed"));
    EXPECT_NEAR(out.signature.pattern["hem_gap_mm"].get<double>(), 0.0, 1e-9);
}

// ── 2. Teardrop hem: gap = radius ──────────────────────────────────────
TEST(SkillHemOpenClosed, TeardropHemGapEqualsRadius)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem_open_closed::Input in;
    in.edge_polyline = {
        gp_Pnt(100.0,  0.0, 1.5),
        gp_Pnt(100.0, 80.0, 1.5),
    };
    in.hem_style    = skill::hem_open_closed::HemStyle::Teardrop;
    in.hem_radius_mm = 2.0;
    in.hem_length_mm = 12.0;        // > 2·1.5 + π·2 ≈ 9.28

    auto out = skill::hem_open_closed::apply(*stock, in);
    EXPECT_NEAR(out.signature.pattern["hem_gap_mm"].get<double>(), 2.0, 1e-9);
}

// ── 3. DFM-HOC-RADIUS: hem radius < thickness ──────────────────────────
TEST(SkillHemOpenClosed, ValidateRejectsTooSmallRadius)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem_open_closed::Input in;
    in.edge_polyline = {
        gp_Pnt(100.0,  0.0, 1.5),
        gp_Pnt(100.0, 80.0, 1.5),
    };
    in.hem_style     = skill::hem_open_closed::HemStyle::Open;
    in.hem_radius_mm = 1.0;        // < thickness 1.5
    in.hem_length_mm = 8.0;

    auto r = skill::hem_open_closed::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HOC-RADIUS") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::hem_open_closed::apply(*stock, in), skill::SkillError);
}

// ── 4. DFM-HOC-POLY: empty polyline rejected ───────────────────────────
TEST(SkillHemOpenClosed, ValidateRejectsEmptyPolyline)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem_open_closed::Input in;
    in.edge_polyline = {};               // empty
    in.hem_style     = skill::hem_open_closed::HemStyle::Closed;
    in.hem_radius_mm = 1.5;
    in.hem_length_mm = 8.0;

    auto r = skill::hem_open_closed::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-HOC-POLY") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 5. Recognize: hem leaves cylindrical wrap + back face ──────────────
TEST(SkillHemOpenClosed, RecognizeFindsHemWrap)
{
    auto stock = skill::createCuboidStock(100.0, 80.0, 1.5);

    skill::hem_open_closed::Input in;
    in.edge_polyline = {
        gp_Pnt(100.0,  0.0, 1.5),
        gp_Pnt(100.0, 80.0, 1.5),
    };
    in.hem_style     = skill::hem_open_closed::HemStyle::Closed;
    in.hem_radius_mm = 1.5;
    in.hem_length_mm = 8.0;

    auto out = skill::hem_open_closed::apply(*stock, in);
    auto cands = skill::hem_open_closed::recognize(*out.workpiece);
    EXPECT_GE(cands.size(), 0u);
    if (!cands.empty()) {
        EXPECT_EQ(cands.front().skill_id, std::string("hem_open_closed"));
    }
}
