// @lat: [[process/test-strategy#skill round-trip]]
//
// thread_grind skill tests (slice — precision finishing variant).
//   1. DFM-GRIND-PARENT rejects without prior thread.
//   2. With a parent thread, DFM passes.
//   3. DFM-GRIND-AMOUNT rejects out-of-range removal.
//   4. DFM-GRIND-MIN rejects too-small thread diameter.
//   5. Sub-tolerance removal records metadata-only geometry tag.
//   6. Above-tolerance removal records helical_swept (or metadata fallback)
//      and references the parent thread.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/thread_grind.hpp"
#include "skills/thread_mill_external.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

// Build a workpiece with an existing external thread that thread_grind
// can finish.
std::shared_ptr<skill::Workpiece> makeThreadedStock()
{
    auto stock = skill::createCylindricalStock(8.0, 25.0);

    skill::thread_mill_external::Input mill;
    mill.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    mill.thread_size      = "M6";
    mill.pitch_mm         = 1.0;
    mill.thread_length_mm = 8.0;
    mill.tool_dia_mm      = 1.5;
    auto threaded = skill::thread_mill_external::apply(*stock, mill);
    return threaded.workpiece;
}
}  // namespace

// ── 1. DFM-GRIND-PARENT rejects without prior thread ─────────────────────
TEST(SkillThreadGrind, ValidateRejectsMissingParent)
{
    auto stock = skill::createCylindricalStock(8.0, 25.0);

    skill::thread_grind::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 8.0;
    in.removal_mm       = 0.01;
    in.is_external      = true;

    auto r = skill::thread_grind::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GRIND-PARENT") found = true;
    EXPECT_TRUE(found);
    EXPECT_THROW(skill::thread_grind::apply(*stock, in), skill::SkillError);
}

// ── 2. With a parent thread, DFM passes ──────────────────────────────────
TEST(SkillThreadGrind, ValidatePassesWithParent)
{
    auto threaded = makeThreadedStock();

    skill::thread_grind::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 8.0;
    in.removal_mm       = 0.01;
    in.is_external      = true;

    auto r = skill::thread_grind::validate(*threaded, in);
    EXPECT_TRUE(r.passed);
}

// ── 3. DFM-GRIND-AMOUNT rejects out-of-range removal ─────────────────────
TEST(SkillThreadGrind, ValidateRejectsOutOfRangeRemoval)
{
    auto threaded = makeThreadedStock();

    skill::thread_grind::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 8.0;
    in.is_external      = true;

    // Too small
    in.removal_mm = 0.0005;
    {
        auto r = skill::thread_grind::validate(*threaded, in);
        EXPECT_FALSE(r.passed);
    }
    // Too large
    in.removal_mm = 0.1;
    {
        auto r = skill::thread_grind::validate(*threaded, in);
        EXPECT_FALSE(r.passed);
    }
}

// ── 4. DFM-GRIND-MIN rejects too-small thread diameter ───────────────────
TEST(SkillThreadGrind, ValidateRejectsTooSmallThread)
{
    // Build a small workpiece with a prior thread (cheat by mocking via
    // thread_mill_external on a small cylinder — DFM-EXT-MIN would reject
    // it, so we craft the feature manually via Workpiece::addFeature).
    auto stock = skill::createCylindricalStock(4.0, 10.0);
    // Manually inject a parent thread feature so thread_grind validation
    // proceeds past DFM-GRIND-PARENT.
    {
        skill::FeatureSignature parent;
        parent.skill_id  = "tap_thread";
        parent.params    = { { "thread_size", "M4" }, { "pitch_mm", 0.7 } };
        parent.pattern   = { { "kind", "tap_thread" } };
        stock->addFeature(parent);
    }

    skill::thread_grind::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M4";
    in.pitch_mm         = 0.7;
    in.thread_length_mm = 4.0;
    in.removal_mm       = 0.01;
    in.is_external      = true;

    auto r = skill::thread_grind::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-GRIND-MIN") found = true;
    EXPECT_TRUE(found);
}

// ── 5. Sub-tolerance removal records metadata-only ───────────────────────
TEST(SkillThreadGrind, SubToleranceIsMetadataOnly)
{
    auto threaded = makeThreadedStock();
    const double volBefore = volumeOf(threaded->shape());

    skill::thread_grind::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 8.0;
    in.removal_mm       = 0.002;   // < kSubToleranceRemovalMm (0.005)
    in.is_external      = true;

    auto out = skill::thread_grind::apply(*threaded, in);

    // Shape unchanged because removal is below modeling tolerance.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-6);
    EXPECT_EQ(out.signature.pattern.at("geometry").get<std::string>(),
              std::string("sub_tolerance_metadata_only"));
    // Parent thread reference recorded.
    EXPECT_GE(out.signature.pattern.at("parent_thread_id").get<int>(), 0);
}

// ── 6. Above-tolerance removal: helical sweep or metadata ────────────────
TEST(SkillThreadGrind, AboveToleranceAttemptsSweep)
{
    auto threaded = makeThreadedStock();
    const double volBefore = volumeOf(threaded->shape());

    skill::thread_grind::Input in;
    in.cylinder_datum = skill::FaceCylinderByAxis{
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)), 5.0
    };
    in.thread_size      = "M6";
    in.pitch_mm         = 1.0;
    in.thread_length_mm = 8.0;
    in.removal_mm       = 0.02;
    in.is_external      = true;

    auto out = skill::thread_grind::apply(*threaded, in);

    EXPECT_LE(volumeOf(out.workpiece->shape()), volBefore + 1e-6);
    const std::string geom = out.signature.pattern["geometry"].get<std::string>();
    EXPECT_TRUE(geom == "helical_swept" || geom == "metadata_only")
        << "unexpected geometry tag: " << geom;

    EXPECT_EQ(out.signature.skill_id, std::string("thread_grind"));
    EXPECT_GE(out.signature.pattern.at("parent_thread_id").get<int>(), 0);

    // Tooling metadata is grind-wheel-specific.
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("thread_grind_wheel"));
    EXPECT_EQ(out.signature.tooling.tool_material, std::string("cbn"));
    EXPECT_EQ(out.signature.tooling.flute_count, 0);  // grinding wheel, not flutes
}
