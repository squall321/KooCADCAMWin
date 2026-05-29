// @lat: [[process/test-strategy#skill round-trip]]
//
// wear_inspect skill — 5-case sweep:
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKind+params
//   4. RecognizeMetadataReplay
//   5. Specific: depth > threshold ⇒ accept_pass=false and DFM-WEAR-FAIL info

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wear_inspect.hpp"

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

}  // namespace

// ─── 1. Apply: geometry COMPLETELY unchanged ─────────────────────────────
TEST(SkillWearInspect, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::wear_inspect::Input in;
    in.wear_depth_um       = 40.0;
    in.wear_pattern        = "uniform";
    in.accept_threshold_um = 100.0;

    auto out = skill::wear_inspect::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("wear_inspect"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ────────────────────────────────────────
TEST(SkillWearInspect, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::wear_inspect::Input badDepth;
    badDepth.wear_depth_um       = -1.0;
    badDepth.accept_threshold_um = 100.0;
    EXPECT_FALSE(skill::wear_inspect::validate(*stock, badDepth).passed);
    EXPECT_THROW(skill::wear_inspect::apply(*stock, badDepth),
                 skill::SkillError);

    skill::wear_inspect::Input badThreshold;
    badThreshold.wear_depth_um       = 20.0;
    badThreshold.accept_threshold_um = 0.0;
    EXPECT_FALSE(skill::wear_inspect::validate(*stock, badThreshold).passed);
    EXPECT_THROW(skill::wear_inspect::apply(*stock, badThreshold),
                 skill::SkillError);
}

// ─── 3. Signature records kind + params ──────────────────────────────────
TEST(SkillWearInspect, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::wear_inspect::Input in;
    in.wear_depth_um       = 65.0;
    in.wear_pattern        = "galling";
    in.accept_threshold_um = 120.0;

    auto out = skill::wear_inspect::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("wear_inspect"));
    EXPECT_NEAR(out.signature.pattern["wear_depth_um"].get<double>(), 65.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["wear_pattern"].get<std::string>(),
              std::string("galling"));
    EXPECT_NEAR(out.signature.pattern["accept_threshold_um"].get<double>(),
                120.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern["accept_pass"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_NEAR(out.signature.params["wear_depth_um"].get<double>(), 65.0, 1e-6);
    EXPECT_EQ(out.signature.params["wear_pattern"].get<std::string>(),
              std::string("galling"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillWearInspect, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::wear_inspect::Input in;
    in.wear_depth_um       = 50.0;
    in.wear_pattern        = "abrasive";
    in.accept_threshold_um = 100.0;
    auto out = skill::wear_inspect::apply(*stock, in);

    auto cands = skill::wear_inspect::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["wear_pattern"].get<std::string>(),
              std::string("abrasive"));

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::wear_inspect::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ─── 5. Specific: depth > threshold ⇒ accept_pass=false + WEAR-FAIL info ─
TEST(SkillWearInspect, ExceedanceMarksAcceptFailAndEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::wear_inspect::Input in;
    in.wear_depth_um       = 200.0;
    in.wear_pattern        = "uniform";
    in.accept_threshold_um = 100.0;

    auto r = skill::wear_inspect::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "exceedance is info-only, should not block";
    EXPECT_TRUE(hasFinding(r, "DFM-WEAR-FAIL"));

    auto out = skill::wear_inspect::apply(*stock, in);
    EXPECT_FALSE(out.signature.pattern["accept_pass"].get<bool>());
}
