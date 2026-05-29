// @lat: [[process/test-strategy#skill round-trip]]
//
// closed_die_forge — metadata-only impression-die forging.  Geometry is
// supplied by the input workpiece (presumed already shaped by the die
// cavity); the skill stamps a signature recording die identity + flash.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/closed_die_forge.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {
double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}
bool hasFinding(const skill::DFMReport& r, const std::string& code) {
    for (const auto& f : r.findings) if (f.code == code) return true;
    return false;
}
}  // namespace

// ── 1. ApplyHandlesInput — geometry is NOT changed ──────────────────────
TEST(SkillClosedDieForge, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::closed_die_forge::Input in;
    in.shape_id           = "watch_caseback_v3";
    in.flash_thickness_mm = 1.2;

    auto out = skill::closed_die_forge::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("closed_die_forge"));
}

// ── 2. ValidateRejectsBadInput — empty shape_id / negative flash ────────
TEST(SkillClosedDieForge, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::closed_die_forge::Input emptyId;
    emptyId.shape_id           = "";
    emptyId.flash_thickness_mm = 1.0;
    auto r1 = skill::closed_die_forge::validate(*stock, emptyId);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::closed_die_forge::apply(*stock, emptyId), skill::SkillError);

    skill::closed_die_forge::Input negFlash;
    negFlash.shape_id           = "die_x";
    negFlash.flash_thickness_mm = -0.5;
    auto r2 = skill::closed_die_forge::validate(*stock, negFlash);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::closed_die_forge::apply(*stock, negFlash), skill::SkillError);
}

// ── 3. SignatureRecordsKind + shape_id + flash_thickness ────────────────
TEST(SkillClosedDieForge, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::closed_die_forge::Input in;
    in.shape_id           = "bezel_die_42mm";
    in.flash_thickness_mm = 0.8;

    auto out = skill::closed_die_forge::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("closed_die_forge"));
    EXPECT_EQ(out.signature.pattern["shape_id"].get<std::string>(),
              std::string("bezel_die_42mm"));
    EXPECT_NEAR(out.signature.pattern["flash_thickness_mm"].get<double>(),
                0.8, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. RecognizeMetadataReplay — recovers shape_id ───────────────────────
TEST(SkillClosedDieForge, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::closed_die_forge::Input in;
    in.shape_id           = "gear_blank_27t";
    in.flash_thickness_mm = 1.5;

    auto out = skill::closed_die_forge::apply(*stock, in);
    auto cands = skill::closed_die_forge::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["shape_id"].get<std::string>(),
              std::string("gear_blank_27t"));
    EXPECT_NEAR(cands[0].recovered_params["flash_thickness_mm"].get<double>(),
                1.5, 1e-9);

    // Stripped of metadata → no recognition.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::closed_die_forge::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. SPECIFIC: excessive flash thickness emits DFM-CDF-FLASH warning ──
TEST(SkillClosedDieForge, ExcessiveFlashWarning)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::closed_die_forge::Input in;
    in.shape_id           = "die_y";
    in.flash_thickness_mm = 7.0;       // > 5 mm → warning

    auto r = skill::closed_die_forge::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "excessive flash is a warning, not an error";
    EXPECT_TRUE(hasFinding(r, "DFM-CDF-FLASH"));
    EXPECT_NO_THROW(skill::closed_die_forge::apply(*stock, in));
}
