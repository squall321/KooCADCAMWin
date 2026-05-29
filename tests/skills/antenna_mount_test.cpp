// @lat: [[process/test-strategy#skill round-trip]]
//
// antenna_mount — 5-case sweep: identity-geometry synthesis, DFM gates on
// antenna_type / height / azimuth, metadata signature replay, azimuth-bound
// specific case.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/antenna_mount.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillAntennaMount, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::antenna_mount::Input in;
    in.antenna_type   = "panel";
    in.mount_height_m = 30.0;
    in.azimuth_deg    = 120.0;

    auto out = skill::antenna_mount::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("antenna_mount"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM: empty type, non-positive height, out-of-range azimuth ────────
TEST(SkillAntennaMount, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antenna_mount::Input bad;
    bad.antenna_type   = "";           // invalid
    bad.mount_height_m = 30.0;
    bad.azimuth_deg    = 0.0;
    auto r = skill::antenna_mount::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::antenna_mount::apply(*stock, bad), skill::SkillError);

    bad.antenna_type   = "panel";
    bad.mount_height_m = 0.0;          // invalid
    EXPECT_FALSE(skill::antenna_mount::validate(*stock, bad).passed);

    bad.mount_height_m = 30.0;
    bad.azimuth_deg    = 360.0;        // invalid: must be < 360
    auto r3 = skill::antenna_mount::validate(*stock, bad);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-ANT-AZIMUTH"));

    bad.azimuth_deg = -10.0;           // invalid: negative
    EXPECT_FALSE(skill::antenna_mount::validate(*stock, bad).passed);
}

// ── 3. Signature records kind + type/height/azimuth ──────────────────────
TEST(SkillAntennaMount, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antenna_mount::Input in;
    in.antenna_type   = "yagi";
    in.mount_height_m = 45.5;
    in.azimuth_deg    = 270.0;

    auto out = skill::antenna_mount::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("antenna_mount"));
    EXPECT_EQ(out.signature.pattern["antenna_type"].get<std::string>(),
              std::string("yagi"));
    EXPECT_NEAR(out.signature.pattern["mount_height_m"].get<double>(), 45.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["azimuth_deg"].get<double>(),   270.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("tower_rigging"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillAntennaMount, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antenna_mount::Input in;
    in.antenna_type   = "omni";
    in.mount_height_m = 20.0;
    in.azimuth_deg    = 0.0;

    auto out = skill::antenna_mount::apply(*stock, in);
    auto cands = skill::antenna_mount::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["antenna_type"].get<std::string>(),
              std::string("omni"));
    EXPECT_NEAR(cands[0].recovered_params["mount_height_m"].get<double>(),
                20.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::antenna_mount::recognize(raw).empty());
}

// ── 5. SPECIFIC: tall tower (> 100 m) emits ANT-HEIGHT warning ───────────
TEST(SkillAntennaMount, TallTowerEmitsHeightWarning)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::antenna_mount::Input in;
    in.antenna_type   = "panel";
    in.mount_height_m = 150.0;        // > 100 m threshold
    in.azimuth_deg    = 90.0;

    auto r = skill::antenna_mount::validate(*stock, in);
    EXPECT_TRUE(r.passed);              // warnings do not block
    EXPECT_TRUE(hasFinding(r, "DFM-ANT-HEIGHT"));
    EXPECT_NO_THROW(skill::antenna_mount::apply(*stock, in));
}
