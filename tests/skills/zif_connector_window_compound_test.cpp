// @lat: [[process/test-strategy#zif_connector_window_compound]]
//
// zif_connector_window_compound — ZIF flex connector through-window + 2 side
// locking-tab recesses (5 tests).
//
// Cases:
//   1. ApplyRemovesVolumeMatchesExpected
//   2. ValidateRejectsZeroDimensions
//   3. ValidateRejectsTooSmallWindowWidth (< 0.3 mm)
//   4. SignatureCarriesCompoundAndPhoneFlags
//   5. RecognizeReplaysHistory

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/zif_connector_window_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFace(const skill::Workpiece& wp)
{
    int bestId = -1; double bestArea = 0.0;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        gp_Dir n; try { n = wp.faceNormal(i); } catch (...) { continue; }
        if (n.Z() < 0.9) continue;
        const double a = wp.faceArea(i);
        if (a > bestArea) { bestArea = a; bestId = i; }
    }
    return bestId;
}

skill::zif_connector_window_compound::Input good(int faceId)
{
    skill::zif_connector_window_compound::Input in;
    in.face_id              = faceId;
    in.center_x_mm          = 30.0;
    in.center_y_mm          = 20.0;
    in.window_length_mm     = 10.0;
    in.window_width_mm      = 1.6;
    in.window_depth_mm      = 1.0;
    in.locking_tab_width_mm = 1.0;
    in.locking_tab_depth_mm = 0.5;
    return in;
}

}  // namespace

// ─── 1. Volume removed within expected envelope ──────────────────────────
TEST(SkillZifConnectorWindowCompound, ApplyRemovesVolumeMatchesExpected)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 4.0);
    const int topId = findTopFace(*stock);
    ASSERT_GE(topId, 0);

    const double v0 = volumeOf(stock->shape());
    auto in = good(topId);
    auto out = skill::zif_connector_window_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());
    const double removed = v0 - v1;

    // Min: through-window L × W × stock_depth (4 mm) = 10×1.6×4 = 64 mm³.
    // Max: through-window + 2 tabs at full pocket extent (rough upper bound).
    const double through = in.window_length_mm * in.window_width_mm * 4.0;
    EXPECT_GT(removed, through * 0.9);
    EXPECT_LT(removed, through * 2.0);  // tabs add a little, not 2x
}

// ─── 2. Validate rejects zero dimensions ─────────────────────────────────
TEST(SkillZifConnectorWindowCompound, ValidateRejectsZeroDimensions)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 4.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.window_length_mm = 0.0;
    auto r = skill::zif_connector_window_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::zif_connector_window_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects sub-0.3 mm window width ─────────────────────────
TEST(SkillZifConnectorWindowCompound, ValidateRejectsTooSmallWindowWidth)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 4.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    in.window_width_mm = 0.2;
    auto r = skill::zif_connector_window_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
}

// ─── 4. Signature carries compound + phone flags ─────────────────────────
TEST(SkillZifConnectorWindowCompound, SignatureCarriesCompoundAndPhoneFlags)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 4.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::zif_connector_window_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("zif_connector_window_compound"));
    EXPECT_TRUE(out.signature.pattern["is_compound"].get<bool>());
    EXPECT_TRUE(out.signature.pattern["is_phone_feature"].get<bool>());
    EXPECT_EQ(out.signature.pattern["phone_feature_type"].get<std::string>(),
              std::string("zif_window"));
    EXPECT_EQ(out.signature.pattern["subfeature_count"].get<int>(), 3);
    EXPECT_EQ(out.signature.pattern["locking_tab_count"].get<int>(), 2);
}

// ─── 5. Recognize replays history ────────────────────────────────────────
TEST(SkillZifConnectorWindowCompound, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(60.0, 40.0, 4.0);
    const int topId = findTopFace(*stock);
    auto in = good(topId);
    auto out = skill::zif_connector_window_compound::apply(*stock, in);

    auto cands = skill::zif_connector_window_compound::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_EQ(cands.front().skill_id,
              std::string("zif_connector_window_compound"));
    EXPECT_GE(cands.front().confidence, 0.9);
}
