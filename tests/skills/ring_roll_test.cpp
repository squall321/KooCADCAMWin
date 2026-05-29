// @lat: [[process/test-strategy#skill round-trip]]
//
// ring_roll — metadata-only ring rolling.  Geometry unchanged; the
// signature records target ID/OD/height and the derived id_od_ratio.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ring_roll.hpp"

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
TEST(SkillRingRoll, ApplyHandlesInput)
{
    auto stock = skill::createCylindricalStock(100.0, 30.0);   // OD=100, h=30
    const double volBefore = volumeOf(stock->shape());

    skill::ring_roll::Input in;
    in.target_id_mm     = 80.0;
    in.target_od_mm     = 120.0;
    in.target_height_mm = 25.0;

    auto out = skill::ring_roll::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.signature.skill_id, std::string("ring_roll"));
}

// ── 2. ValidateRejectsBadInput — OD <= ID and too-thin ratio ────────────
TEST(SkillRingRoll, ValidateRejectsBadInput)
{
    auto stock = skill::createCylindricalStock(100.0, 30.0);

    skill::ring_roll::Input flipped;
    flipped.target_id_mm     = 120.0;
    flipped.target_od_mm     = 80.0;            // OD < ID → invalid
    flipped.target_height_mm = 25.0;
    auto r1 = skill::ring_roll::validate(*stock, flipped);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::ring_roll::apply(*stock, flipped), skill::SkillError);

    skill::ring_roll::Input paperThin;
    paperThin.target_id_mm     = 99.0;
    paperThin.target_od_mm     = 100.0;         // ratio = 0.99 > 0.95 → error
    paperThin.target_height_mm = 25.0;
    auto r2 = skill::ring_roll::validate(*stock, paperThin);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-RR-RATIO"));
}

// ── 3. SignatureRecordsKind + ID/OD/height + ratio ──────────────────────
TEST(SkillRingRoll, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCylindricalStock(100.0, 30.0);

    skill::ring_roll::Input in;
    in.target_id_mm     = 60.0;
    in.target_od_mm     = 100.0;
    in.target_height_mm = 20.0;

    auto out = skill::ring_roll::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ring_roll"));
    EXPECT_NEAR(out.signature.pattern["target_id_mm"].get<double>(), 60.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_od_mm"].get<double>(), 100.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["target_height_mm"].get<double>(),
                20.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["id_od_ratio"].get<double>(),
                0.60, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. RecognizeMetadataReplay ──────────────────────────────────────────
TEST(SkillRingRoll, RecognizeMetadataReplay)
{
    auto stock = skill::createCylindricalStock(100.0, 30.0);

    skill::ring_roll::Input in;
    in.target_id_mm     = 70.0;
    in.target_od_mm     = 110.0;
    in.target_height_mm = 25.0;

    auto out = skill::ring_roll::apply(*stock, in);
    auto cands = skill::ring_roll::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_id_mm"].get<double>(),
                70.0, 1e-9);
    EXPECT_NEAR(cands[0].recovered_params["target_od_mm"].get<double>(),
                110.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::ring_roll::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. SPECIFIC: column ring (height > OD) emits DFM-RR-HEIGHT warning ─
TEST(SkillRingRoll, ColumnRingHeightWarning)
{
    auto stock = skill::createCylindricalStock(100.0, 30.0);

    skill::ring_roll::Input in;
    in.target_id_mm     = 50.0;
    in.target_od_mm     = 80.0;
    in.target_height_mm = 120.0;       // > OD → column-ring warning

    auto r = skill::ring_roll::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "column-ring height should be a warning, not an error";
    EXPECT_TRUE(hasFinding(r, "DFM-RR-HEIGHT"));
    EXPECT_NO_THROW(skill::ring_roll::apply(*stock, in));
}
