// @lat: [[process/test-strategy#skill round-trip]]
//
// upsetting — axial compression with volume-preserving lateral spread.
// The workpiece becomes shorter (Z) and wider (XY).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/upsetting.hpp"

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

// ── 1. ApplyHandlesInput — shorter + wider with volume preserved ────────
TEST(SkillUpsetting, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 60.0);  // L0 along Z = 60
    const double volBefore = volumeOf(stock->shape());

    skill::upsetting::Input in;
    in.compression_ratio = 2.0;        // L1 = 30, lat scale = √2

    auto out = skill::upsetting::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Volume is preserved within float tolerance.
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-6);

    double xMin, yMin, zMin, xMax, yMax, zMax;
    out.workpiece->boundingBox(xMin, yMin, zMin, xMax, yMax, zMax);
    EXPECT_NEAR(zMax - zMin, 30.0, 1e-6);                     // L1 = 30
    EXPECT_NEAR(xMax - xMin, 40.0 * std::sqrt(2.0), 1e-6);    // XY × √2
    EXPECT_NEAR(yMax - yMin, 40.0 * std::sqrt(2.0), 1e-6);

    EXPECT_EQ(out.signature.skill_id, std::string("upsetting"));
}

// ── 2. ValidateRejectsBadInput — cratio <= 1 and buckling slenderness ──
TEST(SkillUpsetting, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 60.0);

    skill::upsetting::Input bad;
    bad.compression_ratio = 1.0;       // not > 1 → upsetting SHORTENS
    auto r1 = skill::upsetting::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::upsetting::apply(*stock, bad), skill::SkillError);

    // Slender column → buckle.
    auto slender = skill::createCuboidStock(20.0, 20.0, 200.0); // L/D = 10
    skill::upsetting::Input heavy;
    heavy.compression_ratio = 2.0;
    auto r2 = skill::upsetting::validate(*slender, heavy);
    EXPECT_FALSE(r2.passed);
    bool found = false;
    for (const auto& f : r2.findings)
        if (f.code == "DFM-UPS-BUCKLE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ── 3. SignatureRecordsKind + compression_ratio + lengths ───────────────
TEST(SkillUpsetting, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 60.0);

    skill::upsetting::Input in;
    in.compression_ratio = 2.5;

    auto out = skill::upsetting::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("upsetting"));
    EXPECT_NEAR(out.signature.params["compression_ratio"].get<double>(),
                2.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["original_length_mm"].get<double>(),
                60.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["final_length_mm"].get<double>(),
                24.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["lateral_scale"].get<double>(),
                std::sqrt(2.5), 1e-9);
    EXPECT_TRUE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ── 4. RecognizeMetadataReplay — exact recovery ─────────────────────────
TEST(SkillUpsetting, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 60.0);

    skill::upsetting::Input in;
    in.compression_ratio = 1.8;

    auto out = skill::upsetting::apply(*stock, in);
    auto cands = skill::upsetting::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["compression_ratio"].get<double>(),
                1.8, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::upsetting::recognize(raw);
    EXPECT_TRUE(cands2.empty());
}

// ── 5. SPECIFIC: heavy compression > 3.0 emits info finding ─────────────
TEST(SkillUpsetting, HeavyCompressionInfo)
{
    auto stock = skill::createCuboidStock(80.0, 80.0, 80.0);   // L/D = 1; safe

    skill::upsetting::Input in;
    in.compression_ratio = 4.0;        // > 3.0 → DFM-UPS-HEAVY info

    auto r = skill::upsetting::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "heavy compression should be info, not error (when L/D ok)";
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-UPS-HEAVY") { found = true; break; }
    EXPECT_TRUE(found);
    EXPECT_NO_THROW(skill::upsetting::apply(*stock, in));
}
