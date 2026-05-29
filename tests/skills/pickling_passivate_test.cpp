// @lat: [[process/test-strategy#skill round-trip]]
//
// pickling_passivate — surface chemical conditioning for stainless / nickel.
// Covers:
//   1. apply preserves geometry verbatim
//   2. validate rejects bad input (non-positive area)
//   3. signature records kind + area + acid_mix + passivation_time_h
//   4. recognize replays metadata
//   5. unknown acid_mix triggers DFM-PICK-MIX info but does NOT block

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/pickling_passivate.hpp"

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

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillPicklingPassivate, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "stainless_316L");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::pickling_passivate::Input in;
    in.surface_area_m2    = 12.0;
    in.acid_mix           = "HNO3+HF";
    in.passivation_time_h = 1.5;

    auto out = skill::pickling_passivate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. validate rejects bad input ───────────────────────────────────────
TEST(SkillPicklingPassivate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "stainless_316L");

    skill::pickling_passivate::Input in;
    in.surface_area_m2    = 0.0;     // invalid
    in.acid_mix           = "HNO3+HF";
    in.passivation_time_h = 2.0;

    auto r = skill::pickling_passivate::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::pickling_passivate::apply(*stock, in),
                 skill::SkillError);

    // Non-positive passivation time also blocks.
    in.surface_area_m2    = 10.0;
    in.passivation_time_h = 0.0;
    auto r2 = skill::pickling_passivate::validate(*stock, in);
    EXPECT_FALSE(r2.passed);
}

// ─── 3. signature records kind + key params ──────────────────────────────
TEST(SkillPicklingPassivate, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "stainless_316L");

    skill::pickling_passivate::Input in;
    in.surface_area_m2    = 25.0;
    in.acid_mix           = "HNO3";
    in.passivation_time_h = 2.5;

    auto out = skill::pickling_passivate::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("pickling_passivate"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("pickling_passivate"));
    EXPECT_NEAR(out.signature.pattern["surface_area_m2"].get<double>(),
                25.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["acid_mix"].get<std::string>(),
              std::string("HNO3"));
    EXPECT_NEAR(out.signature.pattern["passivation_time_h"].get<double>(),
                2.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["expected_surface"].get<std::string>(),
              std::string("chromium_oxide_passive"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. recognize replays metadata ───────────────────────────────────────
TEST(SkillPicklingPassivate, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "stainless_316L");

    skill::pickling_passivate::Input in;
    in.surface_area_m2    = 18.0;
    in.acid_mix           = "citric";
    in.passivation_time_h = 1.0;
    auto out = skill::pickling_passivate::apply(*stock, in);

    auto cands = skill::pickling_passivate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["acid_mix"].get<std::string>(),
              std::string("citric"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::pickling_passivate::recognize(raw).empty());
}

// ─── 5. Unknown acid_mix emits info but does NOT block ───────────────────
TEST(SkillPicklingPassivate, UnknownAcidMixEmitsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "stainless_316L");

    skill::pickling_passivate::Input in;
    in.surface_area_m2    = 10.0;
    in.acid_mix           = "exotic_brew";
    in.passivation_time_h = 1.5;

    auto r = skill::pickling_passivate::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "unknown acid_mix should emit info, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-PICK-MIX"));
    EXPECT_NO_THROW(skill::pickling_passivate::apply(*stock, in));
}
