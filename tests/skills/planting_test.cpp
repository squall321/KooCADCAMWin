// @lat: [[process/test-strategy#skill round-trip]]
//
// planting — 5-case sweep covering identity-geometry synthesis,
// DFM gates on density/depth, metadata signature replay.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/planting.hpp"

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

skill::planting::Input goodInput()
{
    skill::planting::Input in;
    in.seed_type           = "wheat";
    in.seed_density_per_m2 = 350.0;
    in.planting_depth_mm   = 30.0;
    return in;
}

}  // namespace

// ── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillPlanting, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::planting::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("planting"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects empty seed, zero density, zero depth, extreme depth ───
TEST(SkillPlanting, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto bad = goodInput();
    bad.seed_type = "";                  // invalid
    EXPECT_FALSE(skill::planting::validate(*stock, bad).passed);
    EXPECT_THROW(skill::planting::apply(*stock, bad), skill::SkillError);

    bad = goodInput();
    bad.seed_density_per_m2 = 0.0;       // invalid
    EXPECT_FALSE(skill::planting::validate(*stock, bad).passed);

    bad = goodInput();
    bad.planting_depth_mm = -1.0;        // invalid
    EXPECT_FALSE(skill::planting::validate(*stock, bad).passed);

    bad = goodInput();
    bad.planting_depth_mm = 500.0;       // > 200 mm → DFM error
    auto r = skill::planting::validate(*stock, bad);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-PLANT-DEPTH"));
}

// ── 3. Signature records kind + seed_type/density/depth ──────────────────
TEST(SkillPlanting, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.seed_type           = "soybean";
    in.seed_density_per_m2 = 60.0;
    in.planting_depth_mm   = 50.0;

    auto out = skill::planting::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("planting"));
    EXPECT_EQ(out.signature.pattern["seed_type"].get<std::string>(),
              std::string("soybean"));
    EXPECT_NEAR(out.signature.pattern["seed_density_per_m2"].get<double>(),
                60.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["planting_depth_mm"].get<double>(),
                50.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("seed_drill"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillPlanting, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::planting::apply(*stock, goodInput());

    auto cands = skill::planting::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["seed_type"].get<std::string>(),
              std::string("wheat"));
    EXPECT_NEAR(cands[0].recovered_params["planting_depth_mm"].get<double>(),
                30.0, 1e-9);

    // Strip metadata → empty.
    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::planting::recognize(raw).empty());
}

// ── 5. SPECIFIC: high seed density emits info finding but passes ─────────
TEST(SkillPlanting, HighDensityEmitsInfoButPasses)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.seed_density_per_m2 = 2500.0;          // > 2000 threshold

    auto r = skill::planting::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "high density should emit info-only finding, not block";
    EXPECT_TRUE(hasFinding(r, "DFM-PLANT-DENSITY"));

    EXPECT_NO_THROW(skill::planting::apply(*stock, in));
}
