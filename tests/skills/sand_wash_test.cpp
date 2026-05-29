// @lat: [[process/test-strategy#skill round-trip]]
//
// sand_wash skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection on non-positive throughput, metadata-only recognition,
// and out-of-range fineness modulus surfacing as info (non-blocking).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sand_wash.hpp"

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

skill::sand_wash::Input goodInput()
{
    skill::sand_wash::Input in;
    in.throughput_kg_h        = 20000.0;
    in.final_fineness_modulus = 2.7;
    in.water_l_kg             = 1.2;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillSandWash, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::sand_wash::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("sand_wash"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (throughput = 0) ──────────────────────
TEST(SkillSandWash, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.throughput_kg_h = 0.0;

    auto r = skill::sand_wash::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::sand_wash::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + throughput/FM/water ─────────────────────
TEST(SkillSandWash, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.throughput_kg_h        = 35000.0;
    in.final_fineness_modulus = 3.0;
    in.water_l_kg             = 1.5;

    auto out = skill::sand_wash::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sand_wash"));
    EXPECT_NEAR(out.signature.pattern["throughput_kg_h"].get<double>(),
                35000.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["final_fineness_modulus"].get<double>(),
                3.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["water_l_kg"].get<double>(), 1.5, 1e-9);
    EXPECT_NEAR(out.signature.pattern["water_total_l_h"].get<double>(),
                35000.0 * 1.5, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillSandWash, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::sand_wash::apply(*stock, goodInput());

    auto cands = skill::sand_wash::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["throughput_kg_h"].get<double>(),
                20000.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::sand_wash::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "sand_wash cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: out-of-range FM is info-only, NOT blocking ─────────────
TEST(SkillSandWash, OutOfRangeFinenessModulusIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.final_fineness_modulus = 4.5;       // outside [2.0, 3.5]

    auto r = skill::sand_wash::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "out-of-range FM should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-SAND-FM-RANGE"));
    EXPECT_NO_THROW(skill::sand_wash::apply(*stock, in));
}
