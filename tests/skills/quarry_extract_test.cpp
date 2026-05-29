// @lat: [[process/test-strategy#skill round-trip]]
//
// quarry_extract skill — 5-case sweep covering identity-geometry synthesis,
// DFM rejection on non-positive volume, metadata-only recognition,
// and dimension-stone fit warning for drill_blast.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/quarry_extract.hpp"

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

skill::quarry_extract::Input goodInput()
{
    skill::quarry_extract::Input in;
    in.rock_type         = "granite";
    in.volume_m3         = 8.0;
    in.extraction_method = "diamond_wire";
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ──────────────────────────────────
TEST(SkillQuarryExtract, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 1000.0, "granite");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::quarry_extract::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("quarry_extract"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input (volume_m3 = 0) ───────────────────────
TEST(SkillQuarryExtract, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 1000.0, "granite");

    auto in = goodInput();
    in.volume_m3 = 0.0;

    auto r = skill::quarry_extract::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::quarry_extract::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + rock/volume/method ──────────────────────
TEST(SkillQuarryExtract, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 1000.0, "marble");

    auto in = goodInput();
    in.rock_type         = "marble";
    in.volume_m3         = 12.5;
    in.extraction_method = "channeling";

    auto out = skill::quarry_extract::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("quarry_extract"));
    EXPECT_EQ(out.signature.pattern["rock_type"].get<std::string>(),
              std::string("marble"));
    EXPECT_NEAR(out.signature.pattern["volume_m3"].get<double>(), 12.5, 1e-9);
    EXPECT_EQ(out.signature.pattern["extraction_method"].get<std::string>(),
              std::string("channeling"));
    EXPECT_TRUE(out.signature.pattern["is_dimension_stone"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillQuarryExtract, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 1000.0, "granite");
    auto out = skill::quarry_extract::apply(*stock, goodInput());

    auto cands = skill::quarry_extract::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["rock_type"].get<std::string>(),
              std::string("granite"));
    EXPECT_NEAR(cands[0].recovered_params["volume_m3"].get<double>(),
                8.0, 1e-9);

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::quarry_extract::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "quarry_extract cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: drill_blast on marble triggers method-fit warning ───────
TEST(SkillQuarryExtract, DrillBlastOnMarbleEmitsMethodFitWarning)
{
    auto stock = skill::createCuboidStock(2000.0, 1500.0, 1000.0, "marble");

    auto in = goodInput();
    in.rock_type         = "marble";
    in.extraction_method = "drill_blast";

    auto r = skill::quarry_extract::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "drill_blast on marble should warn but not block";
    EXPECT_TRUE(hasFinding(r, "DFM-QUARRY-METHOD-FIT"));
    EXPECT_NO_THROW(skill::quarry_extract::apply(*stock, in));
}
