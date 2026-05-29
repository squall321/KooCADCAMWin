// @lat: [[process/test-strategy#skill round-trip]]
//
// wood_finishing skill — metadata-only surface finish (varnish/stain/oil/...).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/wood_finishing.hpp"

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

// ─── 1. Apply handles input: geometry unchanged, signature added ─────────
TEST(SkillWoodFinishing, ApplyHandlesInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0, "walnut");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::wood_finishing::Input in;
    in.finish_type         = "varnish";
    in.coats               = 3;
    in.cure_hours_per_coat = 8.0;
    in.surface_sheen       = "satin";

    auto out = skill::wood_finishing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("wood_finishing"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input (unknown finish, zero coats, ...) ─────
TEST(SkillWoodFinishing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0);

    // Unknown finish_type → DFM-FINISH-TYPE.
    skill::wood_finishing::Input bad;
    bad.finish_type         = "epoxy_jewel_resin";   // unsupported
    bad.coats               = 2;
    bad.cure_hours_per_coat = 8.0;
    bad.surface_sheen       = "satin";
    auto r1 = skill::wood_finishing::validate(*stock, bad);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-FINISH-TYPE"));
    EXPECT_THROW(skill::wood_finishing::apply(*stock, bad), skill::SkillError);

    // Zero coats → DFM-FINISH-COATS-MIN.
    skill::wood_finishing::Input zero;
    zero.finish_type         = "oil";
    zero.coats               = 0;
    zero.cure_hours_per_coat = 4.0;
    zero.surface_sheen       = "matte";
    auto r2 = skill::wood_finishing::validate(*stock, zero);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-FINISH-COATS-MIN"));

    // Zero cure → DFM-FINISH-CURE.
    skill::wood_finishing::Input noCure;
    noCure.finish_type         = "lacquer";
    noCure.coats               = 2;
    noCure.cure_hours_per_coat = 0.0;
    noCure.surface_sheen       = "gloss";
    auto r3 = skill::wood_finishing::validate(*stock, noCure);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-FINISH-CURE"));
}

// ─── 3. Signature records kind + finish_type + sheen + water_resistance ──
TEST(SkillWoodFinishing, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0);

    skill::wood_finishing::Input in;
    in.finish_type         = "lacquer";
    in.coats               = 4;
    in.cure_hours_per_coat = 2.0;
    in.surface_sheen       = "gloss";

    auto out = skill::wood_finishing::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(), std::string("wood_finishing"));
    EXPECT_EQ(out.signature.pattern["finish_type"].get<std::string>(), std::string("lacquer"));
    EXPECT_EQ(out.signature.pattern["coats"].get<int>(), 4);
    EXPECT_NEAR(out.signature.pattern["cure_hours"].get<double>(), 8.0, 1e-6);
    EXPECT_EQ(out.signature.pattern["surface_sheen"].get<std::string>(), std::string("gloss"));
    EXPECT_EQ(out.signature.pattern["water_resistance"].get<std::string>(), std::string("high"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("finishing_booth"));
}

// ─── 4. Recognize via metadata replay (no geometric fallback) ────────────
TEST(SkillWoodFinishing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0);

    skill::wood_finishing::Input in;
    in.finish_type         = "oil";
    in.coats               = 2;
    in.cure_hours_per_coat = 12.0;
    in.surface_sheen       = "matte";
    auto out = skill::wood_finishing::apply(*stock, in);

    auto cands = skill::wood_finishing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["finish_type"].get<std::string>(), std::string("oil"));
    EXPECT_EQ(cands[0].recovered_params["coats"].get<int>(), 2);

    // Stripped workpiece → no candidate.
    skill::Workpiece bare(out.workpiece->shape(), out.workpiece->material());
    auto cands2 = skill::wood_finishing::recognize(bare);
    EXPECT_TRUE(cands2.empty())
        << "wood_finishing cannot be recognised without metadata";
}

// ─── 5. Excess coats emit warning but apply succeeds ──────────────────────
TEST(SkillWoodFinishing, ExcessCoatsWarnsButProceeds)
{
    auto stock = skill::createCuboidStock(200.0, 100.0, 20.0);

    skill::wood_finishing::Input many;
    many.finish_type         = "varnish";
    many.coats               = 10;    // > 6 triggers warning
    many.cure_hours_per_coat = 8.0;
    many.surface_sheen       = "satin";

    auto r = skill::wood_finishing::validate(*stock, many);
    EXPECT_TRUE(r.passed) << "excess coats is a warning, not error";
    EXPECT_TRUE(hasFinding(r, "DFM-FINISH-COATS-MAX"));
    EXPECT_NO_THROW(skill::wood_finishing::apply(*stock, many));
}
