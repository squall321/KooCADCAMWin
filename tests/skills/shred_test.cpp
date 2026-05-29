// @lat: [[process/test-strategy#skill round-trip]]
//
// shred skill — 5-case sweep covering:
//   1. identity geometry on apply
//   2. DFM rejects bad input (zero/negative throughput or size)
//   3. signature carries skill kind + throughput_kg_h + final_particle_size_mm
//   4. recognize via metadata replay
//   5. specific: very fine particle size emits info finding and proceeds

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/shred.hpp"

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

// ─── 1. Apply: geometry is COMPLETELY unchanged ──────────────────────────
TEST(SkillShred, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 12.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::shred::Input in;
    in.throughput_kg_h        = 1000.0;
    in.final_particle_size_mm = 15.0;

    auto out = skill::shred::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("shred"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillShred, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shred::Input badThru;
    badThru.throughput_kg_h        = 0.0;       // invalid
    badThru.final_particle_size_mm = 10.0;
    auto r1 = skill::shred::validate(*stock, badThru);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));
    EXPECT_THROW(skill::shred::apply(*stock, badThru), skill::SkillError);

    skill::shred::Input badSize;
    badSize.throughput_kg_h        = 500.0;
    badSize.final_particle_size_mm = -1.0;      // invalid
    auto r2 = skill::shred::validate(*stock, badSize);
    EXPECT_FALSE(r2.passed);
    EXPECT_THROW(skill::shred::apply(*stock, badSize), skill::SkillError);
}

// ─── 3. Signature records skill kind + key params ────────────────────────
TEST(SkillShred, SignatureRecordsKindAndParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shred::Input in;
    in.throughput_kg_h        = 750.0;
    in.final_particle_size_mm = 12.5;

    auto out = skill::shred::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("shred"));
    EXPECT_NEAR(out.signature.pattern["throughput_kg_h"].get<double>(),
                750.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["final_particle_size_mm"].get<double>(),
                12.5, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("shredder"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillShred, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shred::Input in;
    in.throughput_kg_h        = 600.0;
    in.final_particle_size_mm = 20.0;

    auto out = skill::shred::apply(*stock, in);

    auto cands = skill::shred::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["final_particle_size_mm"].get<double>(),
                20.0, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto empty = skill::shred::recognize(raw);
    EXPECT_TRUE(empty.empty())
        << "shred cannot be recognized geometrically";
}

// ─── 5. Specific: very-fine particle size emits info finding, proceeds ───
TEST(SkillShred, FineParticleSizeInfoNotError)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::shred::Input in;
    in.throughput_kg_h        = 300.0;
    in.final_particle_size_mm = 0.5;     // < 1 mm → info hint

    auto r = skill::shred::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "fine grind is informational, not blocking";
    EXPECT_TRUE(hasFinding(r, "DFM-SHRED-FINE"));
    EXPECT_NO_THROW(skill::shred::apply(*stock, in));
}
