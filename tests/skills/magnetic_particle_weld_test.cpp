// @lat: [[process/test-strategy#skill round-trip]]
//
// magnetic_particle_weld — MT NDT skill, metadata-only sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/magnetic_particle_weld.hpp"

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

// ─── 1. Apply: geometry unchanged ────────────────────────────────────────
TEST(SkillMagneticParticleWeld, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();

    skill::magnetic_particle_weld::Input in;
    in.weld_id   = "W1";
    in.technique = "wet";
    in.accept    = true;

    auto out = skill::magnetic_particle_weld::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillMagneticParticleWeld, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::magnetic_particle_weld::Input in;
    in.weld_id   = "";           // empty
    in.technique = "fluorescent"; // not in {wet,dry}

    auto r = skill::magnetic_particle_weld::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::magnetic_particle_weld::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillMagneticParticleWeld, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::magnetic_particle_weld::Input in;
    in.weld_id   = "WELD-M9";
    in.technique = "dry";
    in.accept    = false;

    auto out = skill::magnetic_particle_weld::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("magnetic_particle_weld"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("magnetic_particle_weld"));
    EXPECT_EQ(out.signature.pattern["weld_id"].get<std::string>(),
              std::string("WELD-M9"));
    EXPECT_EQ(out.signature.pattern["technique"].get<std::string>(),
              std::string("dry"));
    EXPECT_FALSE(out.signature.pattern["accept"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillMagneticParticleWeld, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "carbon_steel_1045");

    skill::magnetic_particle_weld::Input in;
    in.weld_id   = "WELD-M1";
    in.technique = "wet";
    in.accept    = true;
    auto out = skill::magnetic_particle_weld::apply(*stock, in);

    auto cands = skill::magnetic_particle_weld::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["weld_id"].get<std::string>(),
              std::string("WELD-M1"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::magnetic_particle_weld::recognize(raw).empty());
}

// ─── 5. Skill-specific: non-ferromagnetic material → info finding ────────
TEST(SkillMagneticParticleWeld, NonFerromagneticMaterialEmitsInfo)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0, "aluminum_6061");

    skill::magnetic_particle_weld::Input in;
    in.weld_id   = "W1";
    in.technique = "wet";
    in.accept    = true;

    auto r = skill::magnetic_particle_weld::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "non-ferromagnetic material should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-NDT-MATERIAL"));
    EXPECT_NO_THROW(skill::magnetic_particle_weld::apply(*stock, in));
}
