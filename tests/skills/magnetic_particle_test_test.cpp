// @lat: [[process/test-strategy#skill round-trip]]
//
// magnetic_particle_test — MT inspection skill (metadata-only).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/magnetic_particle_test.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <gp_Dir.hxx>

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

// ── 1. Apply on ferromagnetic stock clones geometry unchanged ────────────
TEST(SkillMagneticParticleTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");
    const double volBefore = volumeOf(stock->shape());

    skill::magnetic_particle_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique         = "wet";
    in.severity_class    = "none";
    in.surface_prep      = "ground";

    auto out = skill::magnetic_particle_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. Aluminum blocked with DFM-MT-MATERIAL error ───────────────────────
TEST(SkillMagneticParticleTest, AluminumBlockedAsNonFerromagnetic)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "aluminum_6061");

    skill::magnetic_particle_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique         = "wet";
    in.severity_class    = "none";
    in.surface_prep      = "as_machined";

    auto r = skill::magnetic_particle_test::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-MT-MATERIAL"));
    EXPECT_THROW(skill::magnetic_particle_test::apply(*stock, in),
                 skill::SkillError);
}

// ── 3. ValidateRejectsBadInput: bad technique / severity / surface_prep ──
TEST(SkillMagneticParticleTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");

    skill::magnetic_particle_test::Input badTech;
    badTech.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badTech.technique         = "vapor";   // not allowed
    badTech.severity_class    = "none";
    badTech.surface_prep      = "as_machined";
    auto r1 = skill::magnetic_particle_test::validate(*stock, badTech);
    EXPECT_FALSE(r1.passed);

    skill::magnetic_particle_test::Input badSev = badTech;
    badSev.technique      = "wet";
    badSev.severity_class = "severe";   // not allowed
    auto r2 = skill::magnetic_particle_test::validate(*stock, badSev);
    EXPECT_FALSE(r2.passed);

    skill::magnetic_particle_test::Input badPrep = badTech;
    badPrep.technique      = "wet";
    badPrep.severity_class = "none";
    badPrep.surface_prep   = "anodized";   // not allowed
    auto r3 = skill::magnetic_particle_test::validate(*stock, badPrep);
    EXPECT_FALSE(r3.passed);
}

// ── 4. Signature records MT parameters + tooling ─────────────────────────
TEST(SkillMagneticParticleTest, SignatureRecordsParametersAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "alloy_steel_4140");

    skill::magnetic_particle_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique         = "dry";
    in.severity_class    = "rounded";
    in.surface_prep      = "polished";

    auto out = skill::magnetic_particle_test::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("magnetic_particle_test"));
    EXPECT_EQ(out.signature.pattern["technique"].get<std::string>(),
              std::string("dry"));
    EXPECT_EQ(out.signature.pattern["severity_class"].get<std::string>(),
              std::string("rounded"));
    EXPECT_EQ(out.signature.pattern["surface_prep"].get<std::string>(),
              std::string("polished"));
    EXPECT_TRUE(out.signature.pattern["defect_detected"].get<bool>());
    EXPECT_EQ(out.signature.pattern["ndt_category"].get<std::string>(),
              std::string("MT"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("magnetic_yoke"));
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 5. Recognize metadata replay round-trip ──────────────────────────────
TEST(SkillMagneticParticleTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");

    skill::magnetic_particle_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique         = "wet";
    in.severity_class    = "linear";
    in.surface_prep      = "ground";

    auto out = skill::magnetic_particle_test::apply(*stock, in);

    auto cands = skill::magnetic_particle_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].recovered_params["technique"].get<std::string>(),
              std::string("wet"));
    EXPECT_EQ(cands[0].recovered_params["severity_class"].get<std::string>(),
              std::string("linear"));

    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::magnetic_particle_test::recognize(raw).empty());
}
