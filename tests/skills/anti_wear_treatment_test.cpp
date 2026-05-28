// @lat: [[process/test-strategy#skill round-trip]]
//
// anti_wear_treatment skill — 5-case sweep covering identity-geometry
// synthesis, depth / hardness DFM rejection, signature metadata, metadata
// recognition, and technique-driven tool_type selection.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/anti_wear_treatment.hpp"

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
TEST(SkillAntiWearTreatment, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::anti_wear_treatment::Input in;
    in.entry_face             = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique              = "laser_hardening";
    in.depth_um               = 100.0;
    in.hardness_increase_pct  = 50.0;

    auto out = skill::anti_wear_treatment::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("anti_wear_treatment"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects out-of-range depth + hardness ───────────────────
TEST(SkillAntiWearTreatment, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anti_wear_treatment::Input tooShallow;
    tooShallow.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooShallow.technique             = "laser_hardening";
    tooShallow.depth_um              = 0.5;       // < 1.0
    tooShallow.hardness_increase_pct = 50.0;
    auto r1 = skill::anti_wear_treatment::validate(*stock, tooShallow);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-ANTIWEAR-DEPTH"));
    EXPECT_THROW(skill::anti_wear_treatment::apply(*stock, tooShallow),
                 skill::SkillError);

    skill::anti_wear_treatment::Input tooDeep;
    tooDeep.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    tooDeep.technique             = "laser_hardening";
    tooDeep.depth_um              = 5000.0;       // > 2000
    tooDeep.hardness_increase_pct = 50.0;
    auto r2 = skill::anti_wear_treatment::validate(*stock, tooDeep);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-ANTIWEAR-DEPTH"));

    skill::anti_wear_treatment::Input crazyHardness;
    crazyHardness.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    crazyHardness.technique             = "laser_hardening";
    crazyHardness.depth_um              = 100.0;
    crazyHardness.hardness_increase_pct = 1000.0; // > 500
    auto r3 = skill::anti_wear_treatment::validate(*stock, crazyHardness);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-ANTIWEAR-HARDNESS"));

    skill::anti_wear_treatment::Input zeroDepth;
    zeroDepth.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    zeroDepth.technique             = "laser_hardening";
    zeroDepth.depth_um              = 0.0;
    zeroDepth.hardness_increase_pct = 50.0;
    auto r4 = skill::anti_wear_treatment::validate(*stock, zeroDepth);
    EXPECT_FALSE(r4.passed);
    EXPECT_THROW(skill::anti_wear_treatment::apply(*stock, zeroDepth),
                 skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillAntiWearTreatment, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::anti_wear_treatment::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique             = "ion_implantation";
    in.depth_um              = 2.5;
    in.hardness_increase_pct = 80.0;

    auto out = skill::anti_wear_treatment::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("anti_wear_treatment"));
    EXPECT_EQ(out.signature.pattern["technique"].get<std::string>(),
              std::string("ion_implantation"));
    EXPECT_NEAR(out.signature.pattern["depth_um"].get<double>(), 2.5, 1e-6);
    EXPECT_NEAR(
        out.signature.pattern["hardness_increase_pct"].get<double>(),
        80.0, 1e-6);
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillAntiWearTreatment, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anti_wear_treatment::Input in;
    in.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.technique             = "surface_texturing";
    in.depth_um              = 10.0;
    in.hardness_increase_pct = 20.0;
    auto out = skill::anti_wear_treatment::apply(*stock, in);

    auto cands = skill::anti_wear_treatment::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["technique"].get<std::string>(),
              std::string("surface_texturing"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::anti_wear_treatment::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "anti_wear_treatment cannot be recognized geometrically";
}

// ─── 5. tool_type tracks the supplied technique ──────────────────────────
TEST(SkillAntiWearTreatment, ToolTypeReflectsTechnique)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::anti_wear_treatment::Input laser;
    laser.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    laser.technique             = "laser_hardening";
    laser.depth_um              = 100.0;
    laser.hardness_increase_pct = 50.0;
    auto outLaser = skill::anti_wear_treatment::apply(*stock, laser);
    EXPECT_EQ(outLaser.signature.tooling.tool_type,
              std::string("laser_hardening_head"));

    skill::anti_wear_treatment::Input induct;
    induct.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    induct.technique             = "induction_hardening";
    induct.depth_um              = 500.0;
    induct.hardness_increase_pct = 80.0;
    auto outInduct = skill::anti_wear_treatment::apply(*stock, induct);
    EXPECT_EQ(outInduct.signature.tooling.tool_type,
              std::string("induction_coil"));

    skill::anti_wear_treatment::Input ion;
    ion.entry_face            = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    ion.technique             = "ion_implantation";
    ion.depth_um              = 2.0;
    ion.hardness_increase_pct = 30.0;
    auto outIon = skill::anti_wear_treatment::apply(*stock, ion);
    EXPECT_EQ(outIon.signature.tooling.tool_type,
              std::string("ion_implantation_gun"));
}
