// @lat: [[process/test-strategy#skill round-trip]]
//
// burnish skill — 5-case sweep covering identity-geometry synthesis,
// pressure / Ra DFM rejection, signature metadata, metadata recognition,
// and signature-stacking on repeated burnish passes.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/burnish.hpp"

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
TEST(SkillBurnish, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::burnish::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pressure_kn  = 2.0;
    in.target_ra_um = 0.4;

    auto out = skill::burnish::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("burnish"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects out-of-range pressure + Ra ──────────────────────
TEST(SkillBurnish, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::burnish::Input weak;
    weak.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    weak.pressure_kn  = 0.01;          // < 0.1 kN
    weak.target_ra_um = 0.4;
    auto r1 = skill::burnish::validate(*stock, weak);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-BURNISH-PRESSURE"));
    EXPECT_THROW(skill::burnish::apply(*stock, weak), skill::SkillError);

    skill::burnish::Input crushing;
    crushing.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    crushing.pressure_kn  = 50.0;       // > 20 kN
    crushing.target_ra_um = 0.4;
    auto r2 = skill::burnish::validate(*stock, crushing);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-BURNISH-PRESSURE"));

    skill::burnish::Input coarseRa;
    coarseRa.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    coarseRa.pressure_kn  = 2.0;
    coarseRa.target_ra_um = 6.3;        // > 3.2
    auto r3 = skill::burnish::validate(*stock, coarseRa);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-BURNISH-RA"));

    skill::burnish::Input zeroPress;
    zeroPress.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    zeroPress.pressure_kn  = 0.0;
    zeroPress.target_ra_um = 0.4;
    auto r4 = skill::burnish::validate(*stock, zeroPress);
    EXPECT_FALSE(r4.passed);
    EXPECT_THROW(skill::burnish::apply(*stock, zeroPress), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBurnish, SignatureRecordsKind)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 5.0);

    skill::burnish::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pressure_kn  = 3.5;
    in.target_ra_um = 0.2;

    auto out = skill::burnish::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("burnish"));
    EXPECT_NEAR(out.signature.pattern["pressure_kn"].get<double>(), 3.5, 1e-6);
    EXPECT_NEAR(out.signature.pattern["target_ra_um"].get<double>(), 0.2, 1e-6);
    EXPECT_TRUE(out.signature.pattern.contains("target_face_id"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("burnishing_roller"));
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillBurnish, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::burnish::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pressure_kn  = 2.0;
    in.target_ra_um = 0.4;
    auto out = skill::burnish::apply(*stock, in);

    auto cands = skill::burnish::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_NEAR(cands[0].recovered_params["target_ra_um"].get<double>(),
                0.4, 1e-6);

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::burnish::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "burnish cannot be recognized geometrically";
}

// ─── 5. Multiple burnish passes stack signatures ─────────────────────────
TEST(SkillBurnish, MultiplePassesStackSignatures)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::burnish::Input in;
    in.entry_face   = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.pressure_kn  = 1.5;
    in.target_ra_um = 0.8;
    auto w1 = skill::burnish::apply(*stock, in);

    in.pressure_kn  = 2.5;
    in.target_ra_um = 0.2;
    auto w2 = skill::burnish::apply(*w1.workpiece, in);
    EXPECT_EQ(w2.workpiece->features().size(), 2u);

    auto cands = skill::burnish::recognize(*w2.workpiece);
    EXPECT_EQ(cands.size(), 2u);
}
