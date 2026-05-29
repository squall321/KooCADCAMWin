// @lat: [[process/test-strategy#skill round-trip]]
//
// encapsulate — REAL geometric two-part capsule shell.
// Cases (5):
//   1. ApplyBuildsHollowCapsule
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsCapsuleDims
//   4. RecognizeGeometricHeuristic
//   5. OverfillEmitsInfoForKnownSize

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/encapsulate.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

// ─── 1. Apply builds a hollow capsule (cap + body fused, cavity cut) ───────
//
// Size "0": overall 21.70 mm, body OD 7.65 mm, cap OD 7.77 mm.
// Outer shell volume ≈ cap + body cylinders − overlap.
// Cavity volume ≈ π·(bodyR − 0.10)²·(L − 0.20) for wall = 0.10 mm.
// Shell volume = outer − cavity, must be > 0 and significantly less than
// outer (thin-walled).  Face count grows past the input cuboid's 6.
TEST(SkillEncapsulate, ApplyBuildsHollowCapsule)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 30.0);
    const int faceBefore = stock->faceCount();

    skill::encapsulate::Input in;
    in.capsule_size   = "0";
    in.fill_weight_mg = 400.0;
    in.fill_type      = "powder";

    auto out = skill::encapsulate::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    // Geometry changed — face count is no longer the 6-face cuboid.
    EXPECT_NE(out.workpiece->faceCount(), faceBefore);
    EXPECT_GE(out.workpiece->faceCount(), 3);

    // Shell volume = outer − cavity.  For size "0":
    //   bodyR = 3.825  bodyLen = 13.02   capR = 3.885  capLen+overlap ≈ 9.88
    //   outer ≈ π·3.825²·13.02 + π·3.885²·(8.68+1.5) − π·3.825²·1.5
    //          ≈ 598.5 + 482.7 − 68.9 ≈ 1012 mm³
    //   cavity ≈ π·(3.825−0.10)²·(21.70−0.20)
    //          ≈ π·3.725²·21.50 ≈ 937 mm³
    //   shell ≈ 75 mm³
    const double vShell = volumeOf(out.workpiece->shape());
    EXPECT_GT(vShell, 30.0);
    EXPECT_LT(vShell, 250.0);

    // stock_removed_mm3 is the additive convention: negative & magnitude
    // matches the shell-volume metric in the signature.
    EXPECT_LT(out.signature.tooling.stock_removed_mm3, 0.0);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3,
                -out.signature.pattern["shell_volume_mm3"].get<double>(), 1e-3);

    EXPECT_EQ(out.workpiece->features().size(), 1u);
}

// ─── 2. Validate rejects bad input ─────────────────────────────────────────
TEST(SkillEncapsulate, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0);

    skill::encapsulate::Input zeroW;
    zeroW.capsule_size   = "0";
    zeroW.fill_weight_mg = 0.0;
    zeroW.fill_type      = "powder";
    {
        auto r = skill::encapsulate::validate(*stock, zeroW);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    }
    EXPECT_THROW(skill::encapsulate::apply(*stock, zeroW), skill::SkillError);

    skill::encapsulate::Input badSize;
    badSize.capsule_size   = "Z9";
    badSize.fill_weight_mg = 400.0;
    badSize.fill_type      = "powder";
    {
        auto r = skill::encapsulate::validate(*stock, badSize);
        EXPECT_FALSE(r.passed);
        EXPECT_TRUE(hasFinding(r, "DFM-ENC-SIZE"));
    }
    EXPECT_THROW(skill::encapsulate::apply(*stock, badSize),
                 skill::SkillError);
}

// ─── 3. Signature records catalog dims + computed shell/cavity volumes ─────
TEST(SkillEncapsulate, SignatureRecordsCapsuleDims)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 30.0);

    skill::encapsulate::Input in;
    in.capsule_size   = "00";
    in.fill_weight_mg = 750.0;
    in.fill_type      = "pellet";

    auto out = skill::encapsulate::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("encapsulate"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("encapsulate"));
    EXPECT_EQ(out.signature.pattern["capsule_size"].get<std::string>(),
              std::string("00"));
    EXPECT_TRUE(out.signature.pattern["geometric_change"].get<bool>());

    // Catalog dims for size "00".
    EXPECT_NEAR(out.signature.pattern["overall_len_mm"].get<double>(),
                23.30, 1e-3);
    EXPECT_NEAR(out.signature.pattern["body_od_mm"].get<double>(),
                8.53, 1e-3);

    // shell_volume_mm3 = outer − cavity must be > 0 and matches OCCT volume.
    EXPECT_GT(out.signature.pattern["shell_volume_mm3"].get<double>(), 0.0);
    EXPECT_GT(out.signature.pattern["cavity_volume_mm3"].get<double>(), 0.0);
    EXPECT_NEAR(volumeOf(out.workpiece->shape()),
                out.signature.pattern["shell_volume_mm3"].get<double>(),
                10.0);    // tolerance for overlap approx
}

// ─── 4. Recognize geometric heuristic on a raw shell ───────────────────────
TEST(SkillEncapsulate, RecognizeGeometricHeuristic)
{
    auto stock = skill::createCuboidStock(15.0, 15.0, 30.0);

    skill::encapsulate::Input in;
    in.capsule_size   = "1";
    in.fill_weight_mg = 300.0;
    in.fill_type      = "granule";

    auto out   = skill::encapsulate::apply(*stock, in);

    // Metadata replay path.
    auto cands = skill::encapsulate::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["capsule_size"].get<std::string>(),
              std::string("1"));

    // Stripped of metadata → geometric heuristic.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::encapsulate::recognize(raw);
    ASSERT_EQ(cands2.size(), 1u);
    EXPECT_GE(cands2[0].confidence, 0.4);
    EXPECT_LE(cands2[0].confidence, 0.7);
    EXPECT_EQ(cands2[0].recovered_params["capsule_size"].get<std::string>(),
              std::string("1"));
}

// ─── 5. Overfill on known size emits info, still proceeds ──────────────────
TEST(SkillEncapsulate, OverfillEmitsInfoForKnownSize)
{
    auto stock = skill::createCuboidStock(10.0, 10.0, 20.0);

    // Size "4" capacity 150 mg; overfill at 250 mg → info.
    skill::encapsulate::Input in;
    in.capsule_size   = "4";
    in.fill_weight_mg = 250.0;
    in.fill_type      = "powder";

    auto r = skill::encapsulate::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-ENC-OVERFILL"));

    auto out = skill::encapsulate::apply(*stock, in);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("capsule_filler"));
    EXPECT_GT(out.signature.tooling.tool_dia_mm, 0.0);
}
