// @lat: [[process/test-strategy#skill round-trip]]
//
// bearing_replace skill — 5-case maintenance sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/bearing_replace.hpp"

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

skill::bearing_replace::Input goodInput()
{
    skill::bearing_replace::Input in;
    in.bearing_id        = "6205-2RS";
    in.location          = "MOTOR-DE";
    in.install_torque_nm = 50.0;
    return in;
}

}  // namespace

// ─── 1. Apply: geometry CLONED unchanged ─────────────────────────────────
TEST(SkillBearingReplace, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::bearing_replace::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.signature.skill_id, std::string("bearing_replace"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate REJECTS bad input ───────────────────────────────────────
TEST(SkillBearingReplace, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bearing_id = "";                    // empty
    auto r = skill::bearing_replace::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_TRUE(hasFinding(r, "DFM-INPUT"));
    EXPECT_THROW(skill::bearing_replace::apply(*stock, in), skill::SkillError);

    auto in2 = goodInput();
    in2.install_torque_nm = 0.0;           // non-positive
    auto r2 = skill::bearing_replace::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillBearingReplace, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.bearing_id        = "NU313-E";
    in.location          = "GEARBOX-INPUT";
    in.install_torque_nm = 120.0;

    auto out = skill::bearing_replace::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("bearing_replace"));
    EXPECT_EQ(out.signature.pattern["bearing_id"].get<std::string>(),
              std::string("NU313-E"));
    EXPECT_EQ(out.signature.pattern["location"].get<std::string>(),
              std::string("GEARBOX-INPUT"));
    EXPECT_NEAR(out.signature.pattern["install_torque_nm"].get<double>(),
                120.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillBearingReplace, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    auto out = skill::bearing_replace::apply(*stock, goodInput());

    auto cands = skill::bearing_replace::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["bearing_id"].get<std::string>(),
              std::string("6205-2RS"));

    // Strip metadata → recognize empty.
    skill::Workpiece raw(out.workpiece->shape());
    auto cands2 = skill::bearing_replace::recognize(raw);
    EXPECT_TRUE(cands2.empty())
        << "bearing_replace cannot be recognized geometrically";
}

// ─── 5. SPECIFIC: out-of-band torque → INFO finding, not block ───────────
TEST(SkillBearingReplace, OutOfBandTorqueIsInfoNotError)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    auto in = goodInput();
    in.install_torque_nm = 800.0;          // > 500 Nm upper guidance

    auto r = skill::bearing_replace::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "out-of-band torque should NOT block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-BEARING-TORQUE"));
    EXPECT_NO_THROW(skill::bearing_replace::apply(*stock, in));
}
