// @lat: [[process/test-strategy#skill round-trip]]
//
// primer_install skill — small-arms primer seating no-op sweep.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/primer_install.hpp"

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
TEST(SkillPrimerInstall, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::primer_install::Input in;
    in.primer_size  = "small_pistol";
    in.primer_brand = "CCI";

    auto out = skill::primer_install::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. ValidateRejectsBadInput: empty brand → error ─────────────────────
TEST(SkillPrimerInstall, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::primer_install::Input in;
    in.primer_size  = "small_pistol";
    in.primer_brand = "";            // traceability required

    auto r = skill::primer_install::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    EXPECT_THROW(skill::primer_install::apply(*stock, in), skill::SkillError);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillPrimerInstall, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::primer_install::Input in;
    in.primer_size  = "large_rifle";
    in.primer_brand = "Federal";

    auto out = skill::primer_install::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("primer_install"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("primer_install"));
    EXPECT_EQ(out.signature.pattern["primer_size"].get<std::string>(),
              std::string("large_rifle"));
    EXPECT_EQ(out.signature.pattern["primer_brand"].get<std::string>(),
              std::string("Federal"));
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
}

// ─── 4. Recognize via metadata replay ────────────────────────────────────
TEST(SkillPrimerInstall, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::primer_install::Input in;
    in.primer_size  = "small_rifle";
    in.primer_brand = "Winchester";

    auto out  = skill::primer_install::apply(*stock, in);
    auto cands = skill::primer_install::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["primer_brand"].get<std::string>(),
              std::string("Winchester"));

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::primer_install::recognize(raw).empty());
}

// ─── 5. Skill-specific: unknown primer_size emits info but proceeds ──────
TEST(SkillPrimerInstall, UnknownPrimerSizeIsInfoNotError)
{
    auto stock = skill::createCuboidStock(20.0, 20.0, 30.0, "brass_c260");

    skill::primer_install::Input in;
    in.primer_size  = "tiny_oddball";   // unrecognised
    in.primer_brand = "Generic";

    auto r = skill::primer_install::validate(*stock, in);
    EXPECT_TRUE(r.passed)
        << "unknown primer_size should not block (info-only)";
    EXPECT_TRUE(hasFinding(r, "DFM-AMMO-PRIMER"));
    EXPECT_NO_THROW(skill::primer_install::apply(*stock, in));
}
