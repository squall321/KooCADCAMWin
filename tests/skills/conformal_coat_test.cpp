// @lat: [[process/test-strategy#skill round-trip]]
//
// conformal_coat — protective conformal coating skill, 5-case sweep.
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput (thickness out of IPC range)
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. IpcClassDerivedFromThickness  (specific assertion)

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/conformal_coat.hpp"

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
TEST(SkillConformalCoat, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 1.6, "FR4");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::conformal_coat::Input in;
    in.coat_type    = "acrylic";
    in.thickness_um = 50.0;
    in.coverage_pct = 95.0;

    auto out = skill::conformal_coat::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);

    EXPECT_EQ(out.signature.skill_id, std::string("conformal_coat"));
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. DFM: thickness outside [25, 300] μm or coverage > 100 → error ────
TEST(SkillConformalCoat, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 1.6, "FR4");

    skill::conformal_coat::Input tooThin;
    tooThin.coat_type    = "acrylic";
    tooThin.thickness_um = 10.0;             // < 25 μm
    tooThin.coverage_pct = 100.0;
    auto r1 = skill::conformal_coat::validate(*stock, tooThin);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-CC-THK-LOW"));
    EXPECT_THROW(skill::conformal_coat::apply(*stock, tooThin), skill::SkillError);

    skill::conformal_coat::Input tooThick = tooThin;
    tooThick.thickness_um = 500.0;           // > 300 μm
    auto r2 = skill::conformal_coat::validate(*stock, tooThick);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-CC-THK-HIGH"));

    skill::conformal_coat::Input badCov;
    badCov.coat_type    = "silicone";
    badCov.thickness_um = 50.0;
    badCov.coverage_pct = 150.0;             // > 100 → error
    auto r3 = skill::conformal_coat::validate(*stock, badCov);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-CC-COVERAGE"));
}

// ─── 3. Signature carries kind + key parameters ──────────────────────────
TEST(SkillConformalCoat, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 1.6, "FR4");

    skill::conformal_coat::Input in;
    in.coat_type    = "parylene";
    in.thickness_um = 30.0;
    in.coverage_pct = 100.0;

    auto out = skill::conformal_coat::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("conformal_coat"));
    EXPECT_EQ(out.signature.pattern["coat_type"].get<std::string>(),
              std::string("parylene"));
    EXPECT_NEAR(out.signature.pattern["thickness_um"].get<double>(), 30.0, 1e-6);
    EXPECT_NEAR(out.signature.pattern["coverage_pct"].get<double>(), 100.0, 1e-6);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("conformal_coater"));
}

// ─── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillConformalCoat, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 1.6, "FR4");

    skill::conformal_coat::Input in;
    in.coat_type    = "silicone";
    in.thickness_um = 80.0;
    in.coverage_pct = 98.0;

    auto out = skill::conformal_coat::apply(*stock, in);

    auto cands = skill::conformal_coat::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["coat_type"].get<std::string>(),
              std::string("silicone"));

    skill::Workpiece raw(out.workpiece->shape());
    auto candsEmpty = skill::conformal_coat::recognize(raw);
    EXPECT_TRUE(candsEmpty.empty())
        << "conformal_coat is not geometrically recognizable";
}

// ─── 5. IPC class derives from thickness (specific assertion) ────────────
TEST(SkillConformalCoat, IpcClassDerivedFromThickness)
{
    auto stock = skill::createCuboidStock(100.0, 60.0, 1.6, "FR4");

    // Standard band 25 ≤ thk ≤ 130 μm → "IPC-CC-830"
    skill::conformal_coat::Input mid;
    mid.coat_type    = "acrylic";
    mid.thickness_um = 75.0;
    mid.coverage_pct = 100.0;
    auto outMid = skill::conformal_coat::apply(*stock, mid);
    EXPECT_EQ(outMid.signature.pattern["ipc_class"].get<std::string>(),
              std::string("IPC-CC-830"));

    // Thick band thk > 130 μm → "IPC-CC-830_thick"
    skill::conformal_coat::Input thick;
    thick.coat_type    = "polyurethane";
    thick.thickness_um = 200.0;
    thick.coverage_pct = 100.0;
    auto outThick = skill::conformal_coat::apply(*stock, thick);
    EXPECT_EQ(outThick.signature.pattern["ipc_class"].get<std::string>(),
              std::string("IPC-CC-830_thick"));
}
