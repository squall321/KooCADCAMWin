// @lat: [[process/test-strategy#skill round-trip]]
//
// ultrasonic_test — UT inspection skill (metadata-only, no geometric change).

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/ultrasonic_test.hpp"

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

// ── 1. Apply clones geometry unchanged ───────────────────────────────────
TEST(SkillUltrasonicTest, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::ultrasonic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.freq_mhz          = 5.0;
    in.probe_dia_mm      = 10.0;
    in.area_mm2          = 100.0;
    in.defect_class      = "none";

    auto out = skill::ultrasonic_test::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_TRUE(out.signature.pattern["is_inspection_only"].get<bool>());
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. ValidateRejectsBadInput: bad freq / probe / area / defect_class ───
TEST(SkillUltrasonicTest, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_test::Input badFreq;
    badFreq.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    badFreq.freq_mhz          = 0.0;
    badFreq.probe_dia_mm      = 10.0;
    badFreq.area_mm2          = 100.0;
    badFreq.defect_class      = "none";
    auto r1 = skill::ultrasonic_test::validate(*stock, badFreq);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::ultrasonic_test::apply(*stock, badFreq), skill::SkillError);

    skill::ultrasonic_test::Input badArea = badFreq;
    badArea.freq_mhz = 5.0;
    badArea.area_mm2 = -1.0;
    auto r2 = skill::ultrasonic_test::validate(*stock, badArea);
    EXPECT_FALSE(r2.passed);

    skill::ultrasonic_test::Input badClass = badFreq;
    badClass.freq_mhz     = 5.0;
    badClass.defect_class = "catastrophic";   // not in allowed set
    auto r3 = skill::ultrasonic_test::validate(*stock, badClass);
    EXPECT_FALSE(r3.passed);
}

// ── 3. Signature records UT parameters + tooling ─────────────────────────
TEST(SkillUltrasonicTest, SignatureRecordsParametersAndTooling)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.freq_mhz          = 2.25;
    in.probe_dia_mm      = 12.7;
    in.area_mm2          = 500.0;
    in.defect_class      = "minor";

    auto out = skill::ultrasonic_test::apply(*stock, in);
    EXPECT_EQ(out.signature.skill_id, std::string("ultrasonic_test"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("ultrasonic_test"));
    EXPECT_NEAR(out.signature.pattern["freq_mhz"].get<double>(), 2.25, 1e-9);
    EXPECT_NEAR(out.signature.pattern["probe_dia_mm"].get<double>(), 12.7, 1e-9);
    EXPECT_NEAR(out.signature.pattern["area_mm2"].get<double>(), 500.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["defect_class"].get<std::string>(),
              std::string("minor"));
    EXPECT_TRUE(out.signature.pattern["defect_detected"].get<bool>());
    EXPECT_EQ(out.signature.pattern["ndt_category"].get<std::string>(),
              std::string("UT"));
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("ut_transducer"));
    EXPECT_NEAR(out.signature.tooling.tool_dia_mm, 12.7, 1e-9);
    EXPECT_GT(out.signature.tooling.est_cycle_time_s, 0.0);
}

// ── 4. Recognize metadata replay ─────────────────────────────────────────
TEST(SkillUltrasonicTest, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "carbon_steel_1045");

    skill::ultrasonic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.freq_mhz          = 5.0;
    in.probe_dia_mm      = 10.0;
    in.area_mm2          = 250.0;
    in.defect_class      = "none";

    auto out = skill::ultrasonic_test::apply(*stock, in);
    auto cands = skill::ultrasonic_test::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-9);
    EXPECT_EQ(cands[0].skill_id, std::string("ultrasonic_test"));
    EXPECT_NEAR(cands[0].recovered_params["freq_mhz"].get<double>(), 5.0, 1e-9);

    // Stripped workpiece → no candidates (no geometric fingerprint).
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::ultrasonic_test::recognize(raw).empty());
}

// ── 5. Cast material → info finding but proceeds (UT attenuation) ────────
TEST(SkillUltrasonicTest, CastMaterialEmitsInfoButProceeds)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0, "cast_iron_grey");

    skill::ultrasonic_test::Input in;
    in.target_face_datum = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.freq_mhz          = 5.0;
    in.probe_dia_mm      = 10.0;
    in.area_mm2          = 100.0;
    in.defect_class      = "none";

    auto r = skill::ultrasonic_test::validate(*stock, in);
    EXPECT_TRUE(r.passed) << "high-attenuation material should not BLOCK";
    EXPECT_TRUE(hasFinding(r, "DFM-UT-MATERIAL"));
    EXPECT_NO_THROW(skill::ultrasonic_test::apply(*stock, in));
}
