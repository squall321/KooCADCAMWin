// @lat: [[process/test-strategy#skill round-trip]]
//
// sinter_press — classical press-and-sinter PM (geometric no-op).
// Cases (5):
//   1. ApplyClonesGeometryUnchanged
//   2. ValidateRejectsBadInput
//   3. SignatureRecordsKindAndKeyParams
//   4. RecognizeMetadataReplay
//   5. Unknown material → info-only finding, validate still passes

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/sinter_press.hpp"

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

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillSinterPress, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 8.0);
    const double volBefore   = volumeOf(stock->shape());
    const int    facesBefore = stock->faceCount();
    const int    edgesBefore = stock->edgeCount();

    skill::sinter_press::Input in;
    in.material                = "iron_fc_0205";
    in.compaction_pressure_mpa = 600.0;
    in.sinter_temp_c           = 1120.0;
    in.sinter_time_min         = 45.0;

    auto out = skill::sinter_press::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, volBefore * 1e-6);
    EXPECT_EQ(out.workpiece->faceCount(), facesBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgesBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillSinterPress, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 8.0);

    skill::sinter_press::Input zeroP;
    zeroP.material                = "iron_fc_0205";
    zeroP.compaction_pressure_mpa = 0.0;     // invalid
    zeroP.sinter_temp_c           = 1120.0;
    zeroP.sinter_time_min         = 45.0;
    auto r1 = skill::sinter_press::validate(*stock, zeroP);
    EXPECT_FALSE(r1.passed);
    EXPECT_THROW(skill::sinter_press::apply(*stock, zeroP),
                 skill::SkillError);

    skill::sinter_press::Input zeroT;
    zeroT.material                = "iron_fc_0205";
    zeroT.compaction_pressure_mpa = 600.0;
    zeroT.sinter_temp_c           = 0.0;     // invalid
    zeroT.sinter_time_min         = 45.0;
    auto r2 = skill::sinter_press::validate(*stock, zeroT);
    EXPECT_FALSE(r2.passed);

    skill::sinter_press::Input zeroTime;
    zeroTime.material                = "iron_fc_0205";
    zeroTime.compaction_pressure_mpa = 600.0;
    zeroTime.sinter_temp_c           = 1120.0;
    zeroTime.sinter_time_min         = 0.0;  // invalid
    auto r3 = skill::sinter_press::validate(*stock, zeroTime);
    EXPECT_FALSE(r3.passed);
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillSinterPress, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 8.0);

    skill::sinter_press::Input in;
    in.material                = "bronze_cf_1000";
    in.compaction_pressure_mpa = 400.0;
    in.sinter_temp_c           = 820.0;
    in.sinter_time_min         = 30.0;

    auto out = skill::sinter_press::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("sinter_press"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("sinter_press"));
    EXPECT_EQ(out.signature.pattern["process_family"].get<std::string>(),
              std::string("powder_metallurgy"));
    EXPECT_EQ(out.signature.pattern["material"].get<std::string>(),
              std::string("bronze_cf_1000"));
    EXPECT_NEAR(out.signature.pattern["compaction_pressure_mpa"].get<double>(),
                400.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["sinter_temp_c"].get<double>(),
                820.0, 1e-9);
    EXPECT_NEAR(out.signature.pattern["sinter_time_min"].get<double>(),
                30.0, 1e-9);
    EXPECT_EQ(out.signature.tooling.tool_type,
              std::string("uniaxial_press_furnace"));
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillSinterPress, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 8.0);

    skill::sinter_press::Input in;
    in.material                = "stainless_410";
    in.compaction_pressure_mpa = 700.0;
    in.sinter_temp_c           = 1250.0;
    in.sinter_time_min         = 60.0;

    auto out   = skill::sinter_press::apply(*stock, in);
    auto cands = skill::sinter_press::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_EQ(cands[0].skill_id, std::string("sinter_press"));
    EXPECT_GT(cands[0].confidence, 0.99);
    EXPECT_EQ(cands[0].recovered_params["material"].get<std::string>(),
              std::string("stainless_410"));
    EXPECT_NEAR(cands[0].recovered_params["sinter_temp_c"].get<double>(),
                1250.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::sinter_press::recognize(raw).empty());
}

// ─── 5. Unknown material → info-only, validate still passes ──────────────
TEST(SkillSinterPress, UnknownMaterialIsInfoNotError)
{
    auto stock = skill::createCuboidStock(25.0, 25.0, 8.0);

    skill::sinter_press::Input in;
    in.material                = "exotic_pm_blend_x";   // not in catalog
    in.compaction_pressure_mpa = 600.0;
    in.sinter_temp_c           = 1120.0;
    in.sinter_time_min         = 45.0;

    auto r = skill::sinter_press::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool foundInfo = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-SP-MAT" && f.severity == "info") {
            foundInfo = true;
            break;
        }
    }
    EXPECT_TRUE(foundInfo);

    EXPECT_NO_THROW(skill::sinter_press::apply(*stock, in));
}
