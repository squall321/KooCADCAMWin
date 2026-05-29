// @lat: [[process/test-strategy#skill round-trip]]
//
// fabric_finishing — 5-case sweep covering identity geometry, DFM rejection
// of non-positive finishing_temp_c, signature recording of technique +
// finishing_temp_c, metadata-replay recognition, and high-temp info finding.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/fabric_finishing.hpp"

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

// ── 1. Apply: geometry is COMPLETELY unchanged ───────────────────────────
TEST(SkillFabricFinishing, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    skill::fabric_finishing::Input in;
    in.technique        = "sanforize";
    in.finishing_temp_c = 120.0;

    auto out = skill::fabric_finishing::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_EQ(out.signature.skill_id, std::string("fabric_finishing"));
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ── 2. DFM rejects non-positive finishing_temp_c ─────────────────────────
TEST(SkillFabricFinishing, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_finishing::Input zeroT;
    zeroT.technique = "sanforize"; zeroT.finishing_temp_c = 0.0;
    EXPECT_FALSE(skill::fabric_finishing::validate(*stock, zeroT).passed);
    EXPECT_THROW(skill::fabric_finishing::apply(*stock, zeroT), skill::SkillError);

    skill::fabric_finishing::Input negT;
    negT.technique = "calendering"; negT.finishing_temp_c = -10.0;
    EXPECT_FALSE(skill::fabric_finishing::validate(*stock, negT).passed);
    EXPECT_THROW(skill::fabric_finishing::apply(*stock, negT), skill::SkillError);
}

// ── 3. Signature records kind + technique + finishing_temp_c ─────────────
TEST(SkillFabricFinishing, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_finishing::Input in;
    in.technique        = "heatset";
    in.finishing_temp_c = 190.0;

    auto out = skill::fabric_finishing::apply(*stock, in);
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("fabric_finishing"));
    EXPECT_EQ(out.signature.pattern["technique"].get<std::string>(),
              std::string("heatset"));
    EXPECT_NEAR(out.signature.pattern["finishing_temp_c"].get<double>(),
                190.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometry_changed"].get<bool>());
    EXPECT_EQ(out.signature.tooling.tool_type, std::string("finishing_range"));
}

// ── 4. Recognize via metadata replay ─────────────────────────────────────
TEST(SkillFabricFinishing, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_finishing::Input in;
    in.technique        = "mercerize";
    in.finishing_temp_c = 95.0;

    auto out = skill::fabric_finishing::apply(*stock, in);
    auto cands = skill::fabric_finishing::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].recovered_params["technique"].get<std::string>(),
              std::string("mercerize"));
    EXPECT_NEAR(cands[0].recovered_params["finishing_temp_c"].get<double>(),
                95.0, 1e-9);

    skill::Workpiece raw(out.workpiece->shape());
    EXPECT_TRUE(skill::fabric_finishing::recognize(raw).empty());
}

// ── 5. High finishing_temp_c emits info finding (specific assertion) ─────
TEST(SkillFabricFinishing, HighTempEmitsInfoFinding)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 10.0);

    skill::fabric_finishing::Input hot;
    hot.technique        = "heatset";
    hot.finishing_temp_c = 250.0;       // > 230 °C threshold

    auto r = skill::fabric_finishing::validate(*stock, hot);
    EXPECT_TRUE(r.passed)
        << "high finishing_temp_c should NOT block (info-only)";
    bool info = false;
    for (const auto& f : r.findings) {
        if (f.code == "DFM-TEX-FINISH-TEMP" && f.severity == "info") {
            info = true; break;
        }
    }
    EXPECT_TRUE(info);

    // apply() should still succeed.
    EXPECT_NO_THROW(skill::fabric_finishing::apply(*stock, hot));
}
