// @lat: [[process/test-strategy#skill round-trip]]
//
// freight_loading — truck/container loading metadata skill.  5-case sweep:
//   1. apply clones geometry unchanged
//   2. validate rejects bad input
//   3. signature records kind + freight_class + weight_kg + count + container_type
//   4. recognize metadata replay
//   5. specific: utilization_pct + over/underload findings per container

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/freight_loading.hpp"

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

skill::freight_loading::Input goodInput()
{
    skill::freight_loading::Input in;
    in.freight_class  = "85";
    in.weight_kg      = 15000.0;       // mid-load 40ft container
    in.count          = 20;
    in.container_type = "40ft";
    return in;
}

}  // namespace

// ─── 1. Apply clones geometry unchanged ──────────────────────────────────
TEST(SkillFreightLoading, ApplyClonesGeometryUnchanged)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    const double volBefore  = volumeOf(stock->shape());
    const int    faceBefore = stock->faceCount();
    const int    edgeBefore = stock->edgeCount();

    auto out = skill::freight_loading::apply(*stock, goodInput());
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    EXPECT_NEAR(volumeOf(out.workpiece->shape()), volBefore, 1e-9);
    EXPECT_EQ(out.workpiece->faceCount(), faceBefore);
    EXPECT_EQ(out.workpiece->edgeCount(), edgeBefore);
    EXPECT_EQ(out.workpiece->features().size(), 1u);
    EXPECT_NEAR(out.signature.tooling.stock_removed_mm3, 0.0, 1e-9);
}

// ─── 2. Validate rejects bad input ───────────────────────────────────────
TEST(SkillFreightLoading, ValidateRejectsBadInput)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Empty freight_class → error.
    auto in1 = goodInput();
    in1.freight_class = "";
    auto r1 = skill::freight_loading::validate(*stock, in1);
    EXPECT_FALSE(r1.passed);
    EXPECT_TRUE(hasFinding(r1, "DFM-INPUT"));
    EXPECT_THROW(skill::freight_loading::apply(*stock, in1),
                 skill::SkillError);

    // Non-positive weight → error.
    auto in2 = goodInput();
    in2.weight_kg = 0.0;
    auto r2 = skill::freight_loading::validate(*stock, in2);
    EXPECT_FALSE(r2.passed);
    EXPECT_TRUE(hasFinding(r2, "DFM-INPUT"));

    // Non-positive count → error.
    auto in3 = goodInput();
    in3.count = 0;
    auto r3 = skill::freight_loading::validate(*stock, in3);
    EXPECT_FALSE(r3.passed);
    EXPECT_TRUE(hasFinding(r3, "DFM-INPUT"));

    // Unknown container_type → error.
    auto in4 = goodInput();
    in4.container_type = "rocket_payload_bay";
    auto r4 = skill::freight_loading::validate(*stock, in4);
    EXPECT_FALSE(r4.passed);
    EXPECT_TRUE(hasFinding(r4, "DFM-FL-CONTAINER"));
}

// ─── 3. Signature records kind + key params ──────────────────────────────
TEST(SkillFreightLoading, SignatureRecordsKindAndKeyParams)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    skill::freight_loading::Input in;
    in.freight_class  = "175";
    in.weight_kg      = 5000.0;
    in.count          = 8;
    in.container_type = "20ft";

    auto out = skill::freight_loading::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id, std::string("freight_loading"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("freight_loading"));
    EXPECT_EQ(out.signature.pattern["freight_class"].get<std::string>(),
              std::string("175"));
    EXPECT_NEAR(out.signature.pattern["weight_kg"].get<double>(),
                5000.0, 1e-9);
    EXPECT_EQ(out.signature.pattern["count"].get<int>(), 8);
    EXPECT_EQ(out.signature.pattern["container_type"].get<std::string>(),
              std::string("20ft"));
    EXPECT_NEAR(out.signature.pattern["avg_unit_weight_kg"].get<double>(),
                625.0, 1e-9);
    EXPECT_FALSE(out.signature.pattern["geometric_change"].get<bool>());
}

// ─── 4. Recognize metadata replay ────────────────────────────────────────
TEST(SkillFreightLoading, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);
    auto out = skill::freight_loading::apply(*stock, goodInput());

    auto cands = skill::freight_loading::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_NEAR(cands[0].confidence, 1.0, 1e-6);
    EXPECT_EQ(cands[0].skill_id, std::string("freight_loading"));
    EXPECT_EQ(cands[0].recovered_params["container_type"].get<std::string>(),
              std::string("40ft"));
    EXPECT_EQ(cands[0].recovered_params["count"].get<int>(), 20);

    // Stripped workpiece → recognize empty.
    skill::Workpiece raw(out.workpiece->shape(), out.workpiece->material());
    EXPECT_TRUE(skill::freight_loading::recognize(raw).empty());
}

// ─── 5. Specific: over/underload + utilization_pct ───────────────────────
TEST(SkillFreightLoading, OverloadUnderloadDetected)
{
    auto stock = skill::createCuboidStock(50.0, 50.0, 10.0);

    // Overload: 40000 kg on 40ft container (~28800 max).
    auto over = goodInput();
    over.weight_kg = 40000.0;
    auto rO = skill::freight_loading::validate(*stock, over);
    EXPECT_FALSE(rO.passed);
    EXPECT_TRUE(hasFinding(rO, "DFM-FL-OVERLOAD"));
    EXPECT_THROW(skill::freight_loading::apply(*stock, over),
                 skill::SkillError);

    // Underload: 1000 kg on a 40ft container (<30% util) → info only.
    auto under = goodInput();
    under.weight_kg = 1000.0;
    auto rU = skill::freight_loading::validate(*stock, under);
    EXPECT_TRUE(rU.passed);
    EXPECT_TRUE(hasFinding(rU, "DFM-FL-UNDERLOAD"));
    auto outU = skill::freight_loading::apply(*stock, under);
    // utilization_pct = 100 * 1000 / 28800 ≈ 3.47%
    EXPECT_LT(outU.signature.pattern["utilization_pct"].get<double>(), 30.0);
    EXPECT_GT(outU.signature.pattern["utilization_pct"].get<double>(), 0.0);

    // Healthy mid-load: ~52% util, no findings.
    auto good = goodInput();   // 15000 kg on 40ft
    auto rG = skill::freight_loading::validate(*stock, good);
    EXPECT_TRUE(rG.passed);
    EXPECT_FALSE(hasFinding(rG, "DFM-FL-UNDERLOAD"));
    EXPECT_FALSE(hasFinding(rG, "DFM-FL-OVERLOAD"));
    auto outG = skill::freight_loading::apply(*stock, good);
    EXPECT_GT(outG.signature.pattern["utilization_pct"].get<double>(), 30.0);
    EXPECT_LT(outG.signature.pattern["utilization_pct"].get<double>(), 100.0);
}
