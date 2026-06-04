// @lat: [[process/test-strategy#compound exhaust_manifold_port_compound]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/exhaust_manifold_port_compound.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

int findTopFaceId(const skill::Workpiece& wp) {
    int best = -1;
    double bestZ = -1e9;
    for (int i = 0; i < wp.faceCount(); ++i) {
        if (!wp.isFacePlanar(i)) continue;
        const auto n = wp.faceNormal(i);
        if (n.Z() < 0.9) continue;
        const auto c = wp.faceCenter(i);
        if (c.Z() > bestZ) { bestZ = c.Z(); best = i; }
    }
    return best;
}

skill::exhaust_manifold_port_compound::Input goodInput(const skill::Workpiece& wp)
{
    skill::exhaust_manifold_port_compound::Input in;
    in.face_id              = findTopFaceId(wp);
    in.center_x_mm          = 50.0;
    in.center_y_mm          = 50.0;
    in.port_inlet_rect_w_mm = 30.0;
    in.port_inlet_rect_h_mm = 25.0;
    in.port_outlet_dia_mm   = 32.0;
    in.transition_length_mm = 40.0;
    in.port_angle_deg       = 0.0;
    return in;
}

}  // namespace

// ─── 1. apply removes the port volume (rect inlet + transition + outlet) ─
TEST(SkillExhaustManifoldPort, ApplyCutsPort)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 60.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    const double v0 = volumeOf(stock->shape());
    auto out = skill::exhaust_manifold_port_compound::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());
    const double v1 = volumeOf(out.workpiece->shape());

    EXPECT_GT(v0 - v1, 0.0);
}

// ─── 2. validate rejects negative dimensions ─────────────────────────────
TEST(SkillExhaustManifoldPort, ValidateRejectsNegativeDims)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 60.0);
    auto in    = goodInput(*stock);
    in.port_outlet_dia_mm = -1.0;

    auto r = skill::exhaust_manifold_port_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);

    EXPECT_THROW(skill::exhaust_manifold_port_compound::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. validate rejects port_angle outside +-45 deg ─────────────────────
TEST(SkillExhaustManifoldPort, ValidateRejectsPortAngleOutOfRange)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 60.0);
    auto in    = goodInput(*stock);
    in.port_angle_deg = 60.0;

    auto r = skill::exhaust_manifold_port_compound::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PORT-ANGLE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. signature records engine_feature_type + 3 subfeatures ────────────
TEST(SkillExhaustManifoldPort, SignatureRecordsEngineFeature)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::exhaust_manifold_port_compound::apply(*stock, in);

    EXPECT_EQ(out.signature.pattern.at("kind").get<std::string>(),
              std::string("exhaust_manifold_port_compound"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("engine_feature_type")
              .get<std::string>(),
              std::string("exhaust_manifold_port"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
    EXPECT_NEAR(out.signature.pattern.at("port_outlet_dia_mm").get<double>(),
                32.0, 1e-6);
}

// ─── 5. recognize via metadata replay ────────────────────────────────────
TEST(SkillExhaustManifoldPort, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(100.0, 100.0, 60.0);
    auto in    = goodInput(*stock);
    auto out   = skill::exhaust_manifold_port_compound::apply(*stock, in);

    auto cands = skill::exhaust_manifold_port_compound::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
}
