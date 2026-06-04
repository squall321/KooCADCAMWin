// @lat: [[process/test-strategy#compound vent_burst_disc_pocket]]

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/vent_burst_disc_pocket.hpp"

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

skill::vent_burst_disc_pocket::Input goodInput(const skill::Workpiece& wp) {
    skill::vent_burst_disc_pocket::Input in;
    in.face_id         = findTopFaceId(wp);
    in.center_x_mm     = 20.0;
    in.center_y_mm     = 20.0;
    in.disc_dia_mm     = 10.0;
    in.pocket_depth_mm = 3.0;
    in.o_ring_size_key = "-111";
    in.vent_dia_mm     = 3.0;
    return in;
}
}  // namespace

// ─── 1. Apply cuts pocket + groove + through vent ────────────────────────
TEST(SkillVentBurstDisc, ApplyCutsPocketGrooveAndVent)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    auto in    = goodInput(*stock);
    ASSERT_GE(in.face_id, 0);

    auto out = skill::vent_burst_disc_pocket::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double v0 = volumeOf(stock->shape());
    const double v1 = volumeOf(out.workpiece->shape());
    EXPECT_LT(v1, v0);  // material removed
}

// ─── 2. Validate rejects unknown AS568 key ───────────────────────────────
TEST(SkillVentBurstDisc, ValidateRejectsUnknownAS568Key)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    auto in    = goodInput(*stock);
    in.o_ring_size_key = "-999";  // not in table

    auto r = skill::vent_burst_disc_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-AS568-KEY") { found = true; break; }
    EXPECT_TRUE(found);

    EXPECT_THROW(skill::vent_burst_disc_pocket::apply(*stock, in),
                 skill::SkillError);
}

// ─── 3. Validate rejects oversize vent (≥ disc/2) ────────────────────────
TEST(SkillVentBurstDisc, ValidateRejectsOversizeVent)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    auto in    = goodInput(*stock);
    in.vent_dia_mm = in.disc_dia_mm / 2.0 + 0.5;

    auto r = skill::vent_burst_disc_pocket::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool found = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-VENT-OVERSIZE") { found = true; break; }
    EXPECT_TRUE(found);
}

// ─── 4. Signature records compound + ev_feature_type + o_ring key ────────
TEST(SkillVentBurstDisc, SignatureRecordsCompoundAndORingKey)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    auto in    = goodInput(*stock);
    auto out   = skill::vent_burst_disc_pocket::apply(*stock, in);

    EXPECT_EQ(out.signature.skill_id,
              std::string("vent_burst_disc_pocket"));
    EXPECT_TRUE(out.signature.pattern.at("is_compound").get<bool>());
    EXPECT_EQ(out.signature.pattern.at("ev_feature_type").get<std::string>(),
              std::string("vent_burst_disc"));
    EXPECT_EQ(out.signature.pattern.at("o_ring_size_key").get<std::string>(),
              std::string("-111"));
    EXPECT_EQ(out.signature.pattern.at("subfeature_count").get<int>(), 3);
}

// ─── 5. Recognize via metadata replay ────────────────────────────────────
TEST(SkillVentBurstDisc, RecognizeMetadataReplay)
{
    auto stock = skill::createCuboidStock(40.0, 40.0, 8.0);
    auto in    = goodInput(*stock);
    auto out   = skill::vent_burst_disc_pocket::apply(*stock, in);

    auto cands = skill::vent_burst_disc_pocket::recognize(*out.workpiece);
    ASSERT_EQ(cands.size(), 1u);
    EXPECT_DOUBLE_EQ(cands[0].confidence, 1.0);
    EXPECT_EQ(cands[0].recovered_params.at("disc_dia_mm").get<double>(),
              in.disc_dia_mm);
}
