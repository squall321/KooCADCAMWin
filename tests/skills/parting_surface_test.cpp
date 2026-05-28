// @lat: [[process/test-strategy#skill round-trip]]
//
// parting_surface — large thin annular planar flange around the workpiece
// silhouette at a given Z plane.
//
// Cases (6):
//   1. apply fuses a flange and ADDS volume.
//   2. DFM rejects silhouette_z outside workpiece Z bbox.
//   3. DFM warns when extension < 2 mm.
//   4. recognize replays history.
//   5. Topology recognize finds a large planar flange without history.
//   6. Flange XY bbox extends past workpiece by `extension_mm`.

#include <gtest/gtest.h>

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/parting_surface.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>

using namespace koocadcam;

namespace {

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps p; BRepGProp::VolumeProperties(s, p); return p.Mass();
}

}  // namespace

// ─── 1. Apply adds flange volume ──────────────────────────────────────────
TEST(SkillPartingSurface, ApplyAddsFlangeVolume)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);
    const double stockVol = volumeOf(stock->shape());

    skill::parting_surface::Input in;
    in.silhouette_plane_z_mm = 5.0;     // mid-height
    in.extension_mm          = 5.0;

    auto out = skill::parting_surface::apply(*stock, in);
    ASSERT_FALSE(out.workpiece->shape().IsNull());

    const double added = volumeOf(out.workpiece->shape()) - stockVol;
    // Flange area = (outer - inner) = (50 × 40) - (40 × 30) = 2000 - 1200 = 800
    // Thickness = 0.5.  Added = 400 mm³.
    const double approxAdded = (50.0 * 40.0 - 40.0 * 30.0) * 0.5;
    EXPECT_GT(added, 0.0);
    EXPECT_NEAR(added, approxAdded, approxAdded * 0.10);

    EXPECT_EQ(out.signature.skill_id, std::string("parting_surface"));
    EXPECT_EQ(out.signature.pattern["kind"].get<std::string>(),
              std::string("parting_surface"));
}

// ─── 2. DFM rejects silhouette_z outside Z bbox ───────────────────────────
TEST(SkillPartingSurface, ValidateRejectsZOutOfRange)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::parting_surface::Input in;
    in.silhouette_plane_z_mm = 50.0;    // > 10
    in.extension_mm          = 5.0;

    auto r = skill::parting_surface::validate(*stock, in);
    EXPECT_FALSE(r.passed);
    bool sawCode = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PART-Z") { sawCode = true; break; }
    EXPECT_TRUE(sawCode);

    EXPECT_THROW(skill::parting_surface::apply(*stock, in), skill::SkillError);
}

// ─── 3. DFM warns on small extension ──────────────────────────────────────
TEST(SkillPartingSurface, ValidateWarnsOnSmallExtension)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::parting_surface::Input in;
    in.silhouette_plane_z_mm = 5.0;
    in.extension_mm          = 1.0;    // < 2

    auto r = skill::parting_surface::validate(*stock, in);
    EXPECT_TRUE(r.passed);
    bool sawWarn = false;
    for (const auto& f : r.findings)
        if (f.code == "DFM-PART-EXT" && f.severity == "warning") { sawWarn = true; break; }
    EXPECT_TRUE(sawWarn);
}

// ─── 4. Recognize replays history ─────────────────────────────────────────
TEST(SkillPartingSurface, RecognizeReplaysHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 12.0);

    skill::parting_surface::Input in;
    in.silhouette_plane_z_mm = 6.5;
    in.extension_mm          = 4.0;

    auto out = skill::parting_surface::apply(*stock, in);
    auto cands = skill::parting_surface::recognize(*out.workpiece);
    ASSERT_GE(cands.size(), 1u);

    const auto& c = cands.front();
    EXPECT_GT(c.confidence, 0.7);
    EXPECT_NEAR(c.recovered_params["silhouette_plane_z_mm"].get<double>(),
                6.5, 0.05);
    EXPECT_NEAR(c.recovered_params["extension_mm"].get<double>(), 4.0, 0.05);
}

// ─── 5. Topology recognize without history ────────────────────────────────
TEST(SkillPartingSurface, RecognizeWithoutHistory)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 12.0);

    skill::parting_surface::Input in;
    in.silhouette_plane_z_mm = 6.0;
    in.extension_mm          = 5.0;

    auto out = skill::parting_surface::apply(*stock, in);
    auto wpNoHist = std::make_shared<skill::Workpiece>(
        out.workpiece->shape(), out.workpiece->material());
    auto cands = skill::parting_surface::recognize(*wpNoHist);
    ASSERT_GE(cands.size(), 1u);
    // Topology-based confidence is lower.
    EXPECT_GT(cands.front().confidence, 0.3);
}

// ─── 6. Flange extends bbox by `extension_mm` ─────────────────────────────
TEST(SkillPartingSurface, FlangeExtendsBoundingBox)
{
    auto stock = skill::createCuboidStock(40.0, 30.0, 10.0);

    skill::parting_surface::Input in;
    in.silhouette_plane_z_mm = 5.0;
    in.extension_mm          = 5.0;

    Bnd_Box bb0;
    BRepBndLib::Add(stock->shape(), bb0);
    double x0Min, y0Min, z0Min, x0Max, y0Max, z0Max;
    bb0.Get(x0Min, y0Min, z0Min, x0Max, y0Max, z0Max);

    auto out = skill::parting_surface::apply(*stock, in);
    Bnd_Box bb1;
    BRepBndLib::Add(out.workpiece->shape(), bb1);
    double x1Min, y1Min, z1Min, x1Max, y1Max, z1Max;
    bb1.Get(x1Min, y1Min, z1Min, x1Max, y1Max, z1Max);

    EXPECT_NEAR(x1Min, x0Min - 5.0, 0.5);
    EXPECT_NEAR(x1Max, x0Max + 5.0, 0.5);
    EXPECT_NEAR(y1Min, y0Min - 5.0, 0.5);
    EXPECT_NEAR(y1Max, y0Max + 5.0, 0.5);
}
