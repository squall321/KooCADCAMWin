// @lat: [[engine/reverse-route#compound-grammar]]
//
// Verification for the B1.2 declarative compound grammar (orthodox un-cap).
//
// The flat foreign-CAD cap keeps domain-compound recognizers below the 0.7
// inference threshold because their loose geometric fallbacks fire on anything;
// a generic "wraps an atom" un-cap was REFUTED (fpscan_test).  The orthodox
// path recognizes a compound only when a SPECIFIC declared pattern of measured
// precise atoms is present.  This test proves the first pattern — bolt-circle —
// end to end:
//   1. RECALL: it fires on a real ring of equal, parallel-axis drilled holes,
//      recovering the right hole count + bolt-circle diameter + centre.
//   2. SPECIFICITY: it stays silent on arrangements that are NOT bolt circles
//      (too few holes, mixed diameters, collinear holes) and on the foreign
//      fpscan panel (no part there contains a bolt circle).

#include <gtest/gtest.h>

#include "re/CompoundGrammar.hpp"
#include "re/Recognizer.hpp"
#include "io/StepIO.hpp"
#include "skills/Skill.hpp"
#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"

#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace koocadcam;
using json = nlohmann::json;

namespace {

// Hand-build a drill_hole candidate the way drill_hole::recognize() emits one.
// faceId is unique per physical hole; wrapRad defaults to a full revolution so a
// synthetic hole passes the grammar's revolution-completeness gate (a real hole,
// not a partial corner fillet).
skill::RecognizedFeature drillAt(double x, double y, double dia, int faceId,
                                 double confidence = 0.95,
                                 double wrapRad = 2.0 * M_PI)
{
    skill::RecognizedFeature rf;
    rf.skill_id = "drill_hole";
    rf.recovered_params = {
        { "position_x_mm", x }, { "position_y_mm", y }, { "position_z_mm", 20.0 },
        { "axis_dir", { 0.0, 0.0, -1.0 } },
        { "diameter_mm", dia }, { "through_hole", true },
    };
    rf.confidence = confidence;
    rf.matched_geometry = {
        { "cylindrical_face_id", faceId }, { "cyl_wrap_rad", wrapRad },
    };
    return rf;
}

// Four holes evenly spaced on a circle of radius R centred at (cx, cy).
std::vector<skill::RecognizedFeature> ringOfHoles(int n, double R, double dia,
                                                  double cx = 30.0, double cy = 30.0)
{
    std::vector<skill::RecognizedFeature> v;
    for (int i = 0; i < n; ++i) {
        const double t = 2.0 * M_PI * i / n;
        v.push_back(drillAt(cx + R * std::cos(t), cy + R * std::sin(t), dia, i));
    }
    return v;
}

const skill::RecognizedFeature* findCompound(
    const std::vector<skill::RecognizedFeature>& v, const std::string& id)
{
    for (const auto& c : v)
        if (c.skill_id == id) return &c;
    return nullptr;
}

}  // namespace

// ── 1. Pure-function recall: a 4-hole ring is recognised as a bolt circle ──
TEST(CompoundGrammar, RecognisesFourHoleBoltCircle)
{
    const auto holes = ringOfHoles(4, /*R=*/20.0, /*dia=*/5.0);
    const auto out = re::recognizeCompounds(holes);

    const auto* bc = findCompound(out, "bolt_circle_pattern");
    ASSERT_NE(bc, nullptr) << "grammar failed to recognise a clean 4-hole ring";
    EXPECT_EQ(bc->recovered_params.value("hole_count", 0), 4);
    EXPECT_NEAR(bc->recovered_params.value("hole_dia_mm", 0.0), 5.0, 1e-6);
    EXPECT_NEAR(bc->recovered_params.value("bolt_circle_dia_mm", 0.0), 40.0, 1e-3);
    EXPECT_NEAR(bc->recovered_params.value("center_x_mm", 0.0), 30.0, 1e-3);
    EXPECT_NEAR(bc->recovered_params.value("center_y_mm", 0.0), 30.0, 1e-3);
    EXPECT_GE(bc->confidence, 0.9);
    EXPECT_TRUE(bc->matched_geometry.value("is_compound", false));
}

// A six-hole ring recovers hole_count = 6.
TEST(CompoundGrammar, RecognisesSixHoleBoltCircle)
{
    const auto out = re::recognizeCompounds(ringOfHoles(6, 25.0, 4.0));
    const auto* bc = findCompound(out, "bolt_circle_pattern");
    ASSERT_NE(bc, nullptr);
    EXPECT_EQ(bc->recovered_params.value("hole_count", 0), 6);
    EXPECT_NEAR(bc->recovered_params.value("bolt_circle_dia_mm", 0.0), 50.0, 1e-3);
}

// A row of equally-spaced equal holes is recognised as a linear array.
TEST(CompoundGrammar, RecognisesLinearHoleArray)
{
    // 5 holes at x = 10,20,30,40,50 on y = 30 — collinear, pitch 10.
    std::vector<skill::RecognizedFeature> row;
    for (int i = 0; i < 5; ++i) row.push_back(drillAt(10.0 + 10.0 * i, 30.0, 4.0, i));
    const auto out = re::recognizeCompounds(row);

    const auto* la = findCompound(out, "linear_hole_array");
    ASSERT_NE(la, nullptr) << "grammar failed to recognise an evenly-pitched row";
    EXPECT_EQ(la->recovered_params.value("hole_count", 0), 5);
    EXPECT_NEAR(la->recovered_params.value("pitch_mm", 0.0), 10.0, 1e-3);
    EXPECT_NEAR(la->recovered_params.value("hole_dia_mm", 0.0), 4.0, 1e-6);
    EXPECT_NEAR(la->recovered_params.value("span_mm", 0.0), 40.0, 1e-3);
    // A clean line is NOT also reported as a bolt circle.
    EXPECT_EQ(findCompound(out, "bolt_circle_pattern"), nullptr);
}

// A diagonal row is still a linear array (direction-independent).
TEST(CompoundGrammar, RecognisesDiagonalLinearArray)
{
    std::vector<skill::RecognizedFeature> row;
    for (int i = 0; i < 4; ++i) row.push_back(drillAt(10.0 + 7.0 * i, 10.0 + 7.0 * i, 5.0, i));
    const auto out = re::recognizeCompounds(row);
    const auto* la = findCompound(out, "linear_hole_array");
    ASSERT_NE(la, nullptr);
    EXPECT_EQ(la->recovered_params.value("hole_count", 0), 4);
    EXPECT_NEAR(la->recovered_params.value("pitch_mm", 0.0), std::hypot(7.0, 7.0), 1e-3);
}

// Collinear but UNEVENLY spaced holes are not an array.
TEST(CompoundGrammar, IgnoresUnevenLinearSpacing)
{
    std::vector<skill::RecognizedFeature> row = {
        drillAt(10, 30, 4, 0), drillAt(20, 30, 4, 1), drillAt(45, 30, 4, 2),
    };
    EXPECT_EQ(findCompound(re::recognizeCompounds(row), "linear_hole_array"), nullptr);
}

// A bolt circle must NOT be misread as a linear array (its centres aren't collinear).
TEST(CompoundGrammar, BoltCircleIsNotLinearArray)
{
    const auto out = re::recognizeCompounds(ringOfHoles(6, 20.0, 4.0));
    EXPECT_NE(findCompound(out, "bolt_circle_pattern"), nullptr);
    EXPECT_EQ(findCompound(out, "linear_hole_array"), nullptr);
}

// ── 2. Specificity: arrangements that are NOT bolt circles stay silent ─────
TEST(CompoundGrammar, IgnoresTwoHoles)
{
    std::vector<skill::RecognizedFeature> two = { drillAt(10, 30, 5, 0), drillAt(50, 30, 5, 1) };
    EXPECT_EQ(findCompound(re::recognizeCompounds(two), "bolt_circle_pattern"), nullptr);
}

TEST(CompoundGrammar, IgnoresMixedDiameters)
{
    // Three holes on a circle but each a different diameter — not a bolt set.
    std::vector<skill::RecognizedFeature> mix = {
        drillAt(50, 30, 5, 0), drillAt(20, 47, 8, 1), drillAt(20, 13, 11, 2),
    };
    EXPECT_EQ(findCompound(re::recognizeCompounds(mix), "bolt_circle_pattern"), nullptr);
}

TEST(CompoundGrammar, IgnoresCollinearHoles)
{
    // Three equal holes in a straight line — a circle fit is degenerate.
    std::vector<skill::RecognizedFeature> line = {
        drillAt(10, 30, 5, 0), drillAt(30, 30, 5, 1), drillAt(50, 30, 5, 2),
    };
    EXPECT_EQ(findCompound(re::recognizeCompounds(line), "bolt_circle_pattern"), nullptr);
}

// A ring of PARTIAL cylinders (e.g. a rectangular pocket's four concyclic
// corner fillets) must NOT read as a bolt circle: each spans only ~90°, far
// short of a full hole.  This is the exact false positive the fpscan panel hit.
TEST(CompoundGrammar, IgnoresConcyclicCornerFillets)
{
    const double quarter = 0.5 * M_PI;   // 90° corner-fillet arc
    std::vector<skill::RecognizedFeature> corners;
    // Four corners of a 40x24 rectangle centred at (30,30) — concyclic.
    const double hx = 20.0, hy = 12.0;
    corners.push_back(drillAt(30 - hx, 30 - hy, 2, 0, 0.95, quarter));
    corners.push_back(drillAt(30 + hx, 30 - hy, 2, 1, 0.95, quarter));
    corners.push_back(drillAt(30 + hx, 30 + hy, 2, 2, 0.95, quarter));
    corners.push_back(drillAt(30 - hx, 30 + hy, 2, 3, 0.95, quarter));
    EXPECT_EQ(findCompound(re::recognizeCompounds(corners), "bolt_circle_pattern"), nullptr);
}

TEST(CompoundGrammar, IgnoresLowConfidenceAtoms)
{
    // A ring of holes that the recognizer is unsure about (< 0.7) is not
    // trustworthy grounding — no compound is emitted.
    auto weak = ringOfHoles(4, 20.0, 5.0);
    for (auto& h : weak) h.confidence = 0.5;
    EXPECT_EQ(findCompound(re::recognizeCompounds(weak), "bolt_circle_pattern"), nullptr);
}

// ── 3. End-to-end: a drilled bolt circle survives a STEP round-trip ────────
TEST(CompoundGrammar, EndToEndBoltCircleThroughAnalyze)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    auto wp = stock;
    const double cx = 30.0, cy = 30.0, R = 18.0;
    for (int i = 0; i < 4; ++i) {
        const double t = 2.0 * M_PI * i / 4;
        skill::drill_hole::Input in;
        in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
        in.position_x_mm = cx + R * std::cos(t);
        in.position_y_mm = cy + R * std::sin(t);
        in.axis_dir = gp_Dir(0, 0, -1);
        in.diameter_mm = 5.0;
        in.through_hole = true;
        wp = skill::drill_hole::apply(*wp, in).workpiece;
    }

    // STEP round-trip strips all metadata → foreign-equivalent geometry.
    const fs::path p = fs::temp_directory_path() /
                       ("koo_grammar_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(wp->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    const auto* bc = findCompound(cands, "bolt_circle_pattern");
    ASSERT_NE(bc, nullptr)
        << "grammar did not recover the bolt circle after a STEP round-trip";
    EXPECT_EQ(bc->recovered_params.value("hole_count", 0), 4);
    EXPECT_NEAR(bc->recovered_params.value("bolt_circle_dia_mm", 0.0), 2 * R, 0.5);
    // Grounded compound is exempt from the cap — it keeps its high confidence.
    EXPECT_GE(bc->confidence, 0.7);
}

// ── 4. Safety: a single drilled hole produces no bolt circle ───────────────
TEST(CompoundGrammar, SingleHoleNoBoltCircle)
{
    namespace fs = std::filesystem;
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::drill_hole::Input in;
    in.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    in.position_x_mm = 30; in.position_y_mm = 30;
    in.axis_dir = gp_Dir(0, 0, -1); in.diameter_mm = 8; in.through_hole = true;
    auto wp = skill::drill_hole::apply(*stock, in).workpiece;

    const fs::path p = fs::temp_directory_path() /
                       ("koo_grammar1_" + std::to_string(::rand()) + ".step");
    std::string err;
    io::StepIO::write(wp->shape(), p, err);
    auto reim = io::StepIO::read(p, err);
    std::error_code ec; fs::remove(p, ec);
    skill::Workpiece foreign(*reim);

    const auto cands = re::analyze(foreign, /*applyCap=*/true);
    EXPECT_EQ(findCompound(cands, "bolt_circle_pattern"), nullptr)
        << "bolt-circle grammar false-fired on a single hole";
}
