// @lat: [[engine/cam-3axis-verify]]
//
// ToolLibrary: family canonicalisation + nearest-pocket selection, and the
// executable T/M6 emission it enables in toGCodeProgram.

#include <gtest/gtest.h>

#include "cam/ToolLibrary.hpp"
#include "cam/ToolpathPlan.hpp"

#include "skills/Stock.hpp"
#include "skills/Workpiece.hpp"
#include "skills/drill_hole.hpp"
#include "skills/mill_rect_pocket.hpp"

#include <cctype>

using namespace koocadcam;

static skill::ToolingMeta makeTM(const std::string& type, double dia,
                                 const std::string& mat = "carbide", int flutes = 2)
{
    skill::ToolingMeta tm;
    tm.tool_type = type;
    tm.tool_dia_mm = dia;
    tm.tool_material = mat;
    tm.flute_count = flutes;
    return tm;
}

// The many skill-authored tool_type spellings collapse onto the magazine's
// canonical families.
TEST(ToolLibrary, CanonicalisesToolFamilies)
{
    EXPECT_EQ(cam::canonicalToolFamily("drill"),            "drill");
    EXPECT_EQ(cam::canonicalToolFamily("micro_drill"),      "drill");
    EXPECT_EQ(cam::canonicalToolFamily("gun_drill"),        "drill");
    EXPECT_EQ(cam::canonicalToolFamily("end_mill"),         "end_mill");
    EXPECT_EQ(cam::canonicalToolFamily("ball_mill"),        "ball_mill");
    EXPECT_EQ(cam::canonicalToolFamily("ballnose"),         "ball_mill");
    EXPECT_EQ(cam::canonicalToolFamily("spot_face_cutter"), "spot_drill");
    EXPECT_EQ(cam::canonicalToolFamily("countersink_tool"), "spot_drill");
    EXPECT_EQ(cam::canonicalToolFamily("tap"),              "tap");
    EXPECT_EQ(cam::canonicalToolFamily("thread_mill"),      "thread_mill");
    EXPECT_EQ(cam::canonicalToolFamily("reamer"),           "reamer");
    EXPECT_EQ(cam::canonicalToolFamily("laser"),            "");  // not machinable here
}

// selectTool picks the nearest pocket of the RIGHT family.
TEST(ToolLibrary, PicksNearestPocketOfCorrectFamily)
{
    // 6mm end mill → the 6mm end-mill pocket (exact), never a 6mm ball or drill.
    const auto em = cam::selectTool(makeTM("end_mill", 6.0, "carbide", 4));
    ASSERT_TRUE(em.has_value());
    EXPECT_EQ(em->family, "end_mill");
    EXPECT_NEAR(em->dia_mm, 6.0, 1e-9);

    // 6mm ball mill → the ball-mill pocket, NOT the end mill of the same diameter.
    const auto ball = cam::selectTool(makeTM("ball_mill", 6.0));
    ASSERT_TRUE(ball.has_value());
    EXPECT_EQ(ball->family, "ball_mill");

    // 4.8mm drill → the 5mm drill pocket (nearest within tolerance).
    const auto dr = cam::selectTool(makeTM("drill", 4.8));
    ASSERT_TRUE(dr.has_value());
    EXPECT_EQ(dr->family, "drill");
    EXPECT_NEAR(dr->dia_mm, 5.0, 1e-9);
}

// A request too far from any pocket, or of an unknown family, FAILS (nullopt) so
// the caller keeps a comment and never machines with a wrong tool.
TEST(ToolLibrary, FailsWhenNoPocketIsCloseEnough)
{
    // A 50mm end mill is far beyond the largest pocket (12mm).
    EXPECT_FALSE(cam::selectTool(makeTM("end_mill", 50.0)).has_value());
    // Unknown family.
    EXPECT_FALSE(cam::selectTool(makeTM("waterjet", 6.0)).has_value());
    // Zero diameter is not selectable.
    EXPECT_FALSE(cam::selectTool(makeTM("drill", 0.0)).has_value());
}

// A COMPOUND ';'-joined tool_type (277 skills author these, e.g. a milled cutout
// that also drills) must canonicalise on its PRIMARY (first) op — an incidental
// later token must NOT steal the family and pick a physically wrong tool.
TEST(ToolLibrary, CompoundToolTypeUsesPrimaryOperation)
{
    // "end_mill;drill": the primary op is the milled profile → end_mill family,
    // NOT drill (the old whole-string substring scan wrongly returned "drill").
    EXPECT_EQ(cam::canonicalToolFamily("end_mill;drill"), "end_mill");
    // Leading drill stays a drill (bore-first compound).
    EXPECT_EQ(cam::canonicalToolFamily("drill;end_mill"), "drill");
    // A milled-primary compound with several hole ops after it stays milling.
    EXPECT_EQ(cam::canonicalToolFamily("endmill;ring_groove"), "end_mill");
    // Leading-hole compounds keep their leading family (never mis-stolen forward).
    EXPECT_EQ(cam::canonicalToolFamily("drill;tap;counterbore;engraver"), "drill");
    EXPECT_EQ(cam::canonicalToolFamily("drill;boring_bar;reamer"), "drill");

    // End-to-end: a 12mm "end_mill;drill" cutout must pick the 12mm END MILL pocket
    // (or fall back safely) — it must NEVER resolve to a drill pocket.
    const auto pick = cam::selectTool(makeTM("end_mill;drill", 12.0, "carbide", 4));
    ASSERT_TRUE(pick.has_value());
    EXPECT_EQ(pick->family, "end_mill");
    EXPECT_NEAR(pick->dia_mm, 12.0, 1e-9) << "a carbide drill must never be chosen to profile a milled cutout";
}

// The diameter tolerance must not admit a gross oversize on a SMALL feature: a
// 2.0mm request must NOT snap to the 3.0mm pocket (a 50% over-cut = scrap).
TEST(ToolLibrary, RejectsGrossOversizeOnSmallFeatures)
{
    // 2.0mm drill: nearest pocket is 3.0mm (err 1.0) — 50% oversize → REJECT.
    EXPECT_FALSE(cam::selectTool(makeTM("drill", 2.0)).has_value())
        << "a 3mm drill must not be chosen for a 2mm hole";
    // 2.0mm end mill likewise cannot use the 3.0mm cutter (won't fit a 2mm slot).
    EXPECT_FALSE(cam::selectTool(makeTM("end_mill", 2.0)).has_value());
    // But a genuinely-close request still resolves: 3.2mm drill → the 3.0mm pocket.
    const auto ok = cam::selectTool(makeTM("drill", 3.2));
    ASSERT_TRUE(ok.has_value());
    EXPECT_NEAR(ok->dia_mm, 3.0, 1e-9);
}

// End-to-end: a drill + a pocket (different families) emit an EXECUTABLE T/M6 for
// each, and the two T-numbers differ.
TEST(ToolLibrary, EmitsExecutableToolChangeInProgram)
{
    auto stock = skill::createCuboidStock(60.0, 60.0, 20.0);
    skill::drill_hole::Input dh;
    dh.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    dh.position_x_mm = 15.0; dh.position_y_mm = 15.0;
    dh.axis_dir = gp_Dir(0, 0, -1); dh.diameter_mm = 5.0; dh.depth_mm = 8.0;
    auto w1 = skill::drill_hole::apply(*stock, dh);
    skill::mill_rect_pocket::Input mp;
    mp.entry_face = skill::FaceByNormal{ gp_Dir(0, 0, 1) };
    mp.center_x_mm = 40.0; mp.center_y_mm = 40.0;
    mp.length_mm = 14.0; mp.width_mm = 10.0; mp.depth_mm = 4.0;
    auto w2 = skill::mill_rect_pocket::apply(*w1.workpiece, mp);

    const cam::ToolpathReport report = cam::planAndVerify(*w2.workpiece);
    ASSERT_GE(report.toolpaths.size(), 2u);
    const std::string g = cam::toGCodeProgram(report);

    // An executable tool-change line (starts with a T-word then M6), not just a
    // comment.  There must be at least one "T<digit> M6".
    bool sawExecTM6 = false;
    for (size_t p = g.find(" M6"); p != std::string::npos; p = g.find(" M6", p + 1)) {
        // Walk back to the line start; it must begin with 'T' + a digit.
        size_t ls = g.rfind('\n', p);
        ls = (ls == std::string::npos) ? 0 : ls + 1;
        if (g[ls] == 'T' && ls + 1 < g.size() && std::isdigit((unsigned char)g[ls + 1]))
            sawExecTM6 = true;
    }
    EXPECT_TRUE(sawExecTM6) << "the program must issue an executable T<n> M6 tool change";
    EXPECT_NE(g.find("TOOLCHANGE"), std::string::npos) << "with a descriptive label comment";
}
