// @lat: [[engine/reverse-route#feature-editor]]

#include "FeatureEditor.hpp"

#include "engine/primitives/Cuts.hpp"
#include "engine/primitives/Tools.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_List.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Lin.hxx>
#include <gp_Vec.hxx>

#include <nlohmann/json.hpp>

#include <cmath>

namespace koocadcam::edit {

namespace pr = koocadcam::engine::prim;

namespace {

double volumeOf(const TopoDS_Shape& s)
{
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

double bboxDiag(const TopoDS_Shape& s)
{
    Bnd_Box box;
    BRepBndLib::Add(s, box);
    double xm, ym, zm, xx, yx, zx;
    box.Get(xm, ym, zm, xx, yx, zx);
    return std::sqrt((xx - xm) * (xx - xm) + (yx - ym) * (yx - ym)
                     + (zx - zm) * (zx - zm));
}

}  // namespace

std::optional<HoleSpec> holeSpecFromRecovered(const nlohmann::json& p)
{
    if (!p.is_object()) return std::nullopt;
    if (!p.contains("diameter_mm") || !p["diameter_mm"].is_number())
        return std::nullopt;
    if (!p.contains("position_x_mm") || !p.contains("position_y_mm"))
        return std::nullopt;

    HoleSpec h;
    h.diameter_mm = p["diameter_mm"].get<double>();
    h.entry       = gp_Pnt(p.value("position_x_mm", 0.0),
                           p.value("position_y_mm", 0.0),
                           p.value("position_z_mm", 0.0));
    if (p.contains("axis_dir") && p["axis_dir"].is_array() &&
        p["axis_dir"].size() == 3) {
        const double ax = p["axis_dir"][0].get<double>();
        const double ay = p["axis_dir"][1].get<double>();
        const double az = p["axis_dir"][2].get<double>();
        if (std::sqrt(ax * ax + ay * ay + az * az) > 1e-9)
            h.axis = gp_Dir(ax, ay, az);
    }
    h.through  = p.value("through_hole", false);
    h.depth_mm = p.value("depth_mm", 0.0);
    if (!h.through && h.depth_mm <= 0.0) return std::nullopt;
    return h;
}

TopoDS_Shape defeatureHole(const TopoDS_Shape& shape, const HoleSpec& hole,
                           std::string& err)
{
    const double r = hole.diameter_mm / 2.0;
    if (r <= 0.0) { err = "hole diameter must be > 0"; return {}; }

    const gp_Lin axisLine(hole.entry, hole.axis);
    const double radTol = std::max(0.05, 0.01 * r);

    // Collect the hole's faces by GEOMETRY (face IDs are not stable across
    // STEP round-trips; axis+radius are):
    //   - every cylindrical face whose axis is colinear with the hole axis
    //     and whose radius matches;
    //   - for blind holes, the planar bottom: normal along the axis,
    //     centroid on the axis, at a depth inside the hole.
    NCollection_List<TopoDS_Shape> toRemove;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& f = TopoDS::Face(ex.Current());
        try {
            BRepAdaptor_Surface surf(f);
            if (surf.GetType() == GeomAbs_Cylinder) {
                const gp_Cylinder cyl = surf.Cylinder();
                if (std::abs(cyl.Radius() - r) > radTol) continue;
                const gp_Dir cd = cyl.Axis().Direction();
                const double dot = std::abs(cd.X() * hole.axis.X() +
                                            cd.Y() * hole.axis.Y() +
                                            cd.Z() * hole.axis.Z());
                if (dot < 0.999) continue;
                if (axisLine.Distance(cyl.Axis().Location()) > radTol + 0.05)
                    continue;
                toRemove.Append(f);
            } else if (!hole.through && surf.GetType() == GeomAbs_Plane) {
                // Candidate blind bottom.
                GProp_GProps props;
                BRepGProp::SurfaceProperties(f, props);
                const gp_Pnt c = props.CentreOfMass();
                const gp_Vec rel(hole.entry, c);
                const double along = rel.X() * hole.axis.X() +
                                     rel.Y() * hole.axis.Y() +
                                     rel.Z() * hole.axis.Z();
                if (along < 0.5 || along > hole.depth_mm + 0.1) continue;
                const gp_Vec radial =
                    rel - gp_Vec(hole.axis) * along;
                if (radial.Magnitude() > 0.8 * r) continue;
                toRemove.Append(f);
            }
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    if (toRemove.IsEmpty()) {
        err = "no faces matched the hole (axis/radius) — nothing to defeature";
        return {};
    }

    const double vBefore = volumeOf(shape);

    BRepAlgoAPI_Defeaturing df;
    df.SetShape(shape);
    df.AddFacesToRemove(toRemove);
    try {
        df.Build();
    } catch (const Standard_Failure&) {
        err = "defeaturing raised an OCCT exception";
        return {};
    }
    if (df.HasErrors() || df.Shape().IsNull()) {
        err = "defeaturing failed (faces could not be removed/healed)";
        return {};
    }

    const TopoDS_Shape healed = df.Shape();

    // The heal must RESTORE material (the hole volume comes back) and leave
    // a valid solid.  A healed shape that lost volume means the removal set
    // was wrong — refuse rather than hand back damaged geometry.
    if (volumeOf(healed) < vBefore - 1e-6) {
        err = "defeaturing reduced volume — wrong face set, refusing result";
        return {};
    }
    BRepCheck_Analyzer check(healed);
    if (!check.IsValid()) {
        err = "healed solid failed BRepCheck validation";
        return {};
    }
    return healed;
}

TopoDS_Shape cutHole(const TopoDS_Shape& shape, const HoleSpec& hole)
{
    const double diag = bboxDiag(shape);
    const gp_Pnt start(hole.entry.X() - hole.axis.X() * 0.1 * diag,
                       hole.entry.Y() - hole.axis.Y() * 0.1 * diag,
                       hole.entry.Z() - hole.axis.Z() * 0.1 * diag);
    const double length = hole.through ? diag * 1.2
                                       : 0.1 * diag + hole.depth_mm;
    const gp_Ax2 ax(start, hole.axis);
    return pr::cut(shape, pr::cylinder(ax, hole.diameter_mm / 2.0, length));
}

TopoDS_Shape editHole(const TopoDS_Shape& shape, const HoleSpec& oldHole,
                      const HoleSpec& newHole, std::string& err)
{
    const TopoDS_Shape healed = defeatureHole(shape, oldHole, err);
    if (healed.IsNull()) return {};
    try {
        return cutHole(healed, newHole);
    } catch (const Standard_Failure&) {
        err = "recut raised an OCCT exception";
        return {};
    }
}

}  // namespace koocadcam::edit
