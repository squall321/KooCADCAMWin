// @lat: [[engine/skills#Workpiece]]

#include "Workpiece.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <NCollection_IndexedMap.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>

#include <algorithm>
#include <cmath>

namespace koocadcam::skill {

Workpiece::Workpiece(const TopoDS_Shape& shape, const std::string& material)
    : m_shape(shape), m_material(material)
{
    enumerate();
}

void Workpiece::setShape(const TopoDS_Shape& shape)
{
    m_shape = shape;
    m_faces.clear();
    m_edges.clear();
    m_vertices.clear();
    enumerate();
}

void Workpiece::enumerate()
{
    // OCCT 8.0: NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>
    // is the canonical replacement for the deprecated typedef
    // TopTools_IndexedMapOfShape (see process/occt8-migration-cookbook).
    using ShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;

    ShapeMap map;
    TopExp::MapShapes(m_shape, TopAbs_FACE,   map);
    for (int i = 1; i <= map.Extent(); ++i)
        m_faces.push_back(TopoDS::Face(map(i)));

    map.Clear();
    TopExp::MapShapes(m_shape, TopAbs_EDGE,   map);
    for (int i = 1; i <= map.Extent(); ++i)
        m_edges.push_back(TopoDS::Edge(map(i)));

    map.Clear();
    TopExp::MapShapes(m_shape, TopAbs_VERTEX, map);
    for (int i = 1; i <= map.Extent(); ++i)
        m_vertices.push_back(TopoDS::Vertex(map(i)));
}

void Workpiece::addFeature(const FeatureSignature& s)
{
    m_features.push_back(s);
}

// ── Face geometry ────────────────────────────────────────────────────────

gp_Dir Workpiece::faceNormal(int face_id) const
{
    const TopoDS_Face& f = m_faces.at(face_id);
    BRepAdaptor_Surface surf(f);
    const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) / 2.0;
    const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) / 2.0;
    gp_Pnt p;
    gp_Vec du, dv;
    surf.D1(uMid, vMid, p, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-12)
        throw Standard_Failure("Workpiece::faceNormal: degenerate normal");
    n.Normalize();
    // Respect face orientation (reverse if face is flipped relative to surface)
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    return gp_Dir(n);
}

gp_Pnt Workpiece::faceCenter(int face_id) const
{
    const TopoDS_Face& f = m_faces.at(face_id);
    BRepAdaptor_Surface surf(f);
    const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) / 2.0;
    const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) / 2.0;
    return surf.Value(uMid, vMid);
}

double Workpiece::faceArea(int face_id) const
{
    GProp_GProps p;
    BRepGProp::SurfaceProperties(m_faces.at(face_id), p);
    return p.Mass();
}

bool Workpiece::isFacePlanar(int face_id) const
{
    BRepAdaptor_Surface surf(m_faces.at(face_id));
    return surf.GetType() == GeomAbs_Plane;
}

bool Workpiece::isFaceCylinder(int face_id) const
{
    BRepAdaptor_Surface surf(m_faces.at(face_id));
    return surf.GetType() == GeomAbs_Cylinder;
}

// ── Edge geometry ────────────────────────────────────────────────────────

bool Workpiece::isEdgeCircle(int edge_id) const
{
    BRepAdaptor_Curve crv(m_edges.at(edge_id));
    return crv.GetType() == GeomAbs_Circle;
}

gp_Pnt Workpiece::edgeMidPoint(int edge_id) const
{
    BRepAdaptor_Curve crv(m_edges.at(edge_id));
    const double mid = (crv.FirstParameter() + crv.LastParameter()) / 2.0;
    return crv.Value(mid);
}

double Workpiece::edgeMidZ(int edge_id) const
{
    return edgeMidPoint(edge_id).Z();
}

// ── Bounding box ─────────────────────────────────────────────────────────

void Workpiece::boundingBox(double& xMin, double& yMin, double& zMin,
                             double& xMax, double& yMax, double& zMax) const
{
    Bnd_Box box;
    BRepBndLib::AddOptimal(m_shape, box);
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);
}

// ── Datum resolution ─────────────────────────────────────────────────────

namespace {

double angleBetweenDir(const gp_Dir& a, const gp_Dir& b)
{
    const double dot = std::clamp(a.X()*b.X() + a.Y()*b.Y() + a.Z()*b.Z(), -1.0, 1.0);
    return std::acos(dot) * 180.0 / M_PI;
}

}  // namespace

std::optional<int> Workpiece::resolve(const FaceDatum& datum) const
{
    return std::visit([&](const auto& d) -> std::optional<int> {
        using T = std::decay_t<decltype(d)>;

        if constexpr (std::is_same_v<T, FaceIdRef>) {
            if (d.face_id < 0 || d.face_id >= faceCount()) return std::nullopt;
            return d.face_id;
        }
        else if constexpr (std::is_same_v<T, FaceByNormal>) {
            std::vector<int> matches;
            for (int i = 0; i < faceCount(); ++i) {
                if (!isFacePlanar(i)) continue;          // FaceByNormal currently only matches planar
                try {
                    if (angleBetweenDir(faceNormal(i), d.target_normal) < d.tolerance_deg)
                        matches.push_back(i);
                } catch (...) { /* skip degenerate */ }
            }
            if (matches.empty()) return std::nullopt;
            if (d.variant == "any") return matches.front();
            if (d.variant == "largest") {
                return *std::max_element(matches.begin(), matches.end(),
                    [&](int a, int b){ return faceArea(a) < faceArea(b); });
            }
            if (d.variant == "smallest") {
                return *std::min_element(matches.begin(), matches.end(),
                    [&](int a, int b){ return faceArea(a) < faceArea(b); });
            }
            return matches.front();
        }
        else if constexpr (std::is_same_v<T, FaceLargestPlanar>) {
            int best = -1;
            double bestArea = -1.0;
            for (int i = 0; i < faceCount(); ++i) {
                if (!isFacePlanar(i)) continue;
                const double a = faceArea(i);
                if (a > bestArea) { bestArea = a; best = i; }
            }
            return (best >= 0) ? std::optional<int>(best) : std::nullopt;
        }
        else if constexpr (std::is_same_v<T, FaceCylinderByAxis>) {
            int best = -1;
            double bestAngle = d.tolerance_deg;
            for (int i = 0; i < faceCount(); ++i) {
                if (!isFaceCylinder(i)) continue;
                BRepAdaptor_Surface surf(m_faces.at(i));
                gp_Ax1 cylAxis = surf.Cylinder().Axis();
                const double ang = angleBetweenDir(cylAxis.Direction(), d.axis.Direction());
                if (ang < bestAngle) { bestAngle = ang; best = i; }
            }
            return (best >= 0) ? std::optional<int>(best) : std::nullopt;
        }
        else if constexpr (std::is_same_v<T, FaceByRay>) {
            // Ray hit detection — full implementation deferred.  For
            // skill::drill_hole's needs (entry face on a planar block), the
            // FaceByNormal variant covers the common case.
            (void)d;
            return std::nullopt;
        }
        else {
            // Catch-all for FaceTopAtXY (and any future variant alternative).
            // Keeps the chain exhaustive so the surrounding lambda has a
            // total return — avoids MSVC C4702 unreachable trailing code.
            if constexpr (std::is_same_v<T, FaceTopAtXY>) {
                int best = -1;
                double bestZ = -1e30;
                for (int i = 0; i < faceCount(); ++i) {
                    if (!isFacePlanar(i)) continue;
                    BRepAdaptor_Surface surf(m_faces.at(i));
                    gp_Pln pln = surf.Plane();
                    gp_Dir nrm = pln.Axis().Direction();
                    if (std::abs(nrm.Z()) < 1e-6) continue;
                    double zOnPlane = pln.Location().Z()
                        - (nrm.X() * (d.x_mm - pln.Location().X()) +
                           nrm.Y() * (d.y_mm - pln.Location().Y())) / nrm.Z();
                    if (zOnPlane > bestZ) { bestZ = zOnPlane; best = i; }
                }
                return (best >= 0) ? std::optional<int>(best) : std::nullopt;
            }
            else {
                return std::nullopt;
            }
        }
    }, datum);
}

std::optional<int> Workpiece::resolve(const EdgeDatum& datum) const
{
    return std::visit([&](const auto& d) -> std::optional<int> {
        using T = std::decay_t<decltype(d)>;

        if constexpr (std::is_same_v<T, EdgeIdRef>) {
            if (d.edge_id < 0 || d.edge_id >= edgeCount()) return std::nullopt;
            return d.edge_id;
        }
        else if constexpr (std::is_same_v<T, EdgeAtZ>) {
            for (int i = 0; i < edgeCount(); ++i) {
                try {
                    if (std::abs(edgeMidZ(i) - d.z_value) < d.tolerance_mm)
                        return i;
                } catch (...) {}
            }
            return std::nullopt;
        }
        else if constexpr (std::is_same_v<T, EdgeByCircle>) {
            for (int i = 0; i < edgeCount(); ++i) {
                if (!isEdgeCircle(i)) continue;
                BRepAdaptor_Curve crv(m_edges.at(i));
                if (std::abs(crv.Circle().Radius() - d.radius_mm) < d.tolerance_mm)
                    return i;
            }
            return std::nullopt;
        }
        else {
            if constexpr (std::is_same_v<T, EdgeBetweenFaces>) {
                (void)d;  // deferred — needs face/edge adjacency map
            }
            return std::nullopt;
        }
    }, datum);
}

std::optional<int> Workpiece::resolve(const VertexDatum& datum) const
{
    return std::visit([&](const auto& d) -> std::optional<int> {
        using T = std::decay_t<decltype(d)>;

        if constexpr (std::is_same_v<T, VertexIdRef>) {
            if (d.vertex_id < 0 || d.vertex_id >= vertexCount()) return std::nullopt;
            return d.vertex_id;
        }
        else {
            // VertexAtPoint (only remaining alternative).
            int best = -1;
            double bestDist = d.tolerance_mm;
            for (int i = 0; i < vertexCount(); ++i) {
                gp_Pnt p = BRep_Tool::Pnt(m_vertices.at(i));
                const double dx = p.X() - d.target.X();
                const double dy = p.Y() - d.target.Y();
                const double dz = p.Z() - d.target.Z();
                const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (dist < bestDist) { bestDist = dist; best = i; }
            }
            return (best >= 0) ? std::optional<int>(best) : std::nullopt;
        }
    }, datum);
}

}  // namespace koocadcam::skill
