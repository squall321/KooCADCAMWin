#pragma once
// @lat: [[engine/skills#Workpiece]]
//
// A workpiece is a TopoDS_Shape + a stable face/edge/vertex enumeration +
// a history of FeatureSignatures (skills applied so far).
//
// Enumeration uses TopExp::MapShapes for stable IDs across queries on the
// same instance.  IDs are NOT preserved across Boolean operations (each new
// shape gets a fresh enumeration); persistent feature tracking is done via
// FeatureSignature.pattern (geometric fingerprint), not face IDs.

#include "Datum.hpp"
#include "Skill.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <optional>
#include <string>
#include <vector>

namespace koocadcam::skill {

class Workpiece
{
public:
    explicit Workpiece(const TopoDS_Shape& shape,
                       const std::string& material = "aluminum_6061");

    // ── Shape access ──────────────────────────────────────────────────────
    const TopoDS_Shape&  shape()    const { return m_shape; }
    const std::string&   material() const { return m_material; }

    // Replace the shape (called by skills after a Boolean op).  Re-enumerates.
    void setShape(const TopoDS_Shape& shape);

    // ── Topological enumeration ───────────────────────────────────────────
    int faceCount()   const { return static_cast<int>(m_faces.size()); }
    int edgeCount()   const { return static_cast<int>(m_edges.size()); }
    int vertexCount() const { return static_cast<int>(m_vertices.size()); }

    const TopoDS_Face&   face(int id)   const { return m_faces.at(id); }
    const TopoDS_Edge&   edge(int id)   const { return m_edges.at(id); }
    const TopoDS_Vertex& vertex(int id) const { return m_vertices.at(id); }

    // ── Geometry helpers ──────────────────────────────────────────────────
    gp_Dir  faceNormal(int face_id) const;    // outward normal at UV midpoint
    gp_Pnt  faceCenter(int face_id) const;    // surface center
    double  faceArea  (int face_id) const;
    bool    isFacePlanar  (int face_id) const;
    bool    isFaceCylinder(int face_id) const;

    bool    isEdgeCircle (int edge_id) const;
    gp_Pnt  edgeMidPoint(int edge_id) const;
    double  edgeMidZ    (int edge_id) const;

    // ── Bounding info (inferred datum base) ───────────────────────────────
    void  boundingBox(double& xMin, double& yMin, double& zMin,
                      double& xMax, double& yMax, double& zMax) const;

    // ── Datum resolution (hybrid: explicit OR inferred) ───────────────────
    std::optional<int>   resolve(const FaceDatum&   datum) const;
    std::optional<int>   resolve(const EdgeDatum&   datum) const;
    std::optional<int>   resolve(const VertexDatum& datum) const;

    // ── Feature history ───────────────────────────────────────────────────
    //
    // CONTRACT (breakthrough plan B2): every skill apply() MUST return a
    // workpiece whose features() is the FULL build history — all prior
    // signatures, in order, plus exactly one new signature appended.  Use
    // finalizeOutput() below; hand-rolled construction needs a propagation
    // loop and is checked by tests/tools/check_feature_chain.py
    // (ctest: feature_chain_guard) + the SkillContract runtime test.
    void                                      addFeature(const FeatureSignature& s);
    // Set the entire feature list (replaces existing features).  Used by
    // the process Executor to mirror its accumulator into each step's output
    // Workpiece, so wp.features() reflects the full chain history.
    void                                      setFeatures(const std::vector<FeatureSignature>& fs);
    const std::vector<FeatureSignature>&      features() const { return m_features; }

private:
    void enumerate();

    TopoDS_Shape                       m_shape;
    std::string                        m_material;
    std::vector<TopoDS_Face>           m_faces;
    std::vector<TopoDS_Edge>           m_edges;
    std::vector<TopoDS_Vertex>         m_vertices;
    std::vector<FeatureSignature>      m_features;
};

// ── Skill-synthesis output gate ────────────────────────────────────────────
//
// Validate (and heal if needed) a skill-output shape, or throw SkillError.
// Mirrors the engine-side StepGuard / FeatureEditor gate for the skill
// synthesis path: rejects null / empty shapes, runs BRepCheck, attempts a
// single ShapeFix heal when invalid, and throws when unrecoverable.  A
// null/empty/invalid BRep must never be silently accepted as a Workpiece.
TopoDS_Shape validateOrHeal(const TopoDS_Shape& shape,
                            const char* context = "skill output");

// ── Canonical apply() epilogue ─────────────────────────────────────────────
//
// The single sanctioned way to finish a skill apply(): VALIDATES newShape
// (validateOrHeal — heal-or-throw), builds the output workpiece, carries the
// FULL prior feature chain, appends the new signature, and packages the
// SkillOutput.  New skills should end with:
//
//     return finalizeOutput(wp, newShape, sig);
//
// (Delegating compounds that reuse an inner skill's output workpiece are the
// documented exception — see tests/tools/check_feature_chain.py.)
inline SkillOutput finalizeOutput(const Workpiece& wp,
                                  const TopoDS_Shape& newShape,
                                  const FeatureSignature& sig)
{
    auto wpNew = std::make_shared<Workpiece>(validateOrHeal(newShape), wp.material());
    for (const auto& prev : wp.features()) wpNew->addFeature(prev);
    wpNew->addFeature(sig);
    return SkillOutput{ wpNew, sig };
}

}  // namespace koocadcam::skill
