#pragma once
// @lat: [[engine/parametric-templates#primitives — Cuts & Fuses]]
//
// Boolean op helpers.  `cutMany` / `fuseMany` bundle N tool shapes into a
// single TopoDS_Compound for one BRepAlgoAPI_* call — measurably faster
// than N sequential ops when N > 2 and avoids accumulating BRep history.

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>

#include <vector>

namespace koocadcam::engine::prim {

inline TopoDS_Shape cut(const TopoDS_Shape& base, const TopoDS_Shape& tool)
{
    BRepAlgoAPI_Cut op(base, tool);
    op.Build();
    if (!op.IsDone()) throw Standard_Failure("prim::cut: build failed");
    return op.Shape();
}

// Empty `tools` → return `base` unchanged.
inline TopoDS_Shape cutMany(const TopoDS_Shape& base,
                            const std::vector<TopoDS_Shape>& tools)
{
    if (tools.empty()) return base;
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& t : tools) {
        if (!t.IsNull()) builder.Add(compound, t);
    }
    return cut(base, compound);
}

inline TopoDS_Shape fuse(const TopoDS_Shape& base, const TopoDS_Shape& tool)
{
    BRepAlgoAPI_Fuse op(base, tool);
    op.Build();
    if (!op.IsDone()) throw Standard_Failure("prim::fuse: build failed");
    return op.Shape();
}

// Iterative fuse — BRepAlgoAPI_Fuse only supports binary union, so we accumulate.
// Used for watch lugs (add material protrusions to the case body).
inline TopoDS_Shape fuseMany(const TopoDS_Shape& base,
                             const std::vector<TopoDS_Shape>& tools)
{
    TopoDS_Shape current = base;
    for (const auto& t : tools) {
        if (t.IsNull()) continue;
        current = fuse(current, t);
    }
    return current;
}

}  // namespace koocadcam::engine::prim
