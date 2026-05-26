#pragma once
// @lat: [[engine/parametric-templates#primitives — Cuts]]
//
// Boolean cut helpers.  `cutMany` bundles N tool shapes into a single
// TopoDS_Compound for one BRepAlgoAPI_Cut call — measurably faster than
// N sequential cuts when N > 2 and avoids accumulating BRep history.

#include <BRepAlgoAPI_Cut.hxx>
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

}  // namespace koocadcam::engine::prim
