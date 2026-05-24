#include "StepIO.hpp"

#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <spdlog/spdlog.h>

#include <exception>

namespace koocadcam::io {

namespace {

bool ensureMmUnit()
{
    // Set the STEP write unit to MM (idempotent; can be called multiple times).
    Interface_Static::SetCVal("write.step.unit", "MM");
    Interface_Static::SetCVal("write.step.schema", "AP214CD");
    return true;
}

}  // namespace

bool StepIO::write(const TopoDS_Shape& shape,
                   const std::filesystem::path& path,
                   std::string& err) noexcept
{
    err.clear();
    if (shape.IsNull()) {
        err = "StepIO::write: shape is null";
        return false;
    }
    try {
        ensureMmUnit();
        STEPControl_Writer writer;
        IFSelect_ReturnStatus tr = writer.Transfer(shape, STEPControl_AsIs);
        if (tr != IFSelect_RetDone) {
            err = "STEPControl_Writer::Transfer failed (status " + std::to_string(int(tr)) + ")";
            return false;
        }
        IFSelect_ReturnStatus wr = writer.Write(path.string().c_str());
        if (wr != IFSelect_RetDone) {
            err = "STEPControl_Writer::Write failed (status " + std::to_string(int(wr)) + ")";
            return false;
        }
        spdlog::info("STEP written: {}", path.string());
        return true;
    } catch (const std::exception& e) {
        err = std::string("StepIO::write exception: ") + e.what();
        spdlog::error("{}", err);
        return false;
    }
}

std::optional<TopoDS_Shape> StepIO::read(const std::filesystem::path& path,
                                         std::string& err) noexcept
{
    err.clear();
    try {
        STEPControl_Reader reader;
        IFSelect_ReturnStatus rs = reader.ReadFile(path.string().c_str());
        if (rs != IFSelect_RetDone) {
            err = "STEPControl_Reader::ReadFile failed (status " + std::to_string(int(rs)) + ")";
            return std::nullopt;
        }
        Standard_Integer nbRoots = reader.NbRootsForTransfer();
        if (nbRoots <= 0) {
            err = "STEP file contains no roots for transfer";
            return std::nullopt;
        }
        reader.TransferRoots();
        Standard_Integer nbShapes = reader.NbShapes();
        if (nbShapes == 0) {
            err = "STEP file produced no shapes after transfer";
            return std::nullopt;
        }
        if (nbShapes == 1) {
            spdlog::info("STEP read: {} (1 root shape)", path.string());
            return reader.Shape(1);
        }
        // Multiple roots: build a compound.
        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        for (Standard_Integer i = 1; i <= nbShapes; ++i) {
            builder.Add(compound, reader.Shape(i));
        }
        spdlog::info("STEP read: {} ({} root shapes → compound)", path.string(), nbShapes);
        return compound;
    } catch (const std::exception& e) {
        err = std::string("StepIO::read exception: ") + e.what();
        spdlog::error("{}", err);
        return std::nullopt;
    }
}

}  // namespace koocadcam::io
