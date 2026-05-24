// @lat: [[process/test-strategy#smoke]]
// Build/link smoke test: include core OCCT + Qt headers, instantiate primitives.

#include <gtest/gtest.h>

#include <Standard_Version.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>

#include <QCoreApplication>

TEST(Smoke, OcctVersionIs8)
{
    EXPECT_GE(OCC_VERSION_MAJOR, 8);
}

TEST(Smoke, GpPntConstructs)
{
    gp_Pnt p(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(p.X(), 1.0);
    EXPECT_DOUBLE_EQ(p.Y(), 2.0);
    EXPECT_DOUBLE_EQ(p.Z(), 3.0);
}

TEST(Smoke, TopoDsShapeIsNullByDefault)
{
    TopoDS_Shape s;
    EXPECT_TRUE(s.IsNull());
}

TEST(Smoke, QtCoreLinks)
{
    // Just verify that Qt symbols resolved; no QCoreApplication instance needed.
    const char* name = "smoke";
    int argc = 1;
    char* argv[] = { const_cast<char*>(name), nullptr };
    QCoreApplication app(argc, argv);
    EXPECT_FALSE(app.applicationName().isNull());
}
