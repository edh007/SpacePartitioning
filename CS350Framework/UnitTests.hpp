///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include "Application.hpp"

typedef void(*UnitTestFn)(const std::string& testName, FILE* file);

// Simple class that wraps the function pointer for a unit test.
// Makes it easier to pass in the test name and to clear statistics between each test run.
struct UnitTestWrapper
{
  UnitTestWrapper() {}
  UnitTestWrapper(UnitTestFn fn, const char* testName);

  void Run(FILE* outputFile, FILE* timingsFile);

  UnitTestFn mFn;
  const char* mTestName;
};

typedef std::vector<UnitTestWrapper> AssignmentUnitTestList;
extern std::vector<AssignmentUnitTestList> mTestFns;

void InitializeAssignment1Tests();
void InitializeAssignment2Tests();
void InitializeAssignment3Tests();
void InitializeAssignment4Tests();
void InitializeAssignment5Tests();


// Simple component that calls a unit test function pointer during update
class SimpleUnitTesterComponent : public Component
{
public:
  DeclareComponent(SimpleUnitTesterComponent);

  SimpleUnitTesterComponent(UnitTestFn testFn, const std::string& testName);
  void Update(float dt) override;

  UnitTestFn mUnitTestFn;
  std::string mTestName;
};

// Simple level to run a unit test (creates an object with the unit test component)
class SimpleUnitTestLevel : public Level
{
public:
  SimpleUnitTestLevel(const std::string& levelName, UnitTestFn testFn);
  void Load(Application* application) override;
  std::string GetName() const override;

  UnitTestFn mUnitTestFn;
  std::string mLevelName;
};

#define DeclareSimpleUnitTest(fn, assignmentUnitTestList) \
  assignmentUnitTestList.push_back(UnitTestWrapper(fn, #fn));

// A printf style string formatter
std::string FormatString(const char* format, ...);
// Fix some weird floating point issues for printing such as -0.0 vs. 0.0.
float CleanupFloat(float val);
// Printing functions for the unit tests. Mostly control the format specifiers.
std::string PrintFloat(float val);
std::string PrintVector2(float x, float y);
std::string PrintVector2(const Vector2& vec2);
std::string PrintVector3(float x, float y, float z);
std::string PrintVector3(const Vector3& vec3);
std::string PrintVector4(const Vector4& vec4);
std::string PrintMatrix3(const Matrix3& mat3);
std::string PrintAabb(const Aabb& aabb);
std::string PrintSphere(const Sphere& sphere);
std::string PrintTriangle(const Triangle& tri);
std::string PrintPlane(const Plane& plane);
void PrintAabbData(FILE* outFile, const SpatialPartitionQueryData& rhs);
void PrintSphereData(FILE* outFile, const SpatialPartitionQueryData& rhs);
void PrintTestHeader(FILE* outFile, const std::string& testName);

// Function pointer typedef for printing SpatialPartitionData (to choose between checking the aabb and sphere)
typedef void(*PrintDataFn)(FILE* outFile, const SpatialPartitionQueryData& rhs);

// Printing functions for spatial partition results
void PrintSpatialPartitionStructure(SpatialPartition& spatialPartition, FILE* outFile, bool shouldSort = true);
void PrintSpatialPartitionStructure(DynamicAabbTree& spatialPartition, FILE* outFile, bool shouldSort = true);
void PrintSpatialPartitionStructure(BspTree& spatialPartition, FILE* outFile, const std::string& headerName = std::string());
void PrintRayCastResults(SpatialPartition& spatialPartition, const Ray& ray, FILE* outFile);
void PrintFrustumCastResults(SpatialPartition& spatialPartition, const Frustum& frustum, FILE* outFile);
void PrintSpatialPartitionSelfQuery(SpatialPartition& spatialPartition, FILE* outFile);

bool operator<(const Triangle& rhs, const Triangle& lhs);

namespace PrintModes
{
enum Enum
{
  Normal = 1 << 0,
  GraphViz = 1 << 1,
  EmbeddedTree = 1 << 2,
};
}//namespace PrintModes

//--------------------------------------------------------------------BasePrinter
/// Printer base class for spatial partitions. Contains useful helpers to uniquely
/// identify nodes and to print certain formats.
class BasePrinter
{
public:
  BasePrinter();
  virtual ~BasePrinter() {};

  int GetId(void* data);

  void PrintGraphVizHeader();
  void PrintNullNodeDescriptions();
  void PrintNodeConnection(int parentId, int childId, bool constrained);
  void PrintGraphVizFooter();

  void PrintMode();
  virtual void PrintRegular(bool verbose) {};
  virtual void PrintGraphViz(bool verbose) {};
  virtual void PrintEmbedded(bool verbose) {};

  std::unordered_map<void*, int> mIds;
  int mCurrentId;
  int mNullNodes;
  bool mVerbose;
  PrintModes::Enum mMode;
  FILE* mFile;
};

//--------------------------------------------------------------------AabbTreePrinter
/// Generic spatial partition printer. Uses the legacy 'FilloutData' interface function
/// to print data. Assumes the underlying structure is unordered.
class BasicSpatialPartitionPrinter : public BasePrinter
{
public:
  BasicSpatialPartitionPrinter();

  void PrintStructure(SpatialPartition& spatialPartition);

  void PrintRegular(bool verbose) override;
  void PrintGraphViz(bool verbose) override;
  void PrintEmbedded(bool verbose) override;

  std::vector<SpatialPartitionQueryData> mData;
};

//--------------------------------------------------------------------AabbTreePrinter
/// Helper to print an aabb tree.
class AabbTreePrinter : public BasePrinter
{
public:
  AabbTreePrinter();
  
  struct NodeData
  {
    int mId;
    int mParentId;
    int mLeftId;
    int mRightId;
    DynamicAabbTreeNode* mNode;
  };

  void PrintStructure(DynamicAabbTree& tree);
  void FlattenStructure(DynamicAabbTreeNode* node);

  void PrintRegular(bool verbose) override;
  void PrintGraphVizNodeDescriptions(bool onlyInvisibleNodes);
  void PrintGraphVizNodeConnections();
  void PrintGraphViz(bool verbose) override;
  void PrintEmbedded(bool verbose) override;
  void PrintEmbedded(bool verbose, int nodeId, std::string indent, bool last);
  
  std::vector<NodeData> mNodeData;
  std::unordered_map<int, int> mIdToIndexMap;
};

//--------------------------------------------------------------------BspTreePrinter
/// Helper to print a bsp tree.
class BspTreePrinter : public BasePrinter
{
public:
  BspTreePrinter();

  struct NodeData
  {
    int mId;
    int mFrontId;
    int mBackId;
    int mDepth;
    BspTreeNode* mNode;
  };

  void PrintStructure(BspTree& tree);
  void FlattenStructure(BspTreeNode* node, int depth = 0);
  void PrintRegular(bool verbose) override;
  void PrintGraphVizNodeDescriptions(bool onlyInvisibleNodes);
  void PrintGraphVizNodeConnections();
  void PrintGraphViz(bool verbose) override;
  void PrintEmbedded(bool verbose) override;
  void PrintEmbedded(bool verbose, int nodeId, std::string indent, bool last);

  void PrintTriangleList(TriangleList& triangles);

  std::vector<NodeData> mNodeData;
  std::unordered_map<int, int> mIdToIndexMap;
};
