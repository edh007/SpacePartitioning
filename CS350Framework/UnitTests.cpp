#include "Precompiled.hpp"
#include "UnitTests.hpp"
#include "DebugDraw.hpp"

std::vector<AssignmentUnitTestList> mTestFns;

//-----------------------------------------------------------------------------UnitTestWrapper
UnitTestWrapper::UnitTestWrapper(UnitTestFn fn, const char* testName)
  : mFn(fn), mTestName(testName)
{

}

void UnitTestWrapper::Run(FILE* outputFile, FILE* timingsFile)
{
  Application::mStatistics.Clear();
  mFn(mTestName, outputFile);

  if(timingsFile == nullptr)
    return;
  
  double runningAverage = 0.0;
  size_t count = 10;
  for(size_t i = 0; i < count; ++i)
  {
    LARGE_INTEGER frequency, start, end;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);
    mFn(mTestName, nullptr);
    QueryPerformanceCounter(&end);
    runningAverage += (end.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart;
  }
  runningAverage /= count;
  fprintf(timingsFile, "  Timing test (%s): %lf ms\n", mTestName, runningAverage);
}

//-----------------------------------------------------------------------------SimpleUnitTesterComponent
SimpleUnitTesterComponent::SimpleUnitTesterComponent(UnitTestFn testFn, const std::string& testName)
{
  mUnitTestFn = testFn;
  mTestName = testName;
}
  
void SimpleUnitTesterComponent::Update(float dt)
{
  mUnitTestFn(mTestName, nullptr);
}

//-----------------------------------------------------------------------------SimpleUnitTesterComponent
SimpleUnitTestLevel::SimpleUnitTestLevel(const std::string& levelName, UnitTestFn testFn)
{
  mUnitTestFn = testFn;
  mLevelName = levelName;
}

void SimpleUnitTestLevel::Load(Application* application)
{
  GameObject* obj = application->CreateEmptyObject("Tester");
  obj->Add(new SimpleUnitTesterComponent(mUnitTestFn, mLevelName));
}

std::string SimpleUnitTestLevel::GetName() const
{
  return mLevelName;
}

//-----------------------------------------------------------------------------Unit Test Helper Functions
std::string FormatString(const char* format, ...)
{
  va_list va;
  va_start(va, format);

  int bufferSize;
  bufferSize = _vscprintf(format, va) + 1;
  char* stringBuffer = (char*)alloca((bufferSize + 1)*sizeof(char));
  stringBuffer[bufferSize] = '\0';
  vsprintf_s(stringBuffer, bufferSize, format, va);

  va_end(va);

  return std::string(stringBuffer);
}

float CleanupFloat(float val)
{
  // To deal with the difference between -0.0 and 0.0 if the value is below our
  /// printing epsilon (2 decimal places) then just clamp to 0
  if(Math::Abs(val) < 0.01f)
    val = 0;

  if(val == Math::PositiveMax())
    return val;

  // Round the float to 2 decimal places (0.01)
  float scale = std::pow(10.0f, -2.0f);
  return std::round(val / scale) * scale;

  return val;
}

std::string PrintFloat(float val)
{
  return FormatString("%.2f", CleanupFloat(val));
}

std::string PrintVector2(float x, float y)
{
  return FormatString("(%.2f, %.2f)", CleanupFloat(x), CleanupFloat(y));
}

std::string PrintVector2(const Vector2& vec2)
{
  return PrintVector2(vec2.x, vec2.y);
}

std::string PrintVector3(float x, float y, float z)
{
  return FormatString("(%.2f, %.2f, %.2f)", CleanupFloat(x), CleanupFloat(y), CleanupFloat(z));
}

std::string PrintVector3(const Vector3& vec3)
{
  return PrintVector3(vec3.x, vec3.y, vec3.z);
}
std::string PrintVector4(const Vector4& vec4)
{
  return FormatString("(%.2f, %.2f, %.2f, %.2f)", CleanupFloat(vec4.x), CleanupFloat(vec4.y), CleanupFloat(vec4.z), CleanupFloat(vec4.w));
}

std::string PrintMatrix3(const Matrix3& mat3)
{
  return FormatString("((%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f))",
                      CleanupFloat(mat3.m00), CleanupFloat(mat3.m01), CleanupFloat(mat3.m02),
                      CleanupFloat(mat3.m10), CleanupFloat(mat3.m11), CleanupFloat(mat3.m12),
                      CleanupFloat(mat3.m20), CleanupFloat(mat3.m21), CleanupFloat(mat3.m22));
}

std::string PrintAabb(const Aabb& aabb)
{
  return FormatString("Aabb(%s, %s)", PrintVector3(aabb.mMin).c_str(), PrintVector3(aabb.mMax).c_str());
}

std::string PrintSphere(const Sphere& sphere)
{
  return FormatString("Sphere(%s, %s)", PrintVector3(sphere.mCenter).c_str(), PrintFloat(sphere.mRadius).c_str());
}

std::string PrintTriangle(const Triangle& tri)
{
  return FormatString("Triangle(%s, %s, %s)", PrintVector3(tri.mPoints[0]).c_str(), PrintVector3(tri.mPoints[1]).c_str(), PrintVector3(tri.mPoints[2]).c_str());
}

std::string PrintPlane(const Plane& plane)
{
  return FormatString("Plane%s", PrintVector4(plane.mData).c_str());
}

void PrintAabbData(FILE* outFile, const SpatialPartitionQueryData& rhs)
{
  if(outFile != nullptr)
    fprintf(outFile, "    Depth(%d) ClientData(%p) %s\n", rhs.mDepth, rhs.mClientData, PrintAabb(rhs.mAabb).c_str());
}

void PrintSphereData(FILE* outFile, const SpatialPartitionQueryData& rhs)
{
  if(outFile != nullptr)
    fprintf(outFile, "    Depth(%d) ClientData(%p) %s\n", rhs.mDepth, rhs.mClientData, PrintSphere(rhs.mBoundingSphere).c_str());
}

void PrintTestHeader(FILE* outFile, const std::string& testName)
{
  if(outFile != nullptr)
  {
    fprintf(outFile, "\n");
    fprintf(outFile, "////////////////////////////////////////////////////////////\n");
    fprintf(outFile, "%s\n", testName.c_str());
    fprintf(outFile, "////////////////////////////////////////////////////////////\n");
  }
}

void SetupPrinter(BasePrinter& printer, const std::string& headerName, FILE* file)
{
  printer.mFile = file;

  CommandLine* commandLine = CommandLine::GetInstance();
  bool simple = commandLine->ContainsArgument("-simple");
  bool basic = commandLine->ContainsArgument("-basic");
  bool graphviz = commandLine->ContainsArgument("-graphviz");
  bool embedded = commandLine->ContainsArgument("-embedded");
  printer.mVerbose = !simple;
  printer.mMode = PrintModes::EmbeddedTree;
  if(basic)
    printer.mMode = PrintModes::Normal;
  else if(graphviz)
    printer.mMode = PrintModes::GraphViz;

  if(file != nullptr)
  {
    std::string header = headerName;
    if(headerName.empty())
      header = "Test Spatial Partition Structure";
    fprintf(file, "  %s:\n", header.c_str());
  }
}

void PrintSpatialPartitionStructure(SpatialPartition& spatialPartition, FILE* outFile, bool shouldSort)
{
  BasicSpatialPartitionPrinter printer;
  SetupPrinter(printer, std::string(), outFile);

  printer.PrintStructure(spatialPartition);
}

void PrintSpatialPartitionStructure(DynamicAabbTree& spatialPartition, FILE* outFile, bool shouldSort)
{
  AabbTreePrinter printer;
  SetupPrinter(printer, std::string(), outFile);

  printer.PrintStructure(spatialPartition);
}

void PrintSpatialPartitionStructure(BspTree& spatialPartition, FILE* outFile, const std::string& headerName)
{
  BspTreePrinter printer;
  SetupPrinter(printer, headerName, outFile);

  printer.PrintStructure(spatialPartition);
}

void PrintRayCastResults(SpatialPartition& spatialPartition, const Ray& ray, FILE* outFile)
{
  CastResults castResults;
  spatialPartition.CastRay(ray, castResults);

  // Sort the results to guarantee a consistent ordering
  sort(castResults.mResults.begin(), castResults.mResults.end());

  ray.DebugDraw(10.0f);
  spatialPartition.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Matrix4::cIdentity);

  if(outFile == nullptr)
    return;

  fprintf(outFile, "  Test RayCast:\n");
  if(castResults.mResults.empty())
  {
    fprintf(outFile, "    Empty\n");
  }
  else
  {
    for(size_t i = 0; i < castResults.mResults.size(); ++i)
    {
      fprintf(outFile, "    %p %.2f\n", castResults.mResults[i].mClientData, CleanupFloat(castResults.mResults[i].mTime));
    }
  }
}

void PrintFrustumCastResults(SpatialPartition& spatialPartition, const Frustum& frustum, FILE* outFile)
{
  CastResults castResults;
  spatialPartition.CastFrustum(frustum, castResults);

  // Sort the results so we're guaranteed to always produce the same order
  sort(castResults.mResults.begin(), castResults.mResults.end());

  frustum.DebugDraw();
  spatialPartition.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Matrix4::cIdentity);

  if(outFile != nullptr)
  {
    fprintf(outFile, "  Test FrustumCast:\n");
    for(size_t i = 0; i < castResults.mResults.size(); ++i)
    {
      fprintf(outFile, "    %p\n", castResults.mResults[i].mClientData);
    }
  }
}

void PrintSpatialPartitionSelfQuery(SpatialPartition& spatialPartition, FILE* outFile)
{
  QueryResults results;
  spatialPartition.SelfQuery(results);
  std::sort(results.mResults.begin(), results.mResults.end());

  spatialPartition.DebugDraw(Application::mUnitTestValues.mDebugDrawLevel, Matrix4::cIdentity);

  if(outFile != nullptr)
  {
    fprintf(outFile, "  Test SelfQuery:\n");
    for(size_t i = 0; i < results.mResults.size(); ++i)
    {
      fprintf(outFile, "    (%p,%p)\n", results.mResults[i].mClientData0, results.mResults[i].mClientData1);
    }
  }
}

bool operator<(const Triangle& rhs, const Triangle& lhs)
{
  // Quick and dirty sorting of triangles. Average their points to compute the
  // centers, round the floats and then sort by x then y then z
  Vector3 centerRhs = (rhs.mPoints[0] + rhs.mPoints[1] + rhs.mPoints[2]) / 3.0f;
  Vector3 centerLhs = (lhs.mPoints[0] + lhs.mPoints[1] + lhs.mPoints[2]) / 3.0f;
  for(size_t i = 0; i < 3; ++i)
    centerRhs[i] = CleanupFloat(centerRhs[i]);
  for(size_t i = 0; i < 3; ++i)
    centerLhs[i] = CleanupFloat(centerLhs[i]);
  
  for(size_t j = 0; j < 3; ++j)
  {
    if(centerRhs[j] != centerLhs[j])
      return centerRhs[j] < centerLhs[j];
  }
  
  return false;
}

//--------------------------------------------------------------------BasePrinter
BasePrinter::BasePrinter()
{
  mMode = PrintModes::Normal;
  mFile = nullptr;
  mCurrentId = 0;
  mNullNodes = 0;
  mVerbose = false;
}

int BasePrinter::GetId(void* data)
{
  if(data == nullptr)
  {
    ++mNullNodes;
    return -mNullNodes;
  }

  auto it = mIds.find(data);
  if(it != mIds.end())
    return it->second;

  int id = mCurrentId++;
  mIds[data] = id;
  return id;
}

void BasePrinter::PrintGraphVizHeader()
{
  fprintf(mFile, "digraph {\n");
  fprintf(mFile, "ordering = \"out\"\n");
  fprintf(mFile, "node [shape=record]\n");
}

void BasePrinter::PrintNullNodeDescriptions()
{
  for(int i = 0; i <= mNullNodes; ++i)
    fprintf(mFile, "-%d [style = invis]\n", i);
}

void BasePrinter::PrintNodeConnection(int parentId, int childId, bool constrained)
{
  if(constrained)
  {
    if(childId >= 0)
      fprintf(mFile, "%d -> %d [constraint = false]\n", parentId, childId);
  }
  else
    fprintf(mFile, "%d -> %d\n", parentId, childId);
}

void BasePrinter::PrintGraphVizFooter()
{
  fprintf(mFile, "}\n");
}

void BasePrinter::PrintMode()
{
  if(mMode == PrintModes::Normal)
    PrintRegular(mVerbose);
  else if(mMode == PrintModes::GraphViz)
    PrintGraphViz(mVerbose);
  else if(mMode == PrintModes::EmbeddedTree)
    PrintEmbedded(mVerbose);
}

//--------------------------------------------------------------------BasicSpatialPartitionPrinter
BasicSpatialPartitionPrinter::BasicSpatialPartitionPrinter()
{

}

void BasicSpatialPartitionPrinter::PrintStructure(SpatialPartition& spatialPartition)
{
  if(mFile == nullptr)
    return;

  spatialPartition.FilloutData(mData);
  std::sort(mData.begin(), mData.end());
  if(mData.empty())
  {
    fprintf(mFile, "    Empty\n");
    return;
  }

  PrintMode();
}

void BasicSpatialPartitionPrinter::PrintRegular(bool verbose)
{
  for(size_t i = 0; i < mData.size(); ++i)
  {
    SpatialPartitionQueryData& data = mData[i];
    fprintf(mFile, "    ClientData(%p) %s\n", data.mClientData, PrintSphere(data.mBoundingSphere).c_str());
  }
}

void BasicSpatialPartitionPrinter::PrintGraphViz(bool verbose)
{
  //Not supported
  PrintRegular(verbose);
}

void BasicSpatialPartitionPrinter::PrintEmbedded(bool verbose)
{
  //Not supported
  PrintRegular(verbose);
}

//--------------------------------------------------------------------AabbTreePrinter
AabbTreePrinter::AabbTreePrinter()
{
  
}

void AabbTreePrinter::PrintStructure(DynamicAabbTree& tree)
{
  if(mFile == nullptr)
    return;

  DynamicAabbTreeNode* root = tree.GetRoot();
  if(root == nullptr)
  { 
    fprintf(mFile, "    Empty\n");
    return;
  }

  FlattenStructure(root);
  PrintMode();
}

void AabbTreePrinter::FlattenStructure(DynamicAabbTreeNode* node)
{
  if(node == nullptr)
    return;

  NodeData data;
  data.mParentId = GetId(node->GetParent());
  data.mId = GetId(node);
  data.mLeftId = GetId(node->GetLeftChild());
  data.mRightId = GetId(node->GetRightChild());
  data.mNode = node;
  mNodeData.push_back(data);
  mIdToIndexMap[data.mId] = mNodeData.size() - 1;

  FlattenStructure(node->GetLeftChild());
  FlattenStructure(node->GetRightChild());
}

void AabbTreePrinter::PrintRegular(bool verbose)
{
  for(size_t i = 0; i < mNodeData.size(); ++i)
  {
    NodeData& nodeData = mNodeData[i];

    Aabb aabb = nodeData.mNode->GetAabb();
    void* clientData = nodeData.mNode->GetClientData();
    int height = nodeData.mNode->GetHeight();

    if(!verbose)
      fprintf(mFile, "    Depth(%d) ClientData(%p) %s\n", height, clientData, PrintAabb(aabb).c_str());
    else
      fprintf(mFile, "    Node(%d): Parent(%d) Left(%d) Right(%d) Height(%d) ClientData(%p) %s\n", nodeData.mId, nodeData.mParentId, nodeData.mLeftId, nodeData.mRightId, height, clientData, PrintAabb(aabb).c_str());
  }
}

void AabbTreePrinter::PrintGraphVizNodeDescriptions(bool onlyInvisibleNodes)
{
  PrintNullNodeDescriptions();

  if(onlyInvisibleNodes)
    return;
  
  for(size_t i = 0; i < mNodeData.size(); ++i)
  {
    NodeData& nodeData = mNodeData[i];
    int id = nodeData.mId;
    int height = nodeData.mNode->GetHeight();
    std::string aabbStr = PrintAabb(nodeData.mNode->GetAabb());
    void* clientData = nodeData.mNode->GetClientData();
    fprintf(mFile, "%d [label = \"%d | {Height = %d | %s | ClientData = %p}\"]\n", id, id, height, aabbStr.c_str(), clientData);
  }
}

void AabbTreePrinter::PrintGraphVizNodeConnections()
{
  for(size_t i = 0; i < mNodeData.size(); ++i)
  {
    NodeData& nodeData = mNodeData[i];

    Aabb aabb = nodeData.mNode->GetAabb();
    void* clientData = nodeData.mNode->GetClientData();
    int height = nodeData.mNode->GetHeight();

    // Don't print any parent connections to null nodes
    PrintNodeConnection(nodeData.mId, nodeData.mParentId, true);
    PrintNodeConnection(nodeData.mId, nodeData.mLeftId, false);
    PrintNodeConnection(nodeData.mId, nodeData.mRightId, false);
  }
}

void AabbTreePrinter::PrintGraphViz(bool verbose)
{
  PrintGraphVizHeader();
  PrintGraphVizNodeDescriptions(!verbose);
  PrintGraphVizNodeConnections();
  PrintGraphVizFooter();
}

void AabbTreePrinter::PrintEmbedded(bool verbose)
{
  PrintEmbedded(verbose, mNodeData[0].mId, "  ", true);
}

void AabbTreePrinter::PrintEmbedded(bool verbose, int nodeId, std::string indent, bool last)
{
  if(mIdToIndexMap.find(nodeId) == mIdToIndexMap.end())
    return;

  fprintf(mFile, "%s", indent.c_str());
  if(last)
  {
    fprintf(mFile, "\\-");
    indent += "  ";
  }
  else
  {
    fprintf(mFile, "|-");
    indent += "| ";
  }

  int index = mIdToIndexMap[nodeId];
  NodeData& nodeData = mNodeData[index];

  if(!verbose)
    fprintf(mFile, "(%d)\n", nodeData.mId);
  else
  {
    Aabb aabb = nodeData.mNode->GetAabb();
    void* clientData = nodeData.mNode->GetClientData();
    int height = nodeData.mNode->GetHeight();
    fprintf(mFile, "(%d): Parent(%d) Height(%d) ClientData(%p) %s\n", nodeData.mId, nodeData.mParentId, height, clientData, PrintAabb(aabb).c_str());
  }

  PrintEmbedded(verbose, nodeData.mLeftId, indent, false);
  PrintEmbedded(verbose, nodeData.mRightId, indent, true);
}

//--------------------------------------------------------------------BspTreePrinter
BspTreePrinter::BspTreePrinter()
{

}

void BspTreePrinter::PrintStructure(BspTree& tree)
{
  if(mFile == nullptr)
    return;

  BspTreeNode* root = tree.GetRoot();
  if(root == nullptr)
  {
    fprintf(mFile, "    Empty\n");
    return;
  }

  FlattenStructure(root);
  PrintMode();
}

void BspTreePrinter::FlattenStructure(BspTreeNode* node, int depth)
{
  if(node == nullptr)
    return;

  NodeData data;
  data.mId = GetId(node);
  data.mFrontId = GetId(node->GetFrontChild());
  data.mBackId = GetId(node->GetBackChild());
  data.mNode = node;
  data.mDepth = depth;
  mNodeData.push_back(data);
  mIdToIndexMap[data.mId] = mNodeData.size() - 1;

  FlattenStructure(node->GetFrontChild(), depth + 1);
  FlattenStructure(node->GetBackChild(), depth + 1);
}

void BspTreePrinter::PrintRegular(bool verbose)
{
  for(size_t i = 0; i < mNodeData.size(); ++i)
  {
    NodeData& nodeData = mNodeData[i];
    BspTreeNode* node = nodeData.mNode;

    TriangleList triangles;
    node->GetTriangles(triangles);
    std::sort(triangles.begin(), triangles.end());
    if(!verbose)
      fprintf(mFile, "    Node: Depth(%d)\n", nodeData.mDepth);
    else
      fprintf(mFile, "    Node(%d): Depth(%d) Front(%d) Back(%d)\n", nodeData.mId, nodeData.mDepth, nodeData.mFrontId, nodeData.mBackId);

    fprintf(mFile, "      Plane: %s\n", PrintVector4(node->GetSplitPlane().mData).c_str());
    for(size_t triId = 0; triId < triangles.size(); ++triId)
      fprintf(mFile, "      %s\n", PrintTriangle(triangles[triId]).c_str());
  }
}

void BspTreePrinter::PrintGraphVizNodeDescriptions(bool onlyInvisibleNodes)
{
  PrintNullNodeDescriptions();

  if(onlyInvisibleNodes)
    return;

  // Print descriptions of the remaining nodes
  for(size_t i = 0; i < mNodeData.size(); ++i)
  {
    NodeData& nodeData = mNodeData[i];
    int id = nodeData.mId;
    Plane plane = nodeData.mNode->GetSplitPlane();

    TriangleList triangles;
    nodeData.mNode->GetTriangles(triangles);
    std::sort(triangles.begin(), triangles.end());

    fprintf(mFile, "%d [label = \"%d | {Plane = %s", id, id, PrintVector4(plane.mData).c_str());
    for(size_t triIndex = 0; triIndex < triangles.size(); ++triIndex)
      fprintf(mFile, " | %s", PrintTriangle(triangles[triIndex]).c_str());
    fprintf(mFile, "}\"]\n");
  }
}

void BspTreePrinter::PrintGraphVizNodeConnections()
{
  for(size_t i = 0; i < mNodeData.size(); ++i)
  {
    NodeData& nodeData = mNodeData[i];

    PrintNodeConnection(nodeData.mId, nodeData.mFrontId, false);
    PrintNodeConnection(nodeData.mId, nodeData.mBackId, false);
  }
}

void BspTreePrinter::PrintGraphViz(bool verbose)
{
  PrintGraphVizHeader();
  PrintGraphVizNodeDescriptions(!verbose);
  PrintGraphVizNodeConnections();
  PrintGraphVizFooter();
}


void BspTreePrinter::PrintEmbedded(bool verbose)
{
  PrintEmbedded(verbose, mNodeData[0].mId, "  ", true);
}

void BspTreePrinter::PrintEmbedded(bool verbose, int nodeId, std::string indent, bool last)
{
  fprintf(mFile, "%s", indent.c_str());
  if(last)
  {
    fprintf(mFile, "\\-");
    indent += "  ";
  }
  else
  {
    fprintf(mFile, "|-");
    indent += "| ";
  }

  if(mIdToIndexMap.find(nodeId) == mIdToIndexMap.end())
  {
    fprintf(mFile, "null\n");
    return;
  }

  int index = mIdToIndexMap[nodeId];
  NodeData& nodeData = mNodeData[index];

  if(!verbose)
    fprintf(mFile, "(%d)\n", nodeData.mId);
  else
  {
    Plane splitPlane = nodeData.mNode->GetSplitPlane();
    fprintf(mFile, "(%d): %s\n", nodeData.mId, PrintPlane(splitPlane).c_str());
    TriangleList triangles;
    nodeData.mNode->GetTriangles(triangles);
    std::sort(triangles.begin(), triangles.end());
    for(size_t triId = 0; triId < triangles.size(); ++triId)
    {
      fprintf(mFile, "%s", indent.c_str());
      fprintf(mFile, "|   ");
      fprintf(mFile, "%s\n", PrintTriangle(triangles[triId]).c_str());
    }
  }

  PrintEmbedded(verbose, nodeData.mFrontId, indent, false);
  PrintEmbedded(verbose, nodeData.mBackId, indent, true);
}

void BspTreePrinter::PrintTriangleList(TriangleList& triangles)
{
  for(size_t triId = 0; triId < triangles.size(); ++triId)
    fprintf(mFile, "    Depth(%d) %s\n", 0, PrintTriangle(triangles[triId]).c_str());
}
