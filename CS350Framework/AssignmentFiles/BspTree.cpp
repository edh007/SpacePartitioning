///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//--------------------------------------------------------------------BspTreeNode
BspTreeNode* BspTreeNode::GetFrontChild() const
{
    /******Student:Assignment4******/
    return pFront;
}

BspTreeNode* BspTreeNode::GetBackChild() const
{
    /******Student:Assignment4******/
    return pBack;
}

Plane BspTreeNode::GetSplitPlane() const
{
    /******Student:Assignment4******/
    return plane;
}

void BspTreeNode::GetTriangles(TriangleList& triangles) const
{
    /******Student:Assignment4******/
    triangles = triangleList;
}


//--------------------------------------------------------------------BspTree
BspTree::BspTree()
{
}

BspTree::~BspTree()
{
    RemoveNode(mTree);
}

void BspTree::RemoveNode(BspTreeNode*& target)
{
    if (target != nullptr)
    {
        RemoveNode(target->pBack);
        RemoveNode(target->pFront);

        delete target;
        target = nullptr;
    }
}

/*
Triangle splitting must be performed with the robust method described in class
(the table with 9 states). That is, you must properly handle going between all permutations of Inside, Outside, and Coplanar points.
Note that I do not want you to try and handle the edge clipping order (A to B vs. B to A) for this assignment.
You should only attempt to use the table if the triangle is overlapping the split plane,
if it is classified as either being in front or behind the plane then just
insert it into the corresponding array.
If the triangle is coplanar then use the normal of the
triangle to determine if it belongs to coplanar back or front (front if the normal points the same
direction as the split plane). Also, when the result from a split is a quad, turn the quad into two
triangles of the indices [0, 1, 2] and [0, 2, 3].
 */
void BspTree::SplitTriangle(const Plane& plane, const Triangle& tri, TriangleList& coplanarFront, TriangleList& coplanarBack, TriangleList& front, TriangleList& back, float epsilon)
{
    /******Student:Assignment4******/
    IntersectionType::Type type = PlaneTriangle(plane.mData, tri.mPoints[0], tri.mPoints[1], tri.mPoints[2], epsilon);
    if (type == IntersectionType::Inside)
    {
        front.push_back(tri);
    }
    else if (type == IntersectionType::Outside)
    {
        back.push_back(tri);
    }
    else
    {
        IntersectionType::Type p[3];
        p[0] = PointPlane(tri.mPoints[0], plane.mData, epsilon);
        p[1] = PointPlane(tri.mPoints[1], plane.mData, epsilon);
        p[2] = PointPlane(tri.mPoints[2], plane.mData, epsilon);

        if (p[0] == IntersectionType::Coplanar && p[1] == IntersectionType::Coplanar && p[2] == IntersectionType::Coplanar)
        {
            Vector3 cross = Math::Cross(tri.mPoints[1] - tri.mPoints[0], tri.mPoints[2] - tri.mPoints[0]);
            if (Math::Dot(plane.GetNormal(), cross) > 0)
                coplanarFront.push_back(Triangle(tri.mPoints[0], tri.mPoints[1], tri.mPoints[2]));
            else
                coplanarBack.push_back(Triangle(tri.mPoints[0], tri.mPoints[1], tri.mPoints[2]));
        }

        //Straddling
        else
        {
            std::vector<Vector3> f, b;
            for (int A = 0; A < 3; ++A)
            {
                int B = (A + 1) % 3;

                if (p[A] == IntersectionType::Inside)
                {
                    if (p[B] == IntersectionType::Inside)
                    {
                        AddVertex(f, tri.mPoints[B]);
                    }
                    else if (p[B] == IntersectionType::Outside)
                    {
                        Vector3 I = GetIntersection(tri.mPoints[A], tri.mPoints[B], plane, epsilon);
                        AddVertex(f, I);
                        AddVertex(b, I);
                        AddVertex(b, tri.mPoints[B]);
                    }
                    else
                    {
                        AddVertex(f, tri.mPoints[B]);
                        AddVertex(b, tri.mPoints[B]);
                    }
                }
                else if (p[A] == IntersectionType::Outside)
                {
                    if (p[B] == IntersectionType::Inside)
                    {
                        Vector3 I = GetIntersection(tri.mPoints[A], tri.mPoints[B], plane, epsilon);
                        AddVertex(f, I);
                        AddVertex(f, tri.mPoints[B]);
                        AddVertex(b, I);
                    }
                    else if (p[B] == IntersectionType::Outside)
                    {
                        AddVertex(b, tri.mPoints[B]);
                    }
                    else
                    {
                        AddVertex(f, tri.mPoints[B]);
                        AddVertex(b, tri.mPoints[B]);
                    }
                }
                else
                {
                    if (p[B] == IntersectionType::Outside)
                    {
                        AddVertex(b, tri.mPoints[A]);
                        AddVertex(b, tri.mPoints[B]);
                    }
                    else
                    {
                        AddVertex(f, tri.mPoints[B]);
                    }

                }
            }
            for (unsigned i = 0; i < f.size() - 2; ++i)
                front.push_back(Triangle(f[0], f[i + 1], f[i + 2]));
            for (unsigned i = 0; i < b.size() - 2; ++i)
                back.push_back(Triangle(b[0], b[i + 1], b[i + 2]));

        }
    }
}

void BspTree::AddVertex(std::vector<Vector3>& result, Vector3 target)
{
    for (unsigned i = 0; i < result.size(); ++i)
    {
        if (result[i] == target)
            return;
    }
    result.push_back(target);
}

Vector3 BspTree::GetIntersection(Vector3 start, Vector3 end, Plane plane, float epsilon)
{
    float t;
    RayPlane(start, (end - start).Normalized(), plane.mData, t, epsilon);
    return start + t * ((end - start).Normalized());
}

float BspTree::CalculateScore(const TriangleList& triangles, size_t testIndex, float k, float epsilon)
{
    /******Student:Assignment4******/
    Vector3 p0 = triangles[testIndex].mPoints[0];
    Vector3 p1 = triangles[testIndex].mPoints[1];
    Vector3 p2 = triangles[testIndex].mPoints[2];

    Vector3 c0 = p1 - p0;
    Vector3 c1 = p2 - p0;

    if (c0.Cross(c1).Length() == 0)
        return Math::PositiveMax();

    Plane plane(triangles[testIndex]);

    int Nf = 0, Nb = 0, Ns = 0;
    for (unsigned i = 0; i < triangles.size(); ++i)
    {
        if (i != testIndex)
        {
            IntersectionType::Type type = PlaneTriangle(plane.mData, triangles[i].mPoints[0], triangles[i].mPoints[1], triangles[i].mPoints[2], epsilon);
            if (type == IntersectionType::Inside)
                Nf++;
            else if (type == IntersectionType::Outside)
                Nb++;
            else if (type == IntersectionType::Overlaps)
                Ns++;
        }
    }

    return k * Ns + (1 - k) * Math::Abs(Nf - Nb);
}

/*
Simply choose the triangle that produces the lowest score.
*/
size_t BspTree::PickSplitPlane(const TriangleList& triangles, float k, float epsilon)
{
    /******Student:Assignment4******/
    size_t index = 0;
    float score = Math::PositiveMax();
    for (unsigned i = 0; i < triangles.size(); ++i)
    {
        float currentScore = CalculateScore(triangles, i, k, epsilon);

        if (currentScore < score)
        {
            score = currentScore;
            index = i;
        }
    }
    return index;
}

BspTreeNode* BspTree::Construction(const TriangleList& triangles, float k, float epsilon)
{
    if (triangles.size() < 1)
        return nullptr;

    BspTreeNode* newNode = new BspTreeNode;
    unsigned index = PickSplitPlane(triangles, k, epsilon);
    newNode->plane = Plane(triangles[index]);
    newNode->triangleList.push_back(triangles[index]);
    
    TriangleList front, back, coplanarFront, coplanarBack;
    for (unsigned i = 0; i < triangles.size(); ++i)
    {
        if (i != index)
            SplitTriangle(newNode->plane, triangles[i], coplanarFront, coplanarBack, front, back, epsilon);
    }

    newNode->pFront = Construction(front, k, epsilon);
    newNode->pBack = Construction(back, k, epsilon);

    for (unsigned i = 0; i < coplanarFront.size(); ++i)
        newNode->triangleList.push_back(coplanarFront[i]);

    for (unsigned i = 0; i < coplanarBack.size(); ++i)
        newNode->triangleList.push_back(coplanarBack[i]);

    return newNode;
}

/*
Recursively build the tree by splitting the data set with the best scoring triangle’s plane.
Recursion should stop when a there is only 1 triangle remaining. Also note that this is a Node-
Storing tree, that is all coplanar triangles should be stored in the node.
Make sure to not pick a split-plane that is almost the zero vector. If you fail to do
this you can end up with every triangle being coplanar. For this assignment, if the normal’s
length is below the construction epsilon then you should not choose that plane.
 */
void BspTree::Construct(const TriangleList& triangles, float k, float epsilon)
{
    /******Student:Assignment4******/
    mTree = Construction(triangles, k, epsilon);
}

/*
This function should return the root node of your tree.Additionally you should fill out the
provided helper functions on the node class.This is used for me to walk and print the structure of
your tree.
*/
BspTreeNode* BspTree::GetRoot() const
{
    /******Student:Assignment4******/
    return mTree;
}

/*
Optimized ray-casting should be implemented as described in class (using tMin and tMax).
Do not implement the “basic traversal?but rather the 4 main cases with the 3 edge cases.
Make sure that you only check the geometry in the plane when the ray hits the thick plane otherwise you will end up with more PlaneTriangle tests.
Use planeThicknessEpsilon to classify the ray’s start.
Use triExpansionEpsilon for RayTriangle, if you don’t use this then some tests will slip between two triangles.
Be careful as there are a lot of edge cases with raycasting (all should have a unit test).

Go down the side the ray is on
check triangles in plane
go down the opposite side

 */
bool BspTree::RayCast(BspTreeNode* node, const Ray& ray, float& t, float tMin, float tMax, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
    if (node != nullptr)
    {
        bool result = false;
        IntersectionType::Type type = PointPlane(ray.mStart, node->plane.mData, planeThicknessEpsilon);
        BspTreeNode* nearNode = nullptr;
        BspTreeNode* farNode = nullptr;

        if (type == IntersectionType::Inside)
        {
            nearNode = node->pFront;
            farNode = node->pBack;
        }
        else //if (Near == IntersectionType::Outside)
        {
            nearNode = node->pBack;
            farNode = node->pFront;
        }

        //Edge
        if (type == IntersectionType::Coplanar)
        {
            for (unsigned i = 0; i < node->triangleList.size(); ++i)
            {
                float tmp;
                if (RayTriangle(ray.mStart, ray.mDirection, node->triangleList[i].mPoints[0], node->triangleList[i].mPoints[1], node->triangleList[i].mPoints[2], tmp, triExpansionEpsilon))
                {
                    if (tmp < t)
                    {
                        t = tmp;
                        result = true;
                    }
                }
            }
            if (RayCast(nearNode, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex))
                result = true;
            if (RayCast(farNode, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex))
                result = true;

            return result;
        }

        float tPlane;
        //Edge
        if (RayPlane(ray.mStart, ray.mDirection, node->plane.mData, tPlane) == false)
            return RayCast(nearNode, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);

        float theta = ray.mDirection.Dot(node->plane.GetNormal()) / (ray.mDirection.Length() * node->plane.GetNormal().Length());
        float tEp = Math::Abs(planeThicknessEpsilon / theta);

        if (tMin - tEp <= tPlane && tPlane <= tMax + tEp)
        {
            if (RayCast(nearNode, ray, t, tMin, tPlane + tEp, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex))
                result = true;

            for (unsigned i = 0; i < node->triangleList.size(); ++i)
            {
                float tmp;
                if (RayTriangle(ray.mStart, ray.mDirection, node->triangleList[i].mPoints[0], node->triangleList[i].mPoints[1], node->triangleList[i].mPoints[2], tmp, triExpansionEpsilon))
                {
                    if (tmp < t)
                    {
                        t = tmp;
                        result = true;
                    }
                }
            }

            if (RayCast(farNode, ray, t, tPlane - tEp, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex))
                result = true;
            return result;
        }
        else if (tPlane < 0)
        {
            //Only traverse the near side
            return RayCast(nearNode, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
        }
        else if (tMax < tPlane)
        {
            //Only traverse the near side
            return RayCast(nearNode, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
        }
        else
        {
            //Only traverse the far side
            return RayCast(farNode, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
        }
    }
    return false;
}

bool BspTree::RayCast(const Ray& ray, float& t, float planeThicknessEpsilon, float triExpansionEpsilon, int debuggingIndex)
{
    /******Student:Assignment4******/
    t = Math::PositiveMax();

    float tMin = 0;
    float tMax = t;

    return RayCast(mTree, ray, t, tMin, tMax, planeThicknessEpsilon, triExpansionEpsilon, debuggingIndex);
}

void BspTree::AddTriangle(BspTreeNode* node, TriangleList& triangles) const
{
    if (node != nullptr)
    {
        for (unsigned i = 0; i < node->triangleList.size(); ++i)
            triangles.push_back(node->triangleList[i]);

        AddTriangle(node->pFront, triangles);
        AddTriangle(node->pBack, triangles);
    }
}

/*
AllTriangles:
Collect all triangles of in the tree into one list in a pre-order traversal (node then front then back).
 */
void BspTree::AllTriangles(TriangleList& triangles) const
{
    /******Student:Assignment4******/
    AddTriangle(mTree, triangles);
}

void BspTree::InvertingNode(BspTreeNode* node)
{
    if (node != nullptr)
    {
        InvertingNode(node->pBack);
        InvertingNode(node->pFront);

        node->plane.mData *= -1;
        for (unsigned i = 0; i < node->triangleList.size(); ++i)
            Math::Swap(node->triangleList[i].mPoints[0], node->triangleList[i].mPoints[1]);
        Math::Swap(node->pFront, node->pBack);
    }
}

/*
Invert:
When inverting the split plane make sure to properly invert the entire plane (not just the normal).
1. Flip a split plane. This is just multiplying the plane by -1.
2. Flip any contained geometry. With a triangle this amounts to just swapping two points (effectively flipping the sign of the triangle¡¯s normal).
3. Swap the front and back-side pointers. When the tree is inverted what was the positive sub-tree becomes the negative sub-tree.
 */
void BspTree::Invert()
{
    /******Student:Assignment4******/
    InvertingNode(mTree);
}

void BspTree::ClipTriangle(BspTreeNode* const& clipNode, const Triangle& triangle, TriangleList& result, float epsilon)
{
    if (clipNode != nullptr)
    {
        TriangleList front, back;
        SplitTriangle(clipNode->plane, triangle, front, back, front, back, epsilon);

        if (clipNode->pFront == nullptr)
        {
            for (unsigned i = 0; i < front.size(); ++i)
                result.push_back(front[i]);
        }

        for (unsigned i = 0; i < front.size(); ++i)
            ClipTriangle(clipNode->pFront, front[i], result, epsilon);

        for (unsigned i = 0; i < back.size(); ++i)
            ClipTriangle(clipNode->pBack, back[i], result, epsilon);
    }
}

void BspTree::ClipTo(BspTree* tree, BspTreeNode* node, float epsilon)
{
    if (node != nullptr)
    {
        TriangleList list;
        for (unsigned i = 0; i < node->triangleList.size(); ++i)
            ClipTriangle(tree->mTree, node->triangleList[i], list, epsilon);
        node->triangleList = list;

        ClipTo(tree, node->pFront, epsilon);
        ClipTo(tree, node->pBack, epsilon);
    }
}

/*
ClipTo:
The given epsilon should be used when clipping triangles.
For ClipTo make sure that you cull any triangles that end up in a solid leaf node,
that is any triangles that end up on the back side of a leaf node’s split plane.
Coplanar front triangles should be included with front triangles and coplanar back with back.
 */
void BspTree::ClipTo(BspTree* tree, float epsilon)
{
    /******Student:Assignment4******/
    ClipTo(tree, mTree, epsilon);
}

/*
The CSG operations of union, Intersection, and subtraction should be performed using the 3 functions described in class:
    AllTriangles, Invert, and ClipTo.
 */
void BspTree::Union(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    ClipTo(tree, epsilon);
    tree->ClipTo(this, epsilon);
    
    // Remove coplanar faces
    tree->Invert();
    tree->ClipTo(this, epsilon);
    tree->Invert();
    
    TriangleList list;
    AllTriangles(list);
    tree->AllTriangles(list);
    Construct(list, k, epsilon);
}

void BspTree::Intersection(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    tree->Invert();
    Invert();
    Union(tree, k, epsilon);
    Invert();
}

void BspTree::Subtract(BspTree* tree, float k, float epsilon)
{
    /******Student:Assignment4******/
    tree->Invert();
    Intersection(tree, k, epsilon);
}


/*
Draw the nodes in the tree at the given level (level of -1 means draw everything).
You should use the color and bitMask variable passed in to set the corresponding values on the DebugShape.
For color just call DebugShape.Color(color) on the shape returned from a draw function.
For the bitMask just call DebugShape.SetMaskBit(bitMask).
The bitMask is meant to help you by toggling certain debug shapes at runtime.
Hitting any key of 0-9 will toggle drawing of the any shape with those masked bits set (used in CSG operations).
 */
void BspTree::DebugDraw(int level, const Vector4& color, int bitMask)
{
    /******Student:Assignment4******/
    DebugDraw(mTree, level, 0, color, bitMask);
}

void BspTree::DebugDraw(BspTreeNode* node, int level, int currentLevel, const Vector4& color, int bitMask) const
{
    if (node != nullptr)
    {
        if (currentLevel == level || level == -1)
        {
            for (unsigned i = 0; i < node->triangleList.size(); ++i)
            {
                DebugShape &debug = node->triangleList[i].DebugDraw();
                debug.Color(color);
                debug.SetMaskBit(bitMask);
            }
        }
        DebugDraw(node->pBack, level, currentLevel + 1, color, bitMask);
        DebugDraw(node->pFront, level, currentLevel + 1, color, bitMask);
    }
}