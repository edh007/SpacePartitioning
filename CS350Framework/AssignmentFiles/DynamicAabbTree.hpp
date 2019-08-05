///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "SpatialPartition.hpp"
#include "Shapes.hpp"

//--------------------------------------------------------------------DynamicAabbTreeNode
class DynamicAabbTreeNode
{
public:
    DynamicAabbTreeNode * GetParent() const;
    DynamicAabbTreeNode* GetLeftChild() const;
    DynamicAabbTreeNode* GetRightChild() const;
    Aabb GetAabb() const;
    void* GetClientData() const;
    int GetHeight() const;

    // Add you implementation here
    Aabb mAabb;
    void* mClientData;
    DynamicAabbTreeNode* mLeft;
    DynamicAabbTreeNode* mRight;
    DynamicAabbTreeNode* mParent;
    int mHeight;
    size_t mLastAxis;
};

//--------------------------------------------------------------------DynamicAabbTree
/******Student:Assignment3******/
/// You must implement a dynamic aabb tree as we discussed in class.
class DynamicAabbTree : public SpatialPartition
{
public:

    DynamicAabbTree();
    ~DynamicAabbTree();

    // Spatial Partition Interface
    void InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data) override;
    void UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data) override;
    void RemoveData(SpatialPartitionKey& key) override;

    void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color = Vector4(1), int bitMask = 0) override;

    void CastRay(const Ray& ray, CastResults& results) override;
    void CastFrustum(const Frustum& frustum, CastResults& results) override;

    void SelfQuery(QueryResults& results) override;

    DynamicAabbTreeNode* GetRoot() const;

    // A fattening factor to use for insertion to prevent jitter from causing updates
    static const float mFatteningFactor;

    // Add your implementation here
    DynamicAabbTreeNode* mTree;
    
    void Balancing(DynamicAabbTreeNode*& currentNode);
    DynamicAabbTreeNode* CreateNode();
    DynamicAabbTreeNode* CreateNode(SpatialPartitionKey& key, const SpatialPartitionData& data);
    void Insert(DynamicAabbTreeNode*& newNode, DynamicAabbTreeNode*& currentNode);
    void DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask, DynamicAabbTreeNode* currentNode, int currentLevel);
    void CastRay(const Ray& ray, CastResults& results, DynamicAabbTreeNode* currentNode);
    void CastFrustum(const Frustum& frustum, CastResults& results, DynamicAabbTreeNode* currentNode);
    void CastFrustumInside(CastResults& results, DynamicAabbTreeNode* currentNode);
    void SplitNodes(QueryResults& results, DynamicAabbTreeNode* lhs, DynamicAabbTreeNode* rhs);
    void TreeQuery(QueryResults& results, DynamicAabbTreeNode* lhs, DynamicAabbTreeNode* rhs);
    void TreeQuery(QueryResults& results, DynamicAabbTreeNode* node);
    //Helper Functions
    //Calculate all aabbs going down 
    Aabb CalculateAabbDown(DynamicAabbTreeNode*& currentNode)
    {
        if (currentNode->mHeight == 0)
            return currentNode->mAabb;

        currentNode->mAabb = Aabb::Combine(CalculateAabbDown(currentNode->mLeft), CalculateAabbDown(currentNode->mRight));
        return currentNode->mAabb;
    }

    //Calculate all aabbs going down and going up
    void CalculateAabbDownUp(DynamicAabbTreeNode*& currentNode)
    {
        CalculateAabbDown(currentNode);
        while (currentNode->mParent)
        {
            currentNode = currentNode->mParent;
            currentNode->mAabb = Aabb::Combine(currentNode->mLeft->mAabb, currentNode->mRight->mAabb);
        }
    }

    DynamicAabbTreeNode*& SelectNode(DynamicAabbTreeNode*& newNode, DynamicAabbTreeNode*& node0, DynamicAabbTreeNode*& node1)
    {
        const float left = Aabb::Combine(newNode->mAabb, node0->mAabb).GetSurfaceArea() - node0->mAabb.GetSurfaceArea();
        const float right = Aabb::Combine(newNode->mAabb, node1->mAabb).GetSurfaceArea() - node1->mAabb.GetSurfaceArea();

        if (left < right)
            return node0;
        else
            return node1;
    }

    int CalculateHeight(DynamicAabbTreeNode* const& left, DynamicAabbTreeNode* const& right)
    {
        if (left == false && right == false)
            return 0;
        return Math::Max(left->mHeight, right->mHeight) + 1;
    }

    Aabb FattenAabb(const Aabb& origin) const
    {
        return Aabb::BuildFromCenterAndHalfExtents(origin.GetCenter(), origin.GetHalfSize() * mFatteningFactor);
    }
    void RemoveNode(DynamicAabbTreeNode*& target);
};
