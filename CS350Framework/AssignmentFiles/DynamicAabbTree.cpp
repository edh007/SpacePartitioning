///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//--------------------------------------------------------------------DynamicAabbTreeNode
DynamicAabbTreeNode* DynamicAabbTreeNode::GetParent() const
{
    /******Student:Assignment3******/
    return mParent;
}

DynamicAabbTreeNode* DynamicAabbTreeNode::GetLeftChild() const
{
    /******Student:Assignment3******/
    return mLeft;
}

DynamicAabbTreeNode* DynamicAabbTreeNode::GetRightChild() const
{
    /******Student:Assignment3******/
    return mRight;
}

Aabb DynamicAabbTreeNode::GetAabb() const
{
    /******Student:Assignment3******/
    return mAabb;
}

void* DynamicAabbTreeNode::GetClientData() const
{
    /******Student:Assignment3******/
    return mClientData;
}

int DynamicAabbTreeNode::GetHeight() const
{
    /******Student:Assignment3******/
    return mHeight;
}

//--------------------------------------------------------------------DynamicAabbTree
const float DynamicAabbTree::mFatteningFactor = 1.1f;

DynamicAabbTree::DynamicAabbTree()
{
    mType = SpatialPartitionTypes::AabbTree;
    mTree = nullptr;
}

DynamicAabbTree::~DynamicAabbTree()
{
    RemoveNode(mTree);
}

void DynamicAabbTree::RemoveNode(DynamicAabbTreeNode*& target)
{
    if (target == nullptr)
        return;

    RemoveNode(target->mLeft);
    RemoveNode(target->mRight);

    delete target;
    target = nullptr;
}

DynamicAabbTreeNode* DynamicAabbTree::CreateNode()
{
    DynamicAabbTreeNode* newNode = new DynamicAabbTreeNode();
    return newNode;
}

DynamicAabbTreeNode* DynamicAabbTree::CreateNode(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    DynamicAabbTreeNode* newNode = new DynamicAabbTreeNode();
    newNode->mAabb = FattenAabb(data.mAabb);
    newNode->mClientData = data.mClientData;
    newNode->mLastAxis = 0;
    key.mVoidKey = newNode;
    return newNode;
}

void DynamicAabbTree::Insert(DynamicAabbTreeNode*& newNode, DynamicAabbTreeNode*& currentNode)
{
    //case 1: 
    if (currentNode == nullptr)
    {
        newNode->mParent = currentNode;
        currentNode = newNode;
        currentNode->mHeight = 0;
    }
    //case 2: 
    else if (currentNode->mLeft == nullptr && currentNode->mRight == nullptr)
    {
        DynamicAabbTreeNode* temp = CreateNode();

        temp->mLeft = currentNode;
        temp->mRight = newNode;
        temp->mParent = currentNode->mParent;
        currentNode = temp;
        currentNode->mHeight = 1;
        newNode->mParent = currentNode;
        temp->mLeft->mParent = temp;

        temp->mAabb = Aabb::Combine(temp->mLeft->mAabb, temp->mRight->mAabb);
    }
    else
        Insert(newNode, SelectNode(newNode, currentNode->mLeft, currentNode->mRight));
}

void DynamicAabbTree::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    /******Student:Assignment3******/
    DynamicAabbTreeNode* newNode = CreateNode(key, data);
    Insert(newNode, mTree);
    Balancing(newNode);
    CalculateAabbDownUp(mTree);
}

void DynamicAabbTree::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    /******Student:Assignment3******/
    DynamicAabbTreeNode* currentNode = (DynamicAabbTreeNode*)key.mVoidKey;
    Aabb combine = Aabb::Combine(currentNode->mAabb, data.mAabb);
    
    if ((combine.GetMin() == currentNode->mAabb.GetMin() && combine.GetMax() == currentNode->mAabb.GetMax()) == false)
    {
        RemoveData(key);
        InsertData(key, data);
    }
}

void DynamicAabbTree::RemoveData(SpatialPartitionKey& key)
{
    /******Student:Assignment3******/
    auto target = (DynamicAabbTreeNode*)key.mVoidKey;
    auto parent = target->mParent;

    // check the parent
    if (parent == nullptr)
    {
        delete mTree;
        mTree = nullptr;
    }
    // check the grand parent
    else if(parent->mParent == nullptr)
    {
        DynamicAabbTreeNode* tmp;
        if (parent->mLeft == target)
            tmp = parent->mRight;
        else
            tmp = parent->mLeft;
        delete target;
        delete parent;
        mTree = tmp;
        mTree->mParent = nullptr;
    }
    // normal case which has parent & grandparent
    else
    {
        bool dir;
        if (parent->mLeft == target)
            dir = true;
        else
            dir = false;
        delete target;
        target = parent;

        DynamicAabbTreeNode* grandParent = parent->mParent;
        if (grandParent->mLeft == target)
        {
            if (dir == true)
                grandParent->mLeft = parent->mRight;
            else
                grandParent->mLeft = parent->mLeft;
            grandParent->mLeft->mParent = grandParent;
        }
        else
        {
            if (dir == true)
                grandParent->mRight = parent->mRight;
            else
                grandParent->mRight = parent->mLeft;
            grandParent->mRight->mParent = grandParent;
        }
        delete target;

        Balancing(grandParent);
        CalculateAabbDown(mTree);
    }
}


void DynamicAabbTree::Balancing(DynamicAabbTreeNode*& currentNode)
{
    if (currentNode == nullptr)
        return;

    //check upper when it is leaf
    if (currentNode->mLeft == nullptr || currentNode->mRight == nullptr)
    {
        Balancing(currentNode->mParent);
        return;
    }

    //check the difference
    currentNode->mHeight = CalculateHeight(currentNode->mLeft, currentNode->mRight);
    int diff = currentNode->mLeft->mHeight - currentNode->mRight->mHeight;

    if (diff == 0 || diff == 1 || diff == -1)
        Balancing(currentNode->mParent);
    else
    {
        // 1. Identify pivot, small child, and large child
        DynamicAabbTreeNode* pivot;
        if (currentNode->mLeft->mHeight >= currentNode->mRight->mHeight)
            pivot = currentNode->mLeft;
        else
            pivot = currentNode->mRight;

        DynamicAabbTreeNode* small;
        if (pivot->mLeft->mHeight >= pivot->mRight->mHeight)
            small = pivot->mRight;
        else
            small = pivot->mLeft;

        // 2. Detach small child, pivot, and old parent
        if (pivot->mLeft == small)
            pivot->mLeft = nullptr;
        else
            pivot->mRight = nullptr;

        if (currentNode->mLeft == pivot)
            currentNode->mLeft = nullptr;
        else
            currentNode->mRight = nullptr;

        // 3. Replace the grand-parent link of the old parent to the pivot
        DynamicAabbTreeNode* parent = currentNode;
        if (parent->mParent != nullptr)
        {
            if (parent->mParent->mLeft == parent)
                parent->mParent->mLeft = pivot;
            else
                parent->mParent->mRight = pivot;
        }
        else
            mTree = pivot;

        // 4. Insert old parent as new small-child
        if (pivot->mLeft == nullptr)
            pivot->mLeft = parent;
        else
            pivot->mRight = parent;

        // 5. Insert small child where the pivot node was
        if (parent->mLeft == nullptr)
            parent->mLeft = small;
        else
            parent->mRight = small;

        //update parent
        small->mParent = parent;
        pivot->mParent = parent->mParent;
        parent->mParent = pivot;

        //recalculate height / size
        parent->mHeight = CalculateHeight(parent->mLeft, parent->mRight);
        pivot->mHeight = CalculateHeight(pivot->mLeft, pivot->mRight);

        //go up tree
        Balancing(pivot->mParent);
    }
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask, DynamicAabbTreeNode* currentNode, int currentLevel)
{
    //Setting debug draw values on a temp. You need to take the return by reference 
    if(currentNode != nullptr)
    {
        if(currentLevel == level || level == -1)
        {
            DebugShape& tmp = currentNode->mAabb.DebugDraw();
            tmp.SetTransform(transform);
            tmp.Color(color);
            tmp.SetMaskBit(bitMask);
        }
        DebugDraw(level, transform, color, bitMask, currentNode->mLeft, currentLevel + 1);
        DebugDraw(level, transform, color, bitMask, currentNode->mRight, currentLevel + 1);
    }
}

void DynamicAabbTree::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
    /******Student:Assignment3******/
    DebugDraw(level, transform, color, bitMask, mTree, 0);
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results, DynamicAabbTreeNode* currentNode)
{
    if (currentNode != nullptr)
    {
        float t;
        bool r = RayAabb(ray.mStart, ray.mDirection, currentNode->mAabb.GetMin(), currentNode->mAabb.GetMax(), t);
        if (r == true)
        {
            if (currentNode->mHeight == false)
                results.AddResult(CastResult(currentNode->mClientData, t));
            else
            {
                CastRay(ray, results, currentNode->mLeft);
                CastRay(ray, results, currentNode->mRight);
            }
        }
    }
}

void DynamicAabbTree::CastRay(const Ray& ray, CastResults& results)
{
    /******Student:Assignment3******/
    CastRay(ray, results, mTree);
}

void DynamicAabbTree::CastFrustumInside(CastResults& results, DynamicAabbTreeNode* currentNode)
{
    if(currentNode != nullptr)
    {
        if (currentNode->mHeight == 0)
            results.AddResult(CastResult(currentNode->mClientData));

        CastFrustumInside(results, currentNode->mLeft);
        CastFrustumInside(results, currentNode->mRight);
    }
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results, DynamicAabbTreeNode* currentNode)
{
    if (currentNode != nullptr)
    {
        auto result = FrustumAabb(frustum.GetPlanes(), currentNode->mAabb.GetMin(), currentNode->mAabb.GetMax(), currentNode->mLastAxis);
        if (result == IntersectionType::Overlaps)
        {
            if (currentNode->mHeight == 0)
                results.AddResult(CastResult(currentNode->mClientData));
            else
            {
                CastFrustum(frustum, results, currentNode->mLeft);
                CastFrustum(frustum, results, currentNode->mRight);
            }
        }
        else if(result == IntersectionType::Inside)
            CastFrustumInside(results, currentNode);
    }
}

void DynamicAabbTree::CastFrustum(const Frustum& frustum, CastResults& results)
{
    /******Student:Assignment3******/
    CastFrustum(frustum, results, mTree);
}

void DynamicAabbTree::TreeQuery(QueryResults& results, DynamicAabbTreeNode* node)
{
    if (node == nullptr)
        return;

    if (node->mHeight != 0)
    {
        TreeQuery(results, node->mLeft, node->mRight);
        TreeQuery(results, node->mLeft);
        TreeQuery(results, node->mRight);
    }
}

void DynamicAabbTree::TreeQuery(QueryResults& results, DynamicAabbTreeNode* lhs, DynamicAabbTreeNode* rhs)
{
    if (lhs == nullptr || rhs == nullptr)
        return;

    if (AabbAabb(lhs->mAabb.GetMin(), lhs->mAabb.GetMax(), rhs->mAabb.GetMin(), rhs->mAabb.GetMax()) == true)
    {
        // 1. Both are leaf nodes : The only logical thing to do with two leaf nodes is add them as result pairs
        if (lhs->mHeight == 0 && rhs->mHeight == 0)
            results.AddResult(QueryResult(lhs->mClientData, rhs->mClientData));

        // 2. One is leaf and one is internal : add a result pair of leaf nodes so we have to recurse. 
        else if (lhs->mHeight == 0)
        {
            TreeQuery(results, lhs, rhs->mLeft);
            TreeQuery(results, lhs, rhs->mRight);
        }
        else if (rhs->mHeight == 0)
        {
            TreeQuery(results, rhs, lhs->mLeft);
            TreeQuery(results, rhs, lhs->mRight);
        }
        // 3. Both are internal : split the nodes
        else
            SplitNodes(results, lhs, rhs);
    }
}

void DynamicAabbTree::SplitNodes(QueryResults& results, DynamicAabbTreeNode* lhs, DynamicAabbTreeNode* rhs)
{
    if (lhs->mAabb.GetVolume() > rhs->mAabb.GetVolume())
    {
        TreeQuery(results, lhs->mLeft, rhs);
        TreeQuery(results, lhs->mRight, rhs);

    }
    else
    {
        TreeQuery(results, lhs, rhs->mLeft);
        TreeQuery(results, lhs, rhs->mRight);
    }
}

void DynamicAabbTree::SelfQuery(QueryResults& results)
{
    /******Student:Assignment3******/
    TreeQuery(results, mTree);
}

DynamicAabbTreeNode* DynamicAabbTree::GetRoot() const
{
    /******Student:Assignment3******/
    return mTree;
}

