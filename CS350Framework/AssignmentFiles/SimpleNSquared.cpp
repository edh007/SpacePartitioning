///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------NSquaredSpatialPartition
NSquaredSpatialPartition::NSquaredSpatialPartition()
{
    mType = SpatialPartitionTypes::NSquared;
    mCurrentId = 0;
}

void NSquaredSpatialPartition::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    // Doing this lazily (and bad, but it's n-squared...).
    // Just store an ever incrementing id as a key along with the client data so we can look it up later.
    key.mUIntKey = mCurrentId;
    Item item;
    item.mClientData = data.mClientData;
    item.mKey = key.mUIntKey;

    mData.push_back(item);

    ++mCurrentId;
}

void NSquaredSpatialPartition::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    // Nothing to do here, there's no spatial partition data to update
}

void NSquaredSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
    // Find the key data and remove it
    for (size_t i = 0; i < mData.size(); ++i)
    {
        if (mData[i].mKey == key.mUIntKey)
        {
            mData[i] = mData.back();
            mData.pop_back();
            break;
        }
    }
}

void NSquaredSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
    // Nothing to debug draw
}

void NSquaredSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        CastResult result;
        result.mClientData = mData[i].mClientData;
        results.AddResult(result);
    }
}

void NSquaredSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        CastResult result;
        result.mClientData = mData[i].mClientData;
        results.AddResult(result);
    }
}

void NSquaredSpatialPartition::SelfQuery(QueryResults& results)
{
    // Add everything
    for (size_t i = 0; i < mData.size(); ++i)
    {
        for (size_t j = i + 1; j < mData.size(); ++j)
        {
            results.AddResult(QueryResult(mData[i].mClientData, mData[j].mClientData));
        }
    }
}

void NSquaredSpatialPartition::GetDataFromKey(const SpatialPartitionKey& key, SpatialPartitionData& data) const
{
    // Find the key data and remove it
    for (size_t i = 0; i < mData.size(); ++i)
    {
        if (mData[i].mKey == key.mUIntKey)
        {
            data.mClientData = mData[i].mClientData;
        }
    }

}

void NSquaredSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
    for (size_t i = 0; i < mData.size(); ++i)
    {
        SpatialPartitionQueryData data;
        data.mClientData = mData[i].mClientData;
        results.push_back(data);
    }
}

//-----------------------------------------------------------------------------BoundingSphereSpatialPartition
BoundingSphereSpatialPartition::BoundingSphereSpatialPartition()
{
    mType = SpatialPartitionTypes::NSquaredSphere;
}

void BoundingSphereSpatialPartition::InsertData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    /******Student:Assignment2******/
    key.mUIntKey = mCurrentId;
    mMap.insert(std::pair<unsigned, SpatialPartitionData>(mCurrentId, data));
    ++mCurrentId;
}

void BoundingSphereSpatialPartition::UpdateData(SpatialPartitionKey& key, const SpatialPartitionData& data)
{
    /******Student:Assignment2******/
    mMap[key.mUIntKey] = data;
}

void BoundingSphereSpatialPartition::RemoveData(SpatialPartitionKey& key)
{
    /******Student:Assignment2******/
    mMap.erase(key.mUIntKey);
}

void BoundingSphereSpatialPartition::DebugDraw(int level, const Math::Matrix4& transform, const Vector4& color, int bitMask)
{
    /******Student:Assignment2******/
    for (auto it = mMap.begin(); it != mMap.end(); ++it)
    {
        it->second.mBoundingSphere.DebugDraw().SetTransform(transform);
        it->second.mBoundingSphere.DebugDraw().Color(color);
        it->second.mBoundingSphere.DebugDraw().SetMaskBit(bitMask);

        it->second.mAabb.DebugDraw().SetTransform(transform);
        it->second.mAabb.DebugDraw().Color(color);
        it->second.mAabb.DebugDraw().SetMaskBit(bitMask);
    }
}

void BoundingSphereSpatialPartition::CastRay(const Ray& ray, CastResults& results)
{
    /******Student:Assignment2******/
    float t;
    for (auto it = mMap.begin(); it != mMap.end(); ++it)
    {
        bool r = RaySphere(ray.mStart, ray.mDirection, it->second.mBoundingSphere.mCenter, it->second.mBoundingSphere.mRadius, t);
        if(r == true)
        {
            CastResult tmp((it->second.mClientData), t);
            results.AddResult(tmp);
        }
    }
    
}

void BoundingSphereSpatialPartition::CastFrustum(const Frustum& frustum, CastResults& results)
{
    /******Student:Assignment2******/
    size_t lastAxis;
    for (auto it = mMap.begin(); it != mMap.end(); ++it)
    {
        IntersectionType::Type result = FrustumSphere(frustum.GetPlanes(), it->second.mBoundingSphere.mCenter, it->second.mBoundingSphere.mRadius, lastAxis);
        if (result != IntersectionType::Outside)
        {
            CastResult tmp(it->second.mClientData);
            results.AddResult(tmp);
        }
    }
}

void BoundingSphereSpatialPartition::SelfQuery(QueryResults& results)
{
    /******Student:Assignment2******/
    auto endPoint = mMap.end();
    --endPoint;
    for (auto it = mMap.begin(); it != endPoint; ++it)
    {
        auto startPoint = it;
        ++startPoint;
        for (auto it2 = startPoint; it2 != mMap.end(); ++it2)
        {
            bool r = SphereSphere(it->second.mBoundingSphere.mCenter, it->second.mBoundingSphere.mRadius, it2->second.mBoundingSphere.mCenter, it2->second.mBoundingSphere.mRadius);
            if (r == true)
            {
                QueryResult tmp(it->second.mClientData, it2->second.mClientData);
                results.AddResult(tmp);
            }
        }
    }
}

void BoundingSphereSpatialPartition::FilloutData(std::vector<SpatialPartitionQueryData>& results) const
{
    /******Student:Assignment2******/
    for (auto it = mMap.begin(); it != mMap.end(); ++it)
        results.emplace_back(it->second);
}
