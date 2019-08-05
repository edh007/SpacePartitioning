///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Dongho Lee
/// Copyright 2018, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

/******Student:Assignment1******/
bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b,
    float& u, float& v, float expansionEpsilon)
{
    float denominator = ((a - b).Dot(a - b));
    if(denominator == 0)
    {
        u = v = 0;
        return false;
    }
    u = ((point - b).Dot(a - b)) / denominator;
    v = 1 - u;

    if (!(-expansionEpsilon <= u && u <= 1 + expansionEpsilon))
    {
        return false;
    }
    return true;
}

/******Student:Assignment1******/
bool BarycentricCoordinates(const Vector3& point, const Vector3& a, const Vector3& b, const Vector3& c,
    float& u, float& v, float& w, float expansionEpsilon)
{
    Vector3 nPBC = (b - point).Cross(c - point);
    Vector3 nABC = (b - a).Cross(c - a);
    float denominator = nABC.Dot(nABC);
    if(denominator == 0)
    {
        u = v = w = 0;
        return false;
    }
    u = nPBC.Dot(nABC) / denominator;

    Vector3 nPCA = (c - point).Cross(a - point);
    v = nPCA.Dot(nABC) / denominator;

    w = 1 - u - v;

    if (!(-expansionEpsilon <= u && u <= 1 + expansionEpsilon) || !(-expansionEpsilon <= v && v <= 1 + expansionEpsilon) || !(-expansionEpsilon <= w && w <= 1 + expansionEpsilon))
    {
        return false;
    }
    return true;
}

/******Student:Assignment1******/
IntersectionType::Type PointPlane(const Vector3& point, const Vector4& plane, float epsilon)
{
    Vector4 p(point.x, point.y, point.z, -1);
    float w = plane.Dot(p);

    if (w < -epsilon)
        return IntersectionType::Outside;
    else if (-epsilon <= w && w <= epsilon)
        return IntersectionType::Coplanar;
    else //w > epsilon
        return IntersectionType::Inside;
}

/******Student:Assignment1******/
bool PointSphere(const Vector3& point, const Vector3& sphereCenter, float sphereRadius)
{
    float result = (sphereCenter - point).LengthSq() - sphereRadius * sphereRadius;
    if (result <= 0)
        return true;
    else
        return false;
}

/******Student:Assignment1******/
bool PointAabb(const Vector3& point, const Vector3& aabbMin, const Vector3& aabbMax)
{
    bool result = true;
    if (!(aabbMin.x <= point.x && point.x <= aabbMax.x))
        result = false;
    if (!(aabbMin.y <= point.y && point.y <= aabbMax.y))
        result = false;
    if (!(aabbMin.z <= point.z && point.z <= aabbMax.z))
        result = false;
    return result;
}

/******Student:Assignment1******/
bool RayPlane(const Vector3& rayStart, const Vector3& rayDir,
    const Vector4& plane, float& t, float parallelCheckEpsilon)
{
    ++Application::mStatistics.mRayPlaneTests;

    Vector3 n(plane.x, plane.y, plane.z);
    float denominator = rayDir.Dot(n);

    if (-parallelCheckEpsilon < denominator && denominator < parallelCheckEpsilon)
        return false;
    t = (plane.w - rayStart.Dot(n)) / denominator;

    bool result = true;
    if (t < 0)
        result = false;
    else
        result = true;

    return result;
}

/******Student:Assignment1******/
bool RayTriangle(const Vector3& rayStart, const Vector3& rayDir,
    const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
    float& t, float triExpansionEpsilon, float parallelCheckEpsilon)
{
    ++Application::mStatistics.mRayTriangleTests;

    Plane plane(triP0, triP1, triP2);
    if (RayPlane(rayStart, rayDir, plane.mData, t) == false)
        return false;

    float u, v, w;
    return BarycentricCoordinates(rayStart + rayDir * t, triP0, triP1, triP2, u, v, w, triExpansionEpsilon);
}

/******Student:Assignment1******/
bool RaySphere(const Vector3& rayStart, const Vector3& rayDir,
    const Vector3& sphereCenter, float sphereRadius,
    float& t)
{
    ++Application::mStatistics.mRaySphereTests;

    t = 0;
    if (PointSphere(rayStart, sphereCenter, sphereRadius))
        return true;

    Vector3 m = (sphereCenter - rayStart);
    float a = rayDir.Dot(rayDir);
    float b = -2 * rayDir.Dot(m);
    float det = b * b - 4 * a * (m.Dot(m) - sphereRadius * sphereRadius);

    if (det < 0)
        return false;

    if (det == 0)
    {
        t = -b / (2 * a);
        return true;
    }

    if ((-b + sqrt(det)) / (2 * a) < 0)
        return false;

    t = (-b - sqrt(det)) / (2 * a);
    if (t < 0)
    {
        t = 0;
        return true;
    }
    return true;
}

/******Student:Assignment1******/
bool RayAabb(const Vector3& rayStart, const Vector3& rayDir,
    const Vector3& aabbMin, const Vector3& aabbMax, float& t, float parallelCheckEpsilon)
{
    ++Application::mStatistics.mRayAabbTests;

    float min, max, yMin, yMax, zMin, zMax;

    if ((aabbMin.x <= rayStart.x && rayStart.x <= aabbMax.x)
        && (aabbMin.y <= rayStart.y && rayStart.y <= aabbMax.y)
        && (aabbMin.z <= rayStart.z && rayStart.z <= aabbMax.z))
    {
        t = 0;
        return true;
    }

    min = (aabbMin.x - rayStart.x) / rayDir.x;
    max = (aabbMax.x - rayStart.x) / rayDir.x;
    if (rayDir.x < 0)
        Math::Swap(min, max);

    yMin = (aabbMin.y - rayStart.y) / rayDir.y;
    yMax = (aabbMax.y - rayStart.y) / rayDir.y;
    if (rayDir.y < 0)
        Math::Swap(yMin, yMax);

    if ((min > yMax) || (yMin > max))
        return false;
    if (yMin > min)
        min = yMin;
    if (yMax < max)
        max = yMax;

    zMin = (aabbMin.z - rayStart.z) / rayDir.z;
    zMax = (aabbMax.z - rayStart.z) / rayDir.z;
    if (rayDir.z < 0)
        Math::Swap(zMin, zMax);

    if ((min > zMax) || (zMin > max))
        return false;
    if (zMin > min)
        min = zMin;
    if (zMax < max)
        max = zMax;

    if (min > 0)
        t = min;
    else if (max > 0)
        t = max;
    else
        return false;
    return true;
}

/******Student:Assignment1******/
IntersectionType::Type PlaneTriangle(const Vector4& plane,
    const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
    float epsilon)
{
    ++Application::mStatistics.mPlaneTriangleTests;

    IntersectionType::Type a = PointPlane(triP0, plane, epsilon);
    IntersectionType::Type b = PointPlane(triP1, plane, epsilon);
    IntersectionType::Type c = PointPlane(triP2, plane, epsilon);

    if (a == IntersectionType::Inside)
    {
        if (b == IntersectionType::Outside)
            return IntersectionType::Overlaps;
        else
        {
            if (c == IntersectionType::Outside)
                return IntersectionType::Overlaps;
            else
                return IntersectionType::Inside;
        }

    }
    else if (a == IntersectionType::Outside)
    {
        if (b == IntersectionType::Inside)
            return IntersectionType::Overlaps;
        else
        {
            if (c == IntersectionType::Inside)
                return IntersectionType::Overlaps;
            else
                return IntersectionType::Outside;
        }
    }
    else //Coplanar
    {
        if (b == IntersectionType::Inside)
        {
            if (c != IntersectionType::Outside)
                return IntersectionType::Inside;
            else
                return IntersectionType::Overlaps;
        }
        else if (b == IntersectionType::Outside)
        {
            if (c != IntersectionType::Inside)
                return IntersectionType::Outside;
            else
                return IntersectionType::Overlaps;
        }
        else
        {
            if (c == IntersectionType::Inside)
                return IntersectionType::Inside;
            else if (c == IntersectionType::Outside)
                return IntersectionType::Outside;
            else
                return IntersectionType::Coplanar;
        }
    }
}

/******Student:Assignment1******/
IntersectionType::Type PlaneSphere(const Vector4& plane,
    const Vector3& sphereCenter, float sphereRadius)
{
    ++Application::mStatistics.mPlaneSphereTests;

    IntersectionType::Type result = PointPlane(sphereCenter, plane, sphereRadius);
    if (result == IntersectionType::Coplanar)
        return IntersectionType::Overlaps;
    else
        return result;
}

/******Student:Assignment1******/
IntersectionType::Type PlaneAabb(const Vector4& plane,
    const Vector3& aabbMin, const Vector3& aabbMax)
{
    ++Application::mStatistics.mPlaneAabbTests;

    Vector3 n(plane.x, plane.y, plane.z);
    Vector3 center = (aabbMin + aabbMax) / 2;
    Vector3 e = aabbMax - center;

    float r = (e.x * abs(n.x)) + (e.y * abs(n.y)) + (e.z * abs(n.z));

    IntersectionType::Type result = PointPlane(center, plane, r);
    if (result == IntersectionType::Coplanar)
        return IntersectionType::Overlaps;
    else
        return result;
}

/******Student:Assignment1******/
IntersectionType::Type FrustumTriangle(const Vector4 planes[6],
    const Vector3& triP0, const Vector3& triP1, const Vector3& triP2,
    float epsilon)
{
    ++Application::mStatistics.mFrustumTriangleTests;

    IntersectionType::Type result[6];
    for (int i = 0; i < 6; ++i)
    {
        result[i] = PlaneTriangle(planes[i], triP0, triP1, triP2, epsilon);
        if (result[i] == IntersectionType::Outside)
            return IntersectionType::Outside;
    }

    bool sameResultChecker = true;
    for (int i = 0; i < 5; ++i)
        if (result[i] != result[i + 1])
        {
            sameResultChecker = false;
            break;
        }

    if (sameResultChecker)
        if (result[0] == IntersectionType::Inside)
            return IntersectionType::Inside;
    return IntersectionType::Overlaps;
}

/******Student:Assignment1******/
IntersectionType::Type FrustumSphere(const Vector4 planes[6],
    const Vector3& sphereCenter, float sphereRadius, size_t& lastAxis)
{
    ++Application::mStatistics.mFrustumSphereTests;

    IntersectionType::Type result[6];
    for (int i = 0; i < 6; ++i)
    {
        result[i] = PointPlane(sphereCenter, planes[i], sphereRadius);
        if (result[i] == IntersectionType::Outside)
            return IntersectionType::Outside;
    }

    bool sameResultChecker = true;
    for (int i = 0; i < 5; ++i)
        if (result[i] != result[i + 1])
        {
            sameResultChecker = false;
            break;
        }

    if (sameResultChecker)
        if (result[0] == IntersectionType::Inside)
            return IntersectionType::Inside;
    return IntersectionType::Overlaps;
}

/******Student:Assignment1******/
IntersectionType::Type FrustumAabb(const Vector4 planes[6],
    const Vector3& aabbMin, const Vector3& aabbMax, size_t& lastAxis)
{
    ++Application::mStatistics.mFrustumAabbTests;

    auto last = IntersectionType::Inside;
    for (size_t i = 0; i < 6; ++i)
    {
        size_t j = (i + lastAxis) % 6;

        auto result = PlaneAabb(planes[j], aabbMin, aabbMax);
        if (result == IntersectionType::Outside)
        {
            lastAxis = j;
            return IntersectionType::Outside;
        }

        else if (result == IntersectionType::Overlaps)
            last = IntersectionType::Overlaps;
    }
    return last;
}

/******Student:Assignment1******/
bool SphereSphere(const Vector3& sphereCenter0, float sphereRadius0,
    const Vector3& sphereCenter1, float sphereRadius1)
{
    ++Application::mStatistics.mSphereSphereTests;

    float p = (sphereCenter1 - sphereCenter0).LengthSq();
    if (p <= (sphereRadius0 + sphereRadius1)*(sphereRadius0 + sphereRadius1))
        return true;
    return false;
}

/******Student:Assignment1******/
bool AabbAabb(const Vector3& aabbMin0, const Vector3& aabbMax0,
    const Vector3& aabbMin1, const Vector3& aabbMax1)
{
    ++Application::mStatistics.mAabbAabbTests;

    if ((aabbMax0.x < aabbMin1.x) || (aabbMax1.x < aabbMin0.x) ||
        (aabbMax0.y < aabbMin1.y) || (aabbMax1.y < aabbMin0.y) ||
        (aabbMax0.z < aabbMin1.z) || (aabbMax1.z < aabbMin0.z))
        return false;
    return true;
}
