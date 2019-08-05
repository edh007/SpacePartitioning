///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

//-----------------------------------------------------------------------------SupportShape
Vector3 SupportShape::GetCenter(const std::vector<Vector3>& localPoints, const Matrix4& transform) const
{
    Vector3 center = Vector3::cZero;
    /******Student:Assignment5******/
    Sphere sph;
    sph.ComputeCentroid(localPoints);
    center = sph.mCenter;
    center = Math::TransformPoint(transform, center);
    return center;
}

Vector3 SupportShape::Support(const Vector3& worldDirection, const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform) const
{
    Vector3 result = Vector3::cZero;
    /******Student:Assignment5******/
    float projValue = Math::NegativeMin();
    unsigned index = 0;
    Vector3 dir = Math::TransformDirection(localToWorldTransform.Inverted(), worldDirection);
    for (unsigned i = 0; i < localPoints.size(); ++i)
    {
        float tmp = Math::Dot(dir, localPoints[i]);
        if (tmp > projValue)
        {
            index = i;
            projValue = tmp;
        }
    }
    result = Math::TransformPoint(localToWorldTransform, localPoints[index]);
    return result;
}

void SupportShape::DebugDraw(const std::vector<Vector3>& localPoints, const Matrix4& localToWorldTransform, const Vector4& color) const
{
    /******Student:Assignment5******/
    for (unsigned i = 0; i < localPoints.size(); ++i)
    {
        DebugShape& debug = gDebugDrawer->DrawPoint(Math::TransformPoint(localToWorldTransform, localPoints[i]));
        debug.Color(color);
    }
}

//-----------------------------------------------------------------------------ModelSupportShape
Vector3 ModelSupportShape::GetCenter() const
{
    return SupportShape::GetCenter(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

Vector3 ModelSupportShape::Support(const Vector3& worldDirection) const
{
    return SupportShape::Support(worldDirection, mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

void ModelSupportShape::DebugDraw(const Vector4& color) const
{
    SupportShape::DebugDraw(mModel->mMesh->mVertices, mModel->mOwner->has(Transform)->GetTransform());
}

//-----------------------------------------------------------------------------PointsSupportShape
PointsSupportShape::PointsSupportShape()
{
    mScale = Vector3(1);
    mRotation = Matrix3::cIdentity;
    mTranslation = Vector3::cZero;
}

Vector3 PointsSupportShape::GetCenter() const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    return SupportShape::GetCenter(mLocalSpacePoints, transform);
}

Vector3 PointsSupportShape::Support(const Vector3& worldDirection) const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    return SupportShape::Support(worldDirection, mLocalSpacePoints, transform);
}

void PointsSupportShape::DebugDraw(const Vector4& color) const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    SupportShape::DebugDraw(mLocalSpacePoints, transform, color);
}

//-----------------------------------------------------------------------------SphereSupportShape
Vector3 SphereSupportShape::GetCenter() const
{
    return mSphere.mCenter;
}

Vector3 SphereSupportShape::Support(const Vector3& worldDirection) const
{
    /******Student:Assignment5******/
    return mSphere.mCenter + (worldDirection.Normalized()) * mSphere.GetRadius();
}

void SphereSupportShape::DebugDraw(const Vector4& color) const
{
    DebugShape& shape = gDebugDrawer->DrawSphere(mSphere);
    shape.Color(color);
}

//-----------------------------------------------------------------------------ObbSupportShape
Vector3 ObbSupportShape::GetCenter() const
{
    return mTranslation;
}

Vector3 ObbSupport(const Vector3& direction, const Vector3& center, const Vector3& halfExtent, const Matrix3 rotation)
{
    Vector3 result = center;
    Vector3 localDir = Math::Transform(rotation.Inverted(), direction);
    for (unsigned i = 0; i < 3; ++i)
    {
        result += Math::GetSign(localDir[i]) * halfExtent[i] * rotation.Basis(i);
    }
    return result;
}

Vector3 ObbSupportShape::Support(const Vector3& worldDirection) const
{
    /******Student:Assignment5******/
    Vector3 point = GetCenter();
    Vector3 half = mScale / 2.f;
    Matrix4 tr;
    tr.SetIdentity();
    tr.Translate(mTranslation);
    Math::TransformPoint(tr.Inverted(), point);
    point = ObbSupport(worldDirection, point, half, mRotation);
    Math::TransformPoint(tr, point);
    return point;
}

void ObbSupportShape::DebugDraw(const Vector4& color) const
{
    Matrix4 transform = Math::BuildTransform(mTranslation, mRotation, mScale);
    DebugShape& shape = gDebugDrawer->DrawAabb(Aabb(Vector3(-0.5f), Vector3(0.5f)));
    shape.Color(color);
    shape.SetTransform(transform);
}


VoronoiRegion::Type GetTriangle(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
    size_t& newSize, int newIndices[4], float u, float v, float w,
    Vector3& closestPoint, Vector3& searchDirection, VoronoiRegion::Type type)
{
    newSize = 3;
    if (type == VoronoiRegion::Triangle123)
    {
        newIndices[0] = 1;
        newIndices[1] = 2;
        newIndices[2] = 3;
        closestPoint = u * p1 + v * p2 + w * p3;
    }
    else if (type == VoronoiRegion::Triangle012)
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newIndices[2] = 2;
        closestPoint = u * p0 + v * p1 + w * p2;
    }
    else if (type == VoronoiRegion::Triangle013)
    {
        newIndices[0] = 0;
        newIndices[1] = 1;
        newIndices[2] = 3;
        closestPoint = u * p0 + v * p1 + w * p3;
    }
    else if (type == VoronoiRegion::Triangle023)
    {
        newIndices[0] = 0;
        newIndices[1] = 2;
        newIndices[2] = 3;
        closestPoint = u * p0 + v * p2 + w * p3;
    }
    searchDirection = (q - closestPoint);
    return type;
}

VoronoiRegion::Type GetEdge(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
    size_t& newSize, int newIndices[4], float u, float v,
    Vector3& closestPoint, Vector3& searchDirection, VoronoiRegion::Type type)
{
    newSize = 2;
    if (type == VoronoiRegion::Edge01)
    {
        closestPoint = u * p0 + v * p1;
        newIndices[0] = 0;
        newIndices[1] = 1;
    }
    else if (type == VoronoiRegion::Edge02)
    {
        closestPoint = u * p0 + v * p2;
        newIndices[0] = 0;
        newIndices[1] = 2;
    }
    else if (type == VoronoiRegion::Edge03)
    {
        closestPoint = u * p0 + v * p3;
        newIndices[0] = 0;
        newIndices[1] = 3;
    }
    else if (type == VoronoiRegion::Edge12)
    {
        closestPoint = u * p1 + v * p2;
        newIndices[0] = 1;
        newIndices[1] = 2;
    }
    else if (type == VoronoiRegion::Edge13)
    {
        closestPoint = u * p1 + v * p3;
        newIndices[0] = 1;
        newIndices[1] = 3;
    }
    else if (type == VoronoiRegion::Edge23)
    {
        closestPoint = u * p2 + v * p3;
        newIndices[0] = 2;
        newIndices[1] = 3;
    }
    searchDirection = (q - closestPoint);
    return type;
}

VoronoiRegion::Type GetPoint(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
    size_t& newSize, int newIndices[4],
    Vector3& closestPoint, Vector3& searchDirection, VoronoiRegion::Type type)
{
    newSize = 1;
    if (type == VoronoiRegion::Point0)
    {
        closestPoint = p0;
        newIndices[0] = 0;
    }
    else if (type == VoronoiRegion::Point1)
    {
        closestPoint = p1;
        newIndices[0] = 1;
    }
    else if (type == VoronoiRegion::Point2)
    {
        closestPoint = p2;
        newIndices[0] = 2;
    }
    else if (type == VoronoiRegion::Point3)
    {
        closestPoint = p3;
        newIndices[0] = 3;
    }
    searchDirection = (q - closestPoint);
    return type;
}

//------------------------------------------------------------ Voronoi Region Tests
VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0,
    size_t& newSize, int newIndices[4],
    Vector3& closestPoint, Vector3& searchDirection)
{
    /******Student:Assignment5******/
    closestPoint = Vector3::cZero;
    searchDirection = Vector3::cZero;
    newSize = 0;
    return GetPoint(q, p0, q, q, q, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point0);
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1,
    size_t& newSize, int newIndices[4],
    Vector3& closestPoint, Vector3& searchDirection)
{
    /******Student:Assignment5******/
    auto result = VoronoiRegion::Unknown;
    closestPoint = Vector3::cZero;
    searchDirection = Vector3::cZero;
    newSize = 0;
    float u, v;
    BarycentricCoordinates(q, p0, p1, u, v);
    if (v <= 0)
        return GetPoint(q, p0, p1, q, q, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point0);
    else if (u <= 0)
        return GetPoint(q, p0, p1, q, q, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point1);
    else
        return GetEdge(q, p0, p1, q, q, newSize, newIndices, u, v, closestPoint, searchDirection, VoronoiRegion::Edge01);
    return result;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2,
    size_t& newSize, int newIndices[4],
    Vector3& closestPoint, Vector3& searchDirection)
{
    /******Student:Assignment5******/
    auto result = VoronoiRegion::Unknown;
    closestPoint = Vector3::cZero;
    searchDirection = Vector3::cZero;
    newSize = 0;
    float u20, v20, u01, v01, u12, v12;
    BarycentricCoordinates(q, p0, p1, u01, v01);
    BarycentricCoordinates(q, p1, p2, u12, v12);
    BarycentricCoordinates(q, p2, p0, u20, v20);
    if (v01 <= 0 && u20 <= 0)
        return GetPoint(q, p0, p1, p2, q, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point0);
    else if (v20 <= 0 && u12 <= 0)
        return GetPoint(q, p0, p1, p2, q, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point2);
    else if (v12 <= 0 && u01 <= 0)
        return GetPoint(q, p0, p1, p2, q, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point1);

    float u, v, w;
    BarycentricCoordinates(q, p0, p1, p2, u, v, w);
    if (u > 0 && v > 0 && w > 0)
        return GetTriangle(q, p0, p1, p2, q, newSize, newIndices, u, v, w, closestPoint, searchDirection, VoronoiRegion::Triangle012);
    else if (u01 > 0 && v01 > 0 && w < 0)
        return GetEdge(q, p0, p1, p2, q, newSize, newIndices, u01, v01, closestPoint, searchDirection, VoronoiRegion::Edge01);
    else if (u20 > 0 && v20 > 0 && v < 0)
        return GetEdge(q, p2, p1, p0, q, newSize, newIndices, u20, v20, closestPoint, searchDirection, VoronoiRegion::Edge02);
    else if (u12 > 0 && v12 > 0 && u < 0)
        return GetEdge(q, p0, p1, p2, q, newSize, newIndices, u12, v12, closestPoint, searchDirection, VoronoiRegion::Edge12);
    return result;
}

bool GetDirection(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
    Vector3 normal = Math::Cross(p1 - p0, p2 - p0);
    if (Math::Dot(normal, p3 - p0) > 0)
        normal *= -1;
    if (Math::Dot(normal, q - p0) > 0)
        return true;
    else
        return false;
}

VoronoiRegion::Type Gjk::IdentifyVoronoiRegion(const Vector3& q, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3,
    size_t& newSize, int newIndices[4],
    Vector3& closestPoint, Vector3& searchDirection)
{
    /******Student:Assignment5******/
    auto result = VoronoiRegion::Unknown;
    closestPoint = Vector3::cZero;
    searchDirection = Vector3::cZero;
    newSize = 0;
    float u01, v01, u02, v02, u03, v03;
    BarycentricCoordinates(q, p0, p1, u01, v01);
    BarycentricCoordinates(q, p0, p2, u02, v02);
    BarycentricCoordinates(q, p0, p3, u03, v03);

    float u12, v12, u23, v23;
    BarycentricCoordinates(q, p1, p2, u12, v12);
    BarycentricCoordinates(q, p2, p3, u23, v23);

    float u13, v13;
    BarycentricCoordinates(q, p1, p3, u13, v13);
    if (v01 < 0 && v02 < 0 && v03 < 0)
        return GetPoint(q, p0, p1, p2, p3, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point0);
    else if (u02 < 0 && u12 < 0 && v23 < 0)
        return GetPoint(q, p0, p1, p2, p3, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point2);
    else if (u01 < 0 && v12 < 0 && v13 < 0)
        return GetPoint(q, p0, p1, p2, p3, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point1);
    else if (u03 < 0 && u23 < 0 && u13 < 0)
        return GetPoint(q, p0, p1, p2, p3, newSize, newIndices, closestPoint, searchDirection, VoronoiRegion::Point3);

    float u012, v012, w012;
    BarycentricCoordinates(q, p0, p1, p2, u012, v012, w012);
    float u013, v013, w013;
    BarycentricCoordinates(q, p0, p1, p3, u013, v013, w013);
    float u023, v023, w023;
    BarycentricCoordinates(q, p0, p2, p3, u023, v023, w023);
    float u123, v123, w123;
    BarycentricCoordinates(q, p1, p2, p3, u123, v123, w123);

    if (u01 > 0 && v01 > 0 && w012 < 0 && w013 < 0)
        return GetEdge(q, p0, p1, p2, p3, newSize, newIndices, u01, v01, closestPoint, searchDirection, VoronoiRegion::Edge01);
    else if (u02 > 0 && v02 > 0 && v012 < 0 && w023 < 0)
        return GetEdge(q, p0, p1, p2, p3, newSize, newIndices, u02, v02, closestPoint, searchDirection, VoronoiRegion::Edge02);
    else if (u12 > 0 && v12 > 0 && u012 < 0 && w123 < 0)
        return GetEdge(q, p0, p1, p2, p3, newSize, newIndices, u12, v12, closestPoint, searchDirection, VoronoiRegion::Edge12);
    else if (u03 > 0 && v03 > 0 && v013 < 0 && v023 < 0)
        return GetEdge(q, p0, p1, p2, p3, newSize, newIndices, u03, v03, closestPoint, searchDirection, VoronoiRegion::Edge03);
    else if (u13 > 0 && v13 > 0 && v123 < 0 && u013 < 0)
        return GetEdge(q, p0, p1, p2, p3, newSize, newIndices, u13, v13, closestPoint, searchDirection, VoronoiRegion::Edge13);
    else if (u23 > 0 && v23 > 0 && u023 < 0 && u123 < 0)
        return GetEdge(q, p0, p1, p2, p3, newSize, newIndices, u23, v23, closestPoint, searchDirection, VoronoiRegion::Edge23);
    else if (u123 > 0 && v123 > 0 && w123 > 0 && GetDirection(q, p1, p2, p3, p0) == true)
        return GetTriangle(q, p0, p1, p2, p3, newSize, newIndices, u123, v123, w123, closestPoint, searchDirection, VoronoiRegion::Triangle123);
    else if (u023 > 0 && v023 > 0 && w023 > 0 && GetDirection(q, p0, p2, p3, p1) == true)
        return GetTriangle(q, p0, p1, p2, p3, newSize, newIndices, u023, v023, w023, closestPoint, searchDirection, VoronoiRegion::Triangle023);
    else if (u012 > 0 && v012 > 0 && w012 > 0 && GetDirection(q, p0, p1, p2, p3) == true)
        return GetTriangle(q, p0, p1, p2, p3, newSize, newIndices, u012, v012, w012, closestPoint, searchDirection, VoronoiRegion::Triangle012);
    else if (u013 > 0 && v013 > 0 && w013 > 0 && GetDirection(q, p0, p1, p3, p2) == true)
        return GetTriangle(q, p0, p1, p2, p3, newSize, newIndices, u013, v013, w013, closestPoint, searchDirection, VoronoiRegion::Triangle013);
    else if (result == VoronoiRegion::Unknown)
    {
        newSize = 4;
        newIndices[0] = 0;
        newIndices[1] = 1;
        newIndices[2] = 2;
        newIndices[3] = 3;
        result = VoronoiRegion::Tetrahedra0123;
        closestPoint = q;
    }
    return result;
}

Gjk::Gjk()
{
    // 1. Initialize the simplex (to one pioint for us) by searching in a random direction (difference of centers)
    // 2. Determine which voronoi region Q is in and reduce to the smallest simplex
    // 3. Compute P by projecting Q onto the new simplex
    // 4. If P is equal to Q then terminate
    // 5. Compute the new search diretion (Q - P) and search for a new point
    // 6. If the new point is no further than P in the search direction then terminate. 
    //    The length of the vector Q - P is the separation distance.
    // 7. Add the new point to the simplex and go to 2.

}

bool CheckOrigin(std::vector<Gjk::CsoPoint>& s, Vector3& d, float epsilon) {

    Vector3 p1, p2, dir1, dir2;

    Vector3 p0 = s.rbegin()->mCsoPoint;
    Vector3 q = p0 * -1;
    if (s.size() == 3) 
    {
        p1 = s[0].mCsoPoint;
        p2 = s[1].mCsoPoint;

        float u, v, w;
        bool check1 = BarycentricCoordinates(q, p0, p1, p2, u, v, w, epsilon);
        Vector3 point = p0 * u + p1 * v + p2 * w;
        dir1 = q - point;

        if (check1 == false)
            check1 = check1;

        bool check2 = BarycentricCoordinates(q, p0, p2, p1, u, v, w, epsilon);
        point = p0 * u + p2 * v + p1 * w;
        dir2 = q - point;

        if (check2 == false)
            check2 = check2;

        if(check1 == false && check2 == false && dir1.Dot(q) > 0 && dir2.Dot(q) > 0)
        {
            s.erase(s.begin() + 1);
            s.erase(s.begin());

            bool check = BarycentricCoordinates(q, p0, p2, u, v, epsilon);
            point = p0 * u + p2 * v;
            d = q - point;
            if (-epsilon < d.LengthSq() && d.LengthSq() < epsilon)
                return true;
        }
        else if (dir1.Dot(q) > 0)
        {
            s.erase(s.begin() + 1);
            d = (dir1);
        }
        else if (dir2.Dot(q) > 0)
        {
            s.erase(s.begin());
            d = (dir2);
        }
        else
            return true;
    }
    else
    {
        p1 = s[0].mCsoPoint;
        float u, v;
        bool check = BarycentricCoordinates(q, p0, p1, u, v, epsilon);
        Vector3 point = p0 * u + p1 * v;
        d = q - point;
        if (-epsilon < d.LengthSq() && d.LengthSq() < epsilon)
            return true;
    }
    return false;
}
// Returns true if the shapes intersect. If the shapes don't intersect then closestPoint is filled out with the closest points
// on each object as well as the cso point. Epsilon should be used for checking if sufficient progress has been made at any step.
// The debugging values are for your own use (make sure they don't interfere with the unit tests).
bool Gjk::Intersect(const SupportShape* shapeA, const SupportShape* shapeB, unsigned int maxIterations, CsoPoint& closestPoint, float epsilon, int debuggingIndex, bool debugDraw)
{
    // Implement the GJK algorithm to both determine if two convex shapes collide 
    // and to find the closest pair of points if they are separating.
    // To compute the closest points you should project the origin onto the CSO's simplex 
    // and then use barycentric coord. to compute each shapes's respective points.
    // ComputeSuopport function has been stubbed out for you to compute a CsoPoint from a given search direction.

    // If the two shapes do not intersect, 
    // then you should fill out closestPoint with the closest features of the two shapes.
    // When it comes to the termination conditions, 
    // check exactly for the zero vector when determining if the projected point is the origin.
    // Also, when checking if no more progress can be made, 
    // check the distance of the vector from the projection point to the new search point
    // and see if that distance (along the search direction) is less than or equal to the passed in epsilon.

    // While it shouldn't matter when it comes to the final result of GJK, 
    // there are some varying ways you can start GJK.
    // If you run into any output differences consider the following ways that I setup the algorithm.
    // When computing the initial search direction I used the difference of shape A's center and shape B's center.
    // If this produced zero vector I used Vector3(-1, 0, 0) as the starting search direction.

    Vector3 direction = shapeA->GetCenter() - shapeB->GetCenter();
    float dir = direction.LengthSq();
    if (-epsilon < dir && dir < epsilon)
        direction = Vector3(-1.f, 0.f, 0.f);

    std::vector<CsoPoint> points;
    points.push_back(ComputeSupport(shapeA, shapeB, direction));
    direction *= -1.f;

    for (unsigned i = 0; i < maxIterations; ++i)
    {
        points.push_back(ComputeSupport(shapeA, shapeB, direction));
        if (Math::Dot(points.rbegin()->mCsoPoint, direction) <= epsilon)
        {
            closestPoint.mCsoPoint = points.rbegin()->mCsoPoint;
            closestPoint.mPointA = points.rbegin()->mPointA;
            closestPoint.mPointB = points.rbegin()->mPointB;
            return false;
        }
        else
        {
            if(CheckOrigin(points, direction, epsilon) == true)
            {
                return true;
            }
        }

    }
    closestPoint.mCsoPoint = points.rbegin()->mCsoPoint;
    closestPoint.mPointA = points.rbegin()->mPointA;
    closestPoint.mPointB = points.rbegin()->mPointB;
    return false;
}

// Finds the point furthest in the given direction on the CSO (and the relevant points from each object)
Gjk::CsoPoint Gjk::ComputeSupport(const SupportShape* shapeA, const SupportShape* shapeB, const Vector3& direction)
{
    /******Student:Assignment5******/
    CsoPoint result;
    result.mPointA = shapeA->Support(direction);
    result.mPointB = shapeB->Support(-direction);
    result.mCsoPoint = result.mPointA - result.mPointB;
    return result;
}
