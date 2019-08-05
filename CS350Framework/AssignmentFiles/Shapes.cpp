///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"
#include <iostream>
//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment()
{
    mStart = mEnd = Vector3::cZero;
}

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
    mStart = start;
    mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
    return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray()
{
    mStart = mDirection = Vector3::cZero;
}

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
    mStart = start;
    mDirection = dir;
}

Ray Ray::Transform(const Math::Matrix4& transform) const
{
    Ray transformedRay;
    transformedRay.mStart = Math::TransformPoint(transform, mStart);
    transformedRay.mDirection = Math::TransformDirection(transform, mDirection);
    return transformedRay;
}

Vector3 Ray::GetPoint(float t) const
{
    return mStart + mDirection * t;
}

DebugShape& Ray::DebugDraw(float t) const
{
    return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------PCA Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
    /******Student:Assignment2******/
    int size = points.size();
    
    Vector3 u;
    for(int i = 0; i < size; ++i)
        u += points[i];
    u /= float(size);

    Matrix3 c;
    c.ZeroOut();
    for (int i = 0; i < size; ++i)
    {
        c.m00 += (points[i].x - u.x) * (points[i].x - u.x);
        c.m01 += (points[i].x - u.x) * (points[i].y - u.y);
        c.m02 += (points[i].x - u.x) * (points[i].z - u.z);
        c.m10 += (points[i].y - u.y) * (points[i].x - u.x);
        c.m11 += (points[i].y - u.y) * (points[i].y - u.y);
        c.m12 += (points[i].y - u.y) * (points[i].z - u.z);
        c.m20 += (points[i].z - u.z) * (points[i].x - u.x);
        c.m21 += (points[i].z - u.z) * (points[i].y - u.y);
        c.m22 += (points[i].z - u.z) * (points[i].z - u.z);
    }
    return c / float(size);
}

Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
    /******Student:Assignment2******/
    // Compute the jacobi rotation matrix that will turn the largest (magnitude) off-diagonal element of the input
    // matrix into zero. Note: the input matrix should always be (near) symmetric.
    Matrix3 result;
    result.SetIdentity();

    //Find the largest
    int p = 0, q = 1;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (abs(matrix[i][j]) > abs(matrix[p][q]) && i != j)
            {
                p = i;
                q = j;
            }
        }
    }

    float b = (matrix[q][q] - matrix[p][p]) / (matrix[p][q] + matrix[q][p]);
    float t = Math::GetSign(b) / (abs(b) + sqrt(b * b + 1));
    float c = sqrt(1 / (t * t + 1));
    float s = t * c;
    
    result[p][p] = c;
    result[p][q] = s;
    result[q][p] = -s;
    result[q][q] = c;

    return result;
}

void ComputeEigenValuesAndVectors(const Matrix3& covariance, Vector3& eigenValues, Matrix3& eigenVectors, int maxIterations)
{
    /******Student:Assignment2******/
    // Iteratively rotate off the largest off-diagonal elements until the resultant matrix is diagonal or maxIterations.
    Matrix3 a(covariance);
    for (int i = 0; i < maxIterations; ++i)
    {
        Matrix3 jacobi = ComputeJacobiRotation(a);
        a = jacobi.Inverted() * a * jacobi;
        eigenVectors = eigenVectors * jacobi;

        bool checker = true;
        for (int j = 0; j < 3; ++j)
        {
            for (int k = 0; k < 3; ++k)
            {
                if (a[j][k] != 0 && j != k)
                    checker = false;
            }
        }

        if (checker == true)
            break;
    }
    eigenValues[0] = a[0][0];
    eigenValues[1] = a[1][1];
    eigenValues[2] = a[2][2];
}


//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
    mCenter = Vector3::cZero;
    mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
    mCenter = center;
    mRadius = radius;
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
    /******Student:Assignment2******/
    // The centroid method is roughly describe as: find the centroid (not mean) of all
    // points and then find the furthest away point from the centroid.
    Vector3 min = points[0];
    Vector3 max = points[0];
    int size = points.size();
    for (int i = 1; i < size; ++i)
    {
        if (min.x > points[i].x)
            min.x = points[i].x;
        if (min.y > points[i].y)
            min.y = points[i].y;
        if (min.z > points[i].z)
            min.z = points[i].z;

        if (max.x < points[i].x)
            max.x = points[i].x;
        if (max.y < points[i].y)
            max.y = points[i].y;
        if (max.z < points[i].z)
            max.z = points[i].z;
    }
    float rad = 0;
    Vector3 center = (min + max) / 2;
    for (int i = 0; i < size; ++i)
    {
        float tmp = center.DistanceSq(points[i]);
        if (rad < tmp)
            rad = tmp;
    }
    (*this).mCenter = center;
    (*this).mRadius = sqrt(rad);
}

//This is helper function to calculate the center and radius in Ritter & PCA
void CalculateCenterRadius(const std::vector<Vector3>& points, Vector3 &mCenter, float& mRadius, float originalRad)
{
    int size = points.size();
    for (int i = 0; i < size; ++i)
    {
        float calcRad = points[i].DistanceSq(mCenter);
        if (calcRad > originalRad)
        {
            float instantRad = sqrt(calcRad);
            mRadius = (mRadius + instantRad) * 0.5f;
            originalRad = mRadius * mRadius;
            float diff = instantRad - mRadius;
            mCenter = (mRadius * mCenter + diff * points[i]) / instantRad;
        }
    }
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
    /******Student:Assignment2******/
    int xMin = 0, xMax = 0;
    int yMin = 0, yMax = 0;
    int zMin = 0, zMax = 0;
    int size = points.size();
    for (int i = 1; i < size; ++i)
    {
        if (points[xMin].x > points[i].x)
            xMin = i;
        if (points[yMin].y > points[i].y)
            yMin = i;
        if (points[zMin].z > points[i].z)
            zMin = i;

        if (points[xMax].x < points[i].x)
            xMax = i;
        if (points[yMax].y < points[i].y)
            yMax = i;
        if (points[zMax].z < points[i].z)
            zMax = i;
    }

    float xRad = points[xMax].Distance(points[xMin]) / 2;
    float yRad = points[yMax].Distance(points[yMin]) / 2;
    float zRad = points[zMax].Distance(points[zMin]) / 2;

    int max, min;
    if(xRad > yRad && xRad > zRad)
        max = xMax, min = xMin;
    else if (yRad > xRad && yRad > zRad)
        max = yMax, min = yMin;
    else
        max = zMax, min = zMin;

    mCenter = (points[max] + points[min]) / 2;
    float radius = points[max].DistanceSq(mCenter);
    mRadius = sqrt(radius);
    CalculateCenterRadius(points, mCenter, mRadius, radius);
}

void Sphere::ComputePCA(const std::vector<Vector3>& points, int maxIterations)
{
    /******Student:Assignment2******/
    int size = points.size();
    Vector3 eigenValues;
    Matrix3 eigenVectors;
    eigenVectors.SetIdentity();

    ComputeEigenValuesAndVectors(ComputeCovarianceMatrix(points), eigenValues, eigenVectors, maxIterations);
    Vector3 eigenVector(eigenVectors[0][0], eigenVectors[1][1], eigenVectors[2][2]);

    int l = 0;
    float lValue = abs(eigenValues[0]);
    for (int i = 1; i < 3; i++)
    {
        if (lValue < abs(eigenValues[i]))
        {
            lValue = abs(eigenValues[i]);
            l = i;
        }
    }
    for (int i = 0; i < 3; i++)
        eigenValues[i] = eigenVectors[i][l];
    
    int min = -1, max = -1;
    float distMin = FLT_MAX, distMax = 0.f;
    for (int i = 0; i < size; ++i)
    {
        float distacne = points[i].Dot(eigenValues);

        if (distacne < distMin)
        {
            distMin = distacne;
            min = i;
        }
        if (distacne > distMax)
        {
            distMax = distacne;
            max = i;
        }
    }

    mCenter = (points[max] + points[min]) / 2;
    float radius = points[max].DistanceSq(mCenter);
    mRadius = sqrt(radius);
    CalculateCenterRadius(points, mCenter, mRadius, radius);
}

bool Sphere::ContainsPoint(const Vector3& point)
{
    return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const
{
    return mCenter;
}

float Sphere::GetRadius() const
{
    return mRadius;
}

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
    float posDiff = Math::Length(mCenter - rhs.mCenter);
    float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

    return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
    return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
    //set the aabb to an initial bad value (where the min is smaller than the max)
    mMin.Splat(Math::PositiveMax());
    mMax.Splat(Math::NegativeMin());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
    mMin = min;
    mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center, const Vector3& halfExtents)
{
    return Aabb(center - halfExtents, center + halfExtents);
}

Aabb Aabb::BuildFromMinMax(const Vector3& min, const Vector3& max)
{
    return Aabb(min, max);
}

float Aabb::GetVolume() const
{
    /******Student:Assignment2******/
    // Return the aabb's volume
    Vector3 l = (mMax - mMin);
    return (l.x * l.y * l.z);
}

float Aabb::GetSurfaceArea() const
{
    /******Student:Assignment2******/
    // Return the aabb's surface area
    Vector3 l = (mMax - mMin);
    return (l.x * l.y + l.x * l.z + l.y * l.z) * 2;
}

void Aabb::Compute(const std::vector<Vector3>& points)
{
    mMin.Splat(Math::PositiveMax());
    mMax.Splat(Math::NegativeMin());
    for (size_t i = 0; i < points.size(); ++i)
    {
        const Vector3& point = points[i];
        mMin = Math::Min(mMin, point);
        mMax = Math::Max(mMax, point);
    }
}

bool Aabb::Contains(const Aabb& aabb) const
{
    /******Student:Assignment2******/
    // Return if aabb is completely contained in this
    if ((mMax.x >= aabb.GetMax().x && mMax.y >= aabb.GetMax().y && mMax.z >= aabb.GetMax().z) 
        && (mMin.x <= aabb.GetMin().x && mMin.y <= aabb.GetMin().y && mMin.z <= aabb.GetMin().z))
        return true;
    else
        return false;
}

void Aabb::Expand(const Vector3& point)
{
    for (size_t i = 0; i < 3; ++i)
    {
        mMin[i] = Math::Min(mMin[i], point[i]);
        mMax[i] = Math::Max(mMax[i], point[i]);
    }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
    Aabb result;
    for (size_t i = 0; i < 3; ++i)
    {
        result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
        result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
    }
    return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
    float pos1Diff = Math::Length(mMin - rhs.mMin);
    float pos2Diff = Math::Length(mMax - rhs.mMax);

    return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Matrix4& transform)
{
    /******Student:Assignment2******/
    // Compute aabb of the this aabb after it is transformed.
    // You should use the optimize method discussed in class (not transforming all 8 points).

    Vector3 r = mMax - mMin;
    r *= 0.5f;
    Vector3 c = mMin + r;
    Vector3 nr(
        Math::Abs(transform.m00) * r.x +
        Math::Abs(transform.m01) * r.y +
        Math::Abs(transform.m02) * r.z,
        Math::Abs(transform.m10) * r.x +
        Math::Abs(transform.m11) * r.y +
        Math::Abs(transform.m12) * r.z,
        Math::Abs(transform.m20) * r.x +
        Math::Abs(transform.m21) * r.y +
        Math::Abs(transform.m22) * r.z
    );

    Vector3 nc(
        transform.m00 * c.x +
        transform.m01 * c.y +
        transform.m02 * c.z,
        transform.m10 * c.x +
        transform.m11 * c.y +
        transform.m12 * c.z,
        transform.m20 * c.x +
        transform.m21 * c.y +
        transform.m22 * c.z
    );
    Vector3 translation(transform.m03, transform.m13, transform.m23);
    nc += translation;

    mMin = nc - nr;
    mMax = nc + nr;

}

Vector3 Aabb::GetMin() const
{
    return mMin;
}

Vector3 Aabb::GetMax() const
{
    return mMax;
}

Vector3 Aabb::GetCenter() const
{
    return (mMin + mMax) * 0.5f;
}

Vector3 Aabb::GetHalfSize() const
{
    return (mMax - mMin) * 0.5f;
}

DebugShape& Aabb::DebugDraw() const
{
    return gDebugDrawer->DrawAabb(*this);
}

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle()
{
    mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero;
}

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    mPoints[0] = p0;
    mPoints[1] = p1;
    mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
    return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane()
{
    mData = Vector4::cZero;
}

Plane::Plane(const Triangle& triangle)
{
    Set(triangle);
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
    Set(normal, point);
}

void Plane::Set(const Triangle& triangle)
{
    // Set from the triangle's points
    Set(triangle.mPoints[0], triangle.mPoints[1], triangle.mPoints[2]);
}

/******Student:Assignment1******/
void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    // Set mData from the 3 points. Note: You should most likely normalize the plane normal.
    Vector3 r0 = p1 - p0;
    Vector3 r1 = p2 - p0;
    Set(r0.Cross(r1).Normalized(), p0);
}

/******Student:Assignment1******/
void Plane::Set(const Vector3& normal, const Vector3& point)
{
    // Set mData from the normal and point. Note: You should most likely normalize the plane normal.
    Vector3 result = normal.Normalized();
    mData.x = result.x;
    mData.y = result.y;
    mData.z = result.z;
    mData.w = result.Dot(point);
}

Vector3 Plane::GetNormal() const
{
    return Vector3(mData.x, mData.y, mData.z);
}

float Plane::GetDistance() const
{
    return mData.w;
}

DebugShape& Plane::DebugDraw(float size) const
{
    return DebugDraw(size, size);
}

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
    return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn, const Vector3& ltn,
    const Vector3& lbf, const Vector3& rbf, const Vector3& rtf, const Vector3& ltf)
{
    mPoints[0] = lbn;
    mPoints[1] = rbn;
    mPoints[2] = rtn;
    mPoints[3] = ltn;
    mPoints[4] = lbf;
    mPoints[5] = rbf;
    mPoints[6] = rtf;
    mPoints[7] = ltf;

    //left
    mPlanes[0].Set(lbf, ltf, lbn);
    //right
    mPlanes[1].Set(rbn, rtf, rbf);
    //top
    mPlanes[2].Set(ltn, ltf, rtn);
    //bot
    mPlanes[3].Set(rbn, lbf, lbn);
    //near
    mPlanes[4].Set(lbn, ltn, rbn);
    //far
    mPlanes[5].Set(rbf, rtf, lbf);
}

Math::Vector4* Frustum::GetPlanes() const
{
    return (Vector4*)mPlanes;
}

DebugShape& Frustum::DebugDraw() const
{
    return gDebugDrawer->DrawFrustum(*this);
}
