///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
  mColor = Vector4(.6f);
  mMask = (unsigned int)-1;
  mTimer = 0;
  mOnTop = false;
  mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
  mColor = color;
  return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
  mOnTop = state;
  return *this;
}

DebugShape& DebugShape::Time(float time)
{
  mTimer = time;
  return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
  mMask = 1 << bitIndex;
  return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
  mTransform = transform;
  return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
  mActiveMask = (unsigned int)-1;
  mApplication = nullptr;
}

void DebugDrawer::Update(float dt)
{
  std::vector<DebugShape> newShapes;
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];
    shape.mTimer -= dt;

    // If the shape still has time left then add it to the list of shapes to keep drawing,
    // anything that has a timer that ran out will not be in the new list
    if(shape.mTimer >= 0)
      newShapes.push_back(shape);
  }

  mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];

    // If the shape doesn't have one of the active mask bits set then don't draw it
    if((shape.mMask & mActiveMask) == 0)
      continue;
    
    // If this shape always draws on top then disable depth testing
    if(shape.mOnTop)
      glDisable(GL_DEPTH_TEST);


    // Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
    float radians;
    Vector3 scale, translation, axis;
    Matrix3 rotationMat;
    shape.mTransform.Decompose(&scale, &rotationMat, &translation);
    Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
    glPushMatrix();
    // Set the transform
    glTranslatef(translation.x, translation.y, translation.z);
    glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
    glScalef(scale.x, scale.y, scale.z);

    glBegin(GL_LINES);
    glColor3fv(shape.mColor.array);

    // Draw all of the line segments of this shape
    for(size_t j = 0; j < shape.mSegments.size(); ++j)
    {
      LineSegment& segment = shape.mSegments[j];

      glVertex3fv(segment.mStart.array);
      glVertex3fv(segment.mEnd.array);
    }

    glEnd();
    glPopMatrix();

    // Make sure to re-enable depth testing
    if(shape.mOnTop)
      glEnable(GL_DEPTH_TEST);
  }
}

DebugShape& DebugDrawer::GetNewShape()
{
  mShapes.push_back(DebugShape());
  return mShapes.back();
}

void DebugDrawer::AddLine(DebugShape& shape, const Vector3& start, const Vector3& end)
{
  shape.mSegments.push_back(LineSegment(start, end));
}

void DebugDrawer::AddRay(DebugShape& shape, const Vector3& start, const Vector3& direction, float t)
{
  Vector3 end = start + direction * t;
  AddLine(shape, start, end);

  Vector3 normDir = direction;
  Math::AttemptNormalize(&normDir);
  AddArrowHead(shape, end, normDir, 0.2f);
}

void DebugDrawer::AddDisc(DebugShape& shape, const Vector3& center, const Vector3& discNormal, float radius)
{
  Vector3 basis0, basis1;
  Math::GenerateOrthonormalBasis(discNormal, &basis0, &basis1);
  AddDisc(shape, center, basis0, basis1, radius);
}

void DebugDrawer::AddSphereHorizonDisc(DebugShape& shape, const Vector3& sphereCenter, float sphereRadius)
{
  //http:\stackoverflow.com\questions\21648630\radius-of-projected-sphere-in-screen-space

  if(mApplication == nullptr)
    return;
  Vector3 eye = mApplication->mCamera.mTranslation;
  Vector3 eyeDir = mApplication->mCamera.GetDirection();
  Vector3 eyeToCenter = sphereCenter - eye;

  float distanceToSphere = eyeToCenter.AttemptNormalize();
  if(distanceToSphere > sphereRadius)
  {
    // Compute the horizon of the sphere
    // this is the out most edge visible to the eye
    float distSq = distanceToSphere * distanceToSphere;
    float radiusSq = sphereRadius * sphereRadius;

    //compute the radius of the sphere when projected on the plane defined by the camera
    float projectedRadius = Math::Sqrt(distSq - radiusSq) * sphereRadius / distanceToSphere;
    //compute how far the center of the sphere needs to be moved to get the projection center
    float distanceFromCenter = Math::Sqrt(radiusSq - projectedRadius * projectedRadius);

    //compute the center of the projected sphere
    Vector3 newCenter = sphereCenter - eyeToCenter * distanceFromCenter;
    AddDisc(shape, newCenter, eyeToCenter, projectedRadius);
  }
}

void DebugDrawer::AddDisc(DebugShape& shape, const Vector3& center, const Vector3& basis0, const Vector3& basis1, float radius)
{
  float subDivisions = 20;
  float step = Math::cTwoPi / subDivisions;
  float radians = 0;

  for(int i = 0; i < subDivisions; ++i)
  {
    Vector3 point0 = center + radius * (basis0 * Math::Cos(radians) + basis1 * Math::Sin(radians));
    radians += step;
    Vector3 point1 = center + radius * (basis0 * Math::Cos(radians) + basis1 * Math::Sin(radians));

    AddLine(shape, point0, point1);
  }
}

void DebugDrawer::AddQuad(DebugShape& shape, const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
  AddLine(shape, p0, p1);
  AddLine(shape, p1, p2);
  AddLine(shape, p2, p3);
  AddLine(shape, p3, p0);
}

void DebugDrawer::AddArrowHead(DebugShape& shape, const Vector3& p, const Vector3& direction, float headSize)
{
  Vector3 axis0, axis1;
  Math::GenerateOrthonormalBasis(direction, &axis0, &axis1);

  Vector3 discCenter = p - direction * 2 * headSize;
  AddDisc(shape, discCenter, axis0, axis1, headSize);

  AddLine(shape, p, discCenter + axis0 * headSize);
  AddLine(shape, p, discCenter + axis0 * -headSize);
  AddLine(shape, p, discCenter + axis1 * headSize);
  AddLine(shape, p, discCenter + axis1 * -headSize);
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
  return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
  // Draw a simple line
  DebugShape& shape = GetNewShape();
  AddLine(shape, line.mStart, line.mEnd);
  return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
  // Draw a ray to a given t-length. The ray must have an arrow head for visualization
  DebugShape& shape = GetNewShape();
  AddRay(shape, ray.mStart, ray.mDirection, t);
  return shape;
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
  // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
  // Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
  DebugShape& shape = GetNewShape();
  AddDisc(shape, sphere.mCenter, Vector3::cXAxis, Vector3::cYAxis, sphere.mRadius);
  AddDisc(shape, sphere.mCenter, Vector3::cXAxis, Vector3::cZAxis, sphere.mRadius);
  AddDisc(shape, sphere.mCenter, Vector3::cYAxis, Vector3::cZAxis, sphere.mRadius);
  
  AddSphereHorizonDisc(shape, sphere.mCenter, sphere.mRadius);

  return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
  // Draw all edges of an aabb. Make sure to not mis-match edges!
  DebugShape& shape = GetNewShape();  

  Vector3 min = aabb.GetMin();
  Vector3 max = aabb.GetMax();
  AddLine(shape, Vector3(min.x, min.y, min.z), Vector3(max.x, min.y, min.z));
  AddLine(shape, Vector3(min.x, max.y, min.z), Vector3(max.x, max.y, min.z));
  AddLine(shape, Vector3(min.x, min.y, max.z), Vector3(max.x, min.y, max.z));
  AddLine(shape, Vector3(min.x, max.y, max.z), Vector3(max.x, max.y, max.z));
  
  AddLine(shape, Vector3(min.x, min.y, min.z), Vector3(min.x, max.y, min.z));
  AddLine(shape, Vector3(max.x, min.y, min.z), Vector3(max.x, max.y, min.z));
  AddLine(shape, Vector3(min.x, min.y, max.z), Vector3(min.x, max.y, max.z));
  AddLine(shape, Vector3(max.x, min.y, max.z), Vector3(max.x, max.y, max.z));
  
  AddLine(shape, Vector3(min.x, min.y, min.z), Vector3(min.x, min.y, max.z));
  AddLine(shape, Vector3(min.x, max.y, min.z), Vector3(min.x, max.y, max.z));
  AddLine(shape, Vector3(max.x, min.y, min.z), Vector3(max.x, min.y, max.z));
  AddLine(shape, Vector3(max.x, max.y, min.z), Vector3(max.x, max.y, max.z));

  return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
  // Draw the 3 edges of a triangles
  DebugShape& shape = GetNewShape();

  AddLine(shape, triangle.mPoints[0], triangle.mPoints[1]);
  AddLine(shape, triangle.mPoints[0], triangle.mPoints[2]);
  AddLine(shape, triangle.mPoints[1], triangle.mPoints[2]);

  return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
  // Draw a quad with a normal at the plane's center.
  DebugShape& shape = GetNewShape();

  Vector3 normal = plane.GetNormal();
  Vector3 basis0, basis1;
  Math::GenerateOrthonormalBasis(normal, &basis0, &basis1);

  Vector3 center = normal * plane.GetDistance();

  Vector3 pt0 = center + basis0 * sizeX + basis1 * sizeY;
  Vector3 pt1 = center - basis0 * sizeX + basis1 * sizeY;
  Vector3 pt2 = center - basis0 * sizeX - basis1 * sizeY;
  Vector3 pt3 = center + basis0 * sizeX - basis1 * sizeY;

  AddQuad(shape, pt0, pt1, pt2, pt3);
  AddRay(shape, center, normal, 5);

  return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
  // Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
  DebugShape& shape = GetNewShape();

  AddQuad(shape, p0, p1, p2, p3);

  return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
  // Draw the 6 faces of the frustum using the 8 frustum points.
  // See Frustum.Set for the point order. For example, Points[4] is left-bottom-front.
  DebugShape& shape = GetNewShape();

  //left
  AddQuad(shape, frustum.mPoints[7], frustum.mPoints[4], frustum.mPoints[0], frustum.mPoints[3]);
  //right
  AddQuad(shape, frustum.mPoints[2], frustum.mPoints[1], frustum.mPoints[5], frustum.mPoints[6]);
  //top
  AddQuad(shape, frustum.mPoints[7], frustum.mPoints[3], frustum.mPoints[2], frustum.mPoints[6]);
  //bot
  AddQuad(shape, frustum.mPoints[0], frustum.mPoints[4], frustum.mPoints[5], frustum.mPoints[1]);
  //near
  AddQuad(shape, frustum.mPoints[3], frustum.mPoints[0], frustum.mPoints[1], frustum.mPoints[2]);
  //far
  AddQuad(shape, frustum.mPoints[6], frustum.mPoints[5], frustum.mPoints[4], frustum.mPoints[7]);

  return shape;
}
