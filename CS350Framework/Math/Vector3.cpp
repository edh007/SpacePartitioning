///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis, Benjamin Strukus
/// Copyright 2010-2012, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#include "Utilities.hpp"
#include "Vector3.hpp"

namespace Math
{

const Vector3 Vector3::cZero(0.0f, 0.0f, 0.0f);
const Vector3 Vector3::cXAxis(1.0f, 0.0f, 0.0f);
const Vector3 Vector3::cYAxis(0.0f, 1.0f, 0.0f);
const Vector3 Vector3::cZAxis(0.0f, 0.0f, 1.0f);

float* Vector3::ToFloats()
{
  return (float*)this;
}

Vector3::Vector3()
{
  Splat(0.0f);
}

Vector3::Vector3(float xx, float yy, float zz)
{
  x = xx;
  y = yy;
  z = zz;
}

Vector3::Vector3(float xyz)
{
  x = xyz;
  y = xyz;
  z = xyz;
}

Vector3::Vector3(Vec2Param rhs, float zz)
{
  x = rhs.x;
  y = rhs.y;
  z = zz;
}

Vector3::Vector3(const float* data)
{
  array[0] = data[0];
  array[1] = data[1];
  array[2] = data[2];
}

float& Vector3::operator[](unsigned index)
{
  ErrorIf(index > 2, "Math::Vector3 - Subscript out of range.");
  return array[index];
}

float Vector3::operator[](unsigned index) const
{
  ErrorIf(index > 2, "Math::Vector3 - Subscript out of range.");
  return array[index];
}

//---------------------------------------------------- Binary Vector Comparisons
bool Vector3::operator==(Vec3Param rhs) const
{
  return x == rhs.x && 
         y == rhs.y && 
         z == rhs.z;
}

bool Vector3::operator!=(Vec3Param rhs) const
{
  return !(*this == rhs);
}

void Vector3::Set(float x_, float y_, float z_)
{
  x = x_;
  y = y_;
  z = z_;
}

void Vector3::Splat(float xyz)
{
  x = y = z = xyz;
}

void Vector3::ZeroOut()
{
  array[0] = 0.0f;
  array[1] = 0.0f;
  array[2] = 0.0f;
}

float Vector3::Length() const
{
  return Sqrt(LengthSq());
}

float Vector3::LengthSq() const
{
  return Dot(*this);
}

float Vector3::Distance(Vec3Param otherPoint) const
{
  return (*this - otherPoint).Length();
}

float Vector3::DistanceSq(Vec3Param otherPoint) const
{
  return (*this - otherPoint).LengthSq();
}

Vector3 Vector3::Normalized() const
{
  Vector3 ret = *this;
  ret /= Length();
  return ret;
}

float Vector3::Normalize()
{
  float length = Length();
  *this /= length;
  return length;
}

float Vector3::AttemptNormalize()
{
  float lengthSq = LengthSq();

  //Although the squared length may not be zero, the sqrt of a small number
  //may be truncated to zero, causing a divide by zero crash.  This is why
  //we check to make sure that it is larger than our epsilon squared.
  if(lengthSq >= DebugEpsilon() * DebugEpsilon())
  {
    lengthSq = Sqrt(lengthSq);
    *this /= lengthSq;
  }
  return lengthSq;
}

Vector3 Vector3::AttemptNormalized() const
{
  Vector3 ret = *this;
  ret.AttemptNormalize();
  return ret;
}

Vec3Ref Vector3::Negate()
{
  (*this) *= -1.0f;
  return *this;
}

bool Vector3::Valid() const
{
  return IsValid(x) && IsValid(y) && IsValid(z);
}

Vector3 Vector3::Cross(Vec3Param rhs) const
{
  Vector3 ret;
  ret.x = y * rhs.z - z * rhs.y;
  ret.y = z * rhs.x - x * rhs.z;
  ret.z = x * rhs.y - y * rhs.x;
  return ret;
}

//------------------------------------------------------------- Global Functions
Vector3 operator*(float lhs, Vec3Param rhs)
{
  return rhs * lhs;
}

float Distance(Vec3Param lhs, Vec3Param rhs)
{
  return Length(rhs - lhs);
}

float DistanceSq(Vec3Param lhs, Vec3Param rhs)
{
  return LengthSq(rhs - lhs);
}

float Length(Vec3Param vect)
{
  return vect.Length();
}

float LengthSq(Vec3Param vect)
{
  return vect.LengthSq();
}

Vector3 Normalized(Vec3Param vect)
{
  return vect.Normalized();
}

float Normalize(Vec3Ptr vect)
{
  ErrorIf(vect == nullptr, "Vector3 - Null pointer passed for vector.");
  return vect->Normalize();
}

float AttemptNormalize(Vec3Ptr vect)
{
  ErrorIf(vect == nullptr, "Vector3 - Null pointer passed for vector.");
  return vect->AttemptNormalize();
}

Vector3 AttemptNormalized(Vec3Param vec)
{
  return vec.AttemptNormalized();
}

Vector3 Cross(Vec3Param lhs, Vec3Param rhs)
{
  return lhs.Cross(rhs);
}

void ZeroOut(Vec3Ptr vec)
{
  ErrorIf(vec == nullptr, "Vector3 - Null pointer passed for vector.");
  vec->ZeroOut();
}

void Negate(Vec3Ptr vec)
{
  ErrorIf(vec == nullptr, "Vector3 - Null pointer passed for vector.");
  *vec *= -1.0f;
}

Vector3 Negated(Vec3Param vec)
{
  return Vector3(-vec.x, -vec.y, -vec.z);
}

Vector3 Abs(Vec3Param vec)
{
  return Vector3(Math::Abs(vec.x), Math::Abs(vec.y), Math::Abs(vec.z));
}

Vector3 Min(Vec3Param lhs, Vec3Param rhs)
{
  return Vector3(Math::Min(lhs.x, rhs.x),
                 Math::Min(lhs.y, rhs.y),
                 Math::Min(lhs.z, rhs.z));
}

Vector3 Max(Vec3Param lhs, Vec3Param rhs)
{
  return Vector3(Math::Max(lhs.x, rhs.x),
                 Math::Max(lhs.y, rhs.y),
                 Math::Max(lhs.z, rhs.z));
}

Vector3 Lerp(Vec3Param start, Vec3Param end, float tValue)
{
  return Vector3(start[0] + tValue * (end[0] - start[0]),
                 start[1] + tValue * (end[1] - start[1]),
                 start[2] + tValue * (end[2] - start[2]));
}

void Clamp(Vec3Ptr vec, float min, float max)
{
  ErrorIf(vec == nullptr, "Vector3 - Null pointer passed for vector.");
  vec->x = Math::Clamp(vec->x, min, max);
  vec->y = Math::Clamp(vec->y, min, max);
  vec->z = Math::Clamp(vec->z, min, max);
}

Vector3 Clamped(Vec3Param vec, float min, float max)
{
  Vector3 results;
  results[0] = Math::Clamp(vec[0], min, max);
  results[1] = Math::Clamp(vec[1], min, max);
  results[2] = Math::Clamp(vec[2], min, max);
  return results;
}

bool AllLess(Vec3Param lhs, Vec3Param rhs)
{
  return (lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z);
}

bool AnyLess(Vec3Param lhs, Vec3Param rhs)
{
  return (lhs.x < rhs.x || lhs.y < rhs.y || lhs.z < rhs.z);
}

bool AllGreater(Vec3Param lhs, Vec3Param rhs)
{
  return (lhs.x > rhs.x && lhs.y > rhs.y && lhs.z > rhs.z);
}

bool AnyGreater(Vec3Param lhs, Vec3Param rhs)
{
  return (lhs.x > rhs.x || lhs.y > rhs.y || lhs.z > rhs.z);
}


//-------------------------------------------------------------- Unary Operators
Vector3 Vector3::operator-() const
{
  return Vector3(-x, -y, -z);
}

//------------------------------------------ Binary Assignment Operators (reals)
void Vector3::operator*=(float rhs)
{
  x *= rhs;
  y *= rhs;
  z *= rhs;
}

void Vector3::operator/=(float rhs)
{
  ErrorIf(rhs == 0.0f, "Math::Vector3 - Division by zero.");
  x /= rhs;
  y /= rhs;
  z /= rhs;
}

//----------------------------------------------------- Binary Operators (reals)

Vector3 Vector3::operator*(float rhs) const
{
  Vector3 ret = *this;
  ret *= rhs;
  return ret;
}

Vector3 Vector3::operator/(float rhs) const
{
  ErrorIf(Math::DebugIsZero(rhs), "Math::Vector3 - Division by zero.");
  Vector3 ret = *this;
  ret /= rhs;
  return ret;
}

//----------------------------------------- Binary Assignment Operator (Vectors)

void Vector3::operator+=(Vec3Param rhs)
{
  x += rhs.x;
  y += rhs.y;
  z += rhs.z;
}

void Vector3::operator-=(Vec3Param rhs)
{
  x -= rhs.x;
  y -= rhs.y;
  z -= rhs.z;
}

//--------------------------------------------------- Binary Operators (Vectors)
Vector3 Vector3::operator+(Vec3Param rhs) const
{
  Vector3 ret = *this;
  ret += rhs;
  return ret;
}

Vector3 Vector3::operator-(Vec3Param rhs) const
{
  Vector3 ret = *this;
  ret -= rhs;
  return ret;
}

Vector3 Vector3::operator*(Vec3Param rhs) const
{
  return Vector3(x * rhs.x, y * rhs.y, z * rhs.z);
}

Vector3 Vector3::operator/(Vec3Param rhs) const
{  
  ErrorIf(rhs.x == 0.0f || rhs.y == 0.0f || rhs.z == 0.0f,
          "Vector3 - Division by zero.");
  return Vector3(x / rhs.x, y / rhs.y, z / rhs.z);
}

void Vector3::operator*=(Vec3Param rhs)
{
  x *= rhs.x;
  y *= rhs.y;
  z *= rhs.z;
}

void Vector3::operator/=(Vec3Param rhs)
{
  x /= rhs.x;
  y /= rhs.y;
  z /= rhs.z;
}

float Vector3::Dot(Vec3Param rhs) const
{
  return x * rhs.x + y * rhs.y + z * rhs.z;
}

float Dot(Vec3Param lhs, Vec3Param rhs)
{  
  return lhs.Dot(rhs);
}


}// namespace Math
