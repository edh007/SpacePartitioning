///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis, Benjamin Strukus
/// Copyright 2010-2012, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#include "Utilities.hpp"
#include "Vector2.hpp"

namespace Math
{

const Vector2 Vector2::cZero(0.0f, 0.0f);
const Vector2 Vector2::cXAxis(1.0f, 0.0f);
const Vector2 Vector2::cYAxis(0.0f, 1.0f);

float* Vector2::ToFloats()
{
  return (float*)this;
}

Vector2::Vector2()
{
  Splat(0.0f);
}

Vector2::Vector2(float x_, float y_)
{
  x = x_;
  y = y_;
}

Vector2::Vector2(const float* data)
{
  array[0] = data[0];
  array[1] = data[1];
}

float& Vector2::operator[](unsigned index)
{
  ErrorIf(index > 1, "Math::Vector2 - Subscript out of range.");
  return array[index];
}

float Vector2::operator[](unsigned index) const
{
  ErrorIf(index > 1, "Math::Vector2 - Subscript out of range.");
  return array[index];
}


////////// Unary Operators /////////////////////////////////////////////////////

Vector2 Vector2::operator-() const
{
  return Vector2(-x, -y);
}


////////// Binary Assignment Operators (reals) /////////////////////////////////

void Vector2::operator*=(float rhs)
{
  x *= rhs;
  y *= rhs;
}

void Vector2::operator/=(float rhs)
{
  ErrorIf(Math::DebugIsZero(rhs), "Math::Vector2 - Division by zero.");
  x /= rhs;
  y /= rhs;
}


////////// Binary Operators (reals) ////////////////////////////////////////////

Vector2 Vector2::operator*(float rhs) const
{
  return Vector2(x * rhs, y * rhs);
}

Vector2 Vector2::operator/(float rhs) const
{
  ErrorIf(Math::DebugIsZero(rhs), "Math::Vector2 - Division by zero.");
  return Vector2(x / rhs, y / rhs);
}


////////// Binary Assignment Operators (Vectors) ///////////////////////////////

void Vector2::operator+=(Vec2Param rhs)
{
  x += rhs.x;
  y += rhs.y;
}

void Vector2::operator-=(Vec2Param rhs)
{
  x -= rhs.x;
  y -= rhs.y;
}


////////// Binary Operators (Vectors) //////////////////////////////////////////

Vector2 Vector2::operator+(Vec2Param rhs) const
{
  return Vector2(x + rhs.x, y + rhs.y);
}

Vector2 Vector2::operator-(Vec2Param rhs) const
{
  return Vector2(x - rhs.x, y - rhs.y);
}


////////// Binary Vector Comparisons ///////////////////////////////////////////

bool Vector2::operator==(Vec2Param rhs) const
{
  return x == rhs.x && 
         y == rhs.y;
}

bool Vector2::operator!=(Vec2Param rhs) const
{
  return !(*this == rhs);
}

void Vector2::Set(float x_, float y_)
{
  x = x_;
  y = y_;
}

void Vector2::ZeroOut()
{
  array[0] = 0.0f;
  array[1] = 0.0f;
}

float Vector2::Dot(Vec2Param rhs) const
{
  return x * rhs.x + y * rhs.y;
}

Vector2 Vector2::operator/(Vec2Param rhs) const
{
  ErrorIf(rhs.x == 0.0f || rhs.y == 0.0f, 
          "Vector2 - Division by zero.");
  return Vector2(x / rhs.x, y / rhs.y);
}

void Vector2::operator*=(Vec2Param rhs)
{
  x *= rhs.x;
  y *= rhs.y;
}

void Vector2::operator/=(Vec2Param rhs)
{
  x /= rhs.x;
  y /= rhs.y;
}

Vector2 Vector2::operator*(Vec2Param rhs) const
{
  return Vector2(x * rhs.x, y * rhs.y);
}

float Vector2::Length() const
{
  return Sqrt(LengthSq());
}

float Vector2::LengthSq() const
{
  return Dot(*this);
}

float Vector2::Distance(Vec2Param otherPoint) const
{
  return (*this - otherPoint).Length();
}

float Vector2::DistanceSq(Vec2Param otherPoint) const
{
  return (*this - otherPoint).LengthSq();
}

Vector2 Vector2::Normalized() const
{
  Vector2 ret = *this;
  ret /= Length();
  return ret;
}

float Vector2::Normalize()
{
  float length = Length();
  *this /= length;
  return length;
}

float Vector2::AttemptNormalize()
{
  float lengthSq = LengthSq();
  if(Math::DebugIsZero(lengthSq) == false)
  {
    lengthSq = Sqrt(lengthSq);
    *this /= lengthSq;
  }
  return lengthSq;
}

Vector2 Vector2::AttemptNormalized() const
{
  Vector2 ret = *this;
  ret.AttemptNormalize();
  return ret;
}

Vec2Ref Vector2::Negate()
{
  (*this) *= -1.0f;
  return *this;
}

bool Vector2::Valid() const
{
  return IsValid(x) && IsValid(y);
}

void Vector2::Splat(float value)
{
  x = y = value;
}

Vector2 operator*(float lhs, Vec2Param rhs)
{
  return rhs * lhs;
}

float Distance(Vec2Param lhs, Vec2Param rhs)
{
  return (rhs - lhs).Length();
}

float DistanceSq(Vec2Param lhs, Vec2Param rhs)
{
  return (rhs - lhs).LengthSq();
}

float Dot(Vec2Param lhs, Vec2Param rhs)
{
  return lhs.Dot(rhs);
}

float Cross(Vec2Param lhs, Vec2Param rhs)
{
  return lhs.x * rhs.y - rhs.x * lhs.y;
}

float Length(Vec2Param vect)
{
  return vect.Length();
}

float LengthSq(Vec2Param vect)
{
  return vect.LengthSq();
}

Vector2 Normalized(Vec2Param vect)
{
  return vect.Normalized();
}

float Normalize(Vec2Ptr vect)
{
  ErrorIf(vect == nullptr, "Vector2 - Null pointer passed for vector.");
  return vect->Normalize();
}

float AttemptNormalize(Vec2Ptr vect)
{
  ErrorIf(vect == nullptr, "Vector2 - Null pointer passed for vector.");
  return vect->AttemptNormalize();
}

Vector2 AttemptNormalized(Vec2Param vec)
{
  return vec.AttemptNormalized();
}

void Negate(Vec2Ptr vec)
{
  ErrorIf(vec == nullptr, "Vector2 - Null pointer passed for vector.");
  *vec *= -1.0f;
}

Vector2 Negated(Vec2Param vec)
{
  return Vector2(-vec.x, -vec.y);
}

Vector2 Abs(Vec2Param vec)
{
  return Vector2(Math::Abs(vec.x), Math::Abs(vec.y));
}

Vector2 Min(Vec2Param lhs, Vec2Param rhs)
{
  return Vector2(Math::Min(lhs.x, rhs.x),
                 Math::Min(lhs.y, rhs.y));
}

Vector2 Max(Vec2Param lhs, Vec2Param rhs)
{
  return Vector2(Math::Max(lhs.x, rhs.x),
                 Math::Max(lhs.y, rhs.y));
}

Vector2 Lerp(Vec2Param start, Vec2Param end, float tValue)
{
  WarnIf(!Math::InRange(tValue, 0.0f, 1.0f), 
         "Vector2 - Interpolation value is not in the range of [0, 1]");
  return Vector2(start[0] + tValue * (end[0] - start[0]),
                 start[1] + tValue * (end[1] - start[1]));
}

void Clamp(Vec2Ptr vec, float min, float max)
{
  ErrorIf(vec == nullptr, "Null pointer passed into function.");
  Math::Clamp(vec->x, min, max);
  Math::Clamp(vec->y, min, max);
}

}// namespace Math
