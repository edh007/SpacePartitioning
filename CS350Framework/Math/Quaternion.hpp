///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis, Benjamin Strukus
/// Copyright 2010-2012, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "Reals.hpp"
#include "Vector3.hpp"
#include "Vector4.hpp"
#include "Matrix3.hpp"
#include "Matrix4.hpp"

namespace Math
{

///Forward declaration
struct Matrix3;
typedef const Matrix3& Mat3Param;
typedef Matrix3& Mat3Ref;

///Forward declaration
struct Matrix4;
typedef const Matrix4& Mat4Param;
typedef Matrix4& Mat4Ref;

struct Quaternion;
typedef const Quaternion& QuatParam;
typedef Quaternion& QuatRef;
typedef Quaternion* QuatPtr;
typedef Quaternion Quat;

//------------------------------------------------------------------- Quaternion
struct Quaternion
{
  Quaternion() {};
  explicit Quaternion(float x, float y, float z, float w);

  void operator=(QuatParam rhs);
  void operator+=(QuatParam rhs);
  void operator-=(QuatParam rhs);
  void operator*=(QuatParam rhs);
  void operator*=(float rhs);
  void operator/=(float rhs);

  Quaternion operator-() const;

  Quaternion operator*(QuatParam rhs) const;
  Quaternion operator+(QuatParam rhs) const;
  Quaternion operator-(QuatParam rhs) const;
  Quaternion operator*(float rhs) const;
  Quaternion operator/(float rhs) const;

  bool operator==(QuatParam rhs) const;
  bool operator!=(QuatParam rhs) const;

  float& operator[](unsigned index);
  float operator[](unsigned index) const;

  void Set(float x, float y, float z, float w);

  float Normalize();
  Quaternion Normalized() const;
  float Dot(QuatParam rhs) const;
  float Length() const;
  float LengthSq() const;

  void Conjugate();
  Quaternion Conjugated() const;
  void Invert();
  Quaternion Inverted() const;
  Quaternion Exponent() const;
  Quaternion Logarithm() const;
  void RotateVector(Vec3Ptr vector);
  Vector3 RotatedVector(Vec3Param vector) const;
  void ZeroOut();

  bool Valid() const;

  Vector3& V3();
  Vector4& V4();
  const Vector3& V3() const;
  const Vector4& V4() const;

  float x, y, z, w;
  static const Quaternion cIdentity;
};

Quaternion operator*(float lhs, QuatParam rhs);

void Normalize(QuatPtr quaternion);
Quaternion Normalized(QuatParam quaternion);
float Dot(QuatParam lhs, QuatParam rhs);
float Length(QuatParam quaternion);
float LengthSq(QuatParam quaternion);
Quaternion Lerp(QuatParam start, QuatParam end, float tValue);

}// namespace Math
