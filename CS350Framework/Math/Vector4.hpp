///////////////////////////////////////////////////////////////////////////////
/// 
/// Authors: Joshua Davis, Benjamin Strukus
/// Copyright 2010-2012, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Reals.hpp"

namespace Math
{

struct Vector4;
typedef const Vector4& Vec4Param;
typedef Vector4& Vec4Ref;
typedef Vector4* Vec4Ptr;

//--------------------------------------------------------------------- Vector 4
struct Vector4
{
  Vector4();
  explicit Vector4(float x, float y, float z, float w);
  explicit Vector4(const float* data);
  //Splat all elements
  explicit Vector4(float xyzw);

  float* ToFloats();

  float& operator[](unsigned index);
  float operator[](unsigned index) const;

  //Unary Operators
  Vector4 operator-() const;

  //Binary Assignment Operators (reals)
  void operator*=(float rhs);
  void operator/=(float rhs);

  //Binary Operators (Reals)
  Vector4 operator*(float rhs) const;
  Vector4 operator/(float rhs) const;

  //Binary Assignment Operators (vectors)
  void operator+=(Vec4Param rhs);
  void operator-=(Vec4Param rhs);
  void operator*=(Vec4Param rhs);
  void operator/=(Vec4Param rhs);

  //Binary Operators (vectors)
  Vector4 operator+(Vec4Param rhs) const;
  Vector4 operator-(Vec4Param rhs) const;

  //Binary Vector Comparisons
  bool operator==(Vec4Param rhs) const;
  bool operator!=(Vec4Param rhs) const;

  //Vector component wise multiply and divide
  Vector4 operator*(Vec4Param rhs) const;
  Vector4 operator/(Vec4Param rhs) const;

  void Set(float x, float y, float z, float w);

  ///Set all of the values of the vector to the passed in value.
  void Splat(float xyzw);

  void ZeroOut();

  float Dot(Vec4Param rhs) const;
  float Length() const;
  float LengthSq() const;
  float Distance(Vec4Param otherPoint) const;
  float DistanceSq(Vec4Param otherPoint) const;
  Vector4 Normalized() const;
  float Normalize();
  float AttemptNormalize();
  ///Returns a normalized copy of this vector if possible.
  Vector4 AttemptNormalized() const;
  Vec4Ref Negate();
  bool Valid() const;

  union
  {
    struct  
    {
      float x, y, z, w;
    };
    float array[4];
  };

  static const Vector4 cZero;
  static const Vector4 cXAxis;
  static const Vector4 cYAxis;
  static const Vector4 cZAxis;
  static const Vector4 cWAxis;

};

Vector4 operator*(float lhs, Vec4Param rhs);
float Dot(Vec4Param lhs, Vec4Param rhs);
float Length(Vec4Param vect);
float LengthSq(Vec4Param vect);
float Distance(Vec4Param lhs, Vec4Param rhs);
float DistanceSq(Vec4Param lhs, Vec4Param rhs);
Vector4 Normalized(Vec4Param vect);
float Normalize(Vec4Ptr vect);
float AttemptNormalize(Vec4Ptr vect);
Vector4 AttemptNormalized(Vec4Param vect);
void Negate(Vec4Ptr vec);
Vector4 Negated(Vec4Param vec);
Vector4 Abs(Vec4Param vec);
Vector4 Min(Vec4Param lhs, Vec4Param rhs);
Vector4 Max(Vec4Param lhs, Vec4Param rhs);
Vector4 Lerp(Vec4Param start, Vec4Param end, float tValue);

}// namespace Math
