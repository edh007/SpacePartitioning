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

struct Vector2;
typedef const Vector2& Vec2Param;
typedef Vector2& Vec2Ref;
typedef Vector2* Vec2Ptr;

//---------------------------------------------------------------------Vector2
///Two-dimensional vector.
struct Vector2
{
  Vector2();
  explicit Vector2(float x, float y);
  explicit Vector2(const float* data);

  ///Set all of the values of this vector at once.
  void Set(float x, float y);
  void Splat(float value);
  ///Set all of this vector's elements to 0.
  void ZeroOut();
  float* ToFloats();

  float& operator[](unsigned index);
  float operator[](unsigned index) const;

  //Unary Operators
  Vector2 operator-() const;

  //Binary Assignment Operators (reals)
  void operator*=(float rhs);
  void operator/=(float rhs);

  //Binary Operators (Reals)
  Vector2 operator*(float rhs) const;
  Vector2 operator/(float rhs) const;

  //Binary Assignment Operators (vectors)
  void operator+=(Vec2Param rhs);
  void operator-=(Vec2Param rhs);

  //Binary Operators (vectors)
  Vector2 operator+(Vec2Param rhs) const;
  Vector2 operator-(Vec2Param rhs) const;

  //Binary Vector Comparisons
  bool operator==(Vec2Param rhs) const;
  bool operator!=(Vec2Param rhs) const;

  //Vector component-wise multiply and divide
  Vector2 operator*(Vec2Param rhs) const;
  Vector2 operator/(Vec2Param rhs) const;

  ///Component-wise assignment multiplication
  void operator*=(Vec2Param rhs);
  void operator/=(Vec2Param rhs);

  ///Compute the dot product of this vector with the given vector.
  float Dot(Vec2Param rhs) const;

  ///Get the length of this vector.
  float Length() const;

  ///Get the squared length of this vector.
  float LengthSq() const;

  ///Get the distance from the given point
  float Distance(Vec2Param otherPoint) const;

  ///Get the squared distance from the given point
  float DistanceSq(Vec2Param otherPoint) const;

  ///Calculate and return a unit-length copy of this vector.
  Vector2 Normalized() const;

  ///Make this vector have a length of 1, returns the original length.
  float Normalize();

  ///Attempt to give this vector a length of 1, but checks if it's possible.
  ///Instead of crashing, will return 0 if the vector was not able to be 
  ///normalized.
  float AttemptNormalize();

  ///Returns a normalized copy of this vector if possible.
  Vector2 AttemptNormalized() const;

  ///Flips this vector so it's pointing in the opposite direction.
  Vec2Ref Negate();

  ///Checks to see if the values of this vector's elements are usable.
  bool Valid() const;

  union
  {
    struct  
    {
      float x, y;
    };
    float array[2];
  };

  static const Vector2 cZero;
  static const Vector2 cXAxis;
  static const Vector2 cYAxis;
};

Vector2 operator*(float lhs, Vec2Param rhs);

///Compute the distance between two given points.
float Distance(Vec2Param lhs, Vec2Param rhs);
float DistanceSq(Vec2Param lhs, Vec2Param rhs);

///Compute the dot product of the two given vectors.
float Dot(Vec2Param lhs, Vec2Param rhs);

///Compute the 2d cross product of the two given vectors.
float Cross(Vec2Param lhs, Vec2Param rhs);

///Get the length of the given vector.
float Length(Vec2Param vec);

///Get the squared length of the given vector.
float LengthSq(Vec2Param vec);

///Calculate and return a unit-length copy of the given vector.
Vector2 Normalized(Vec2Param vec);

///Make the given vector have a length of 1, returns the original length.
float Normalize(Vec2Ptr vec);

///Attempt to give the given vector a length of 1, but checks if it's possible.
///Instead of crashing, this will return 0 if the vector was not able to be 
///normalized.
float AttemptNormalize(Vec2Ptr vec);

///Returns a normalized copy of this vector if possible.
Vector2 AttemptNormalized(Vec2Param vec);

///Flips the given vector so it's pointing in the opposite direction.
void Negate(Vec2Ptr vec);

///Returns a vector pointing in the opposite direction of the given vector.
Vector2 Negated(Vec2Param vec);

///Returns a vector with absolute valued elements of the given vector.
Vector2 Abs(Vec2Param vec);

///Returns the component-wise minimum vector of the two vectors.
Vector2 Min(Vec2Param lhs, Vec2Param rhs);

///Returns the component-wise maximum vector of the two vectors.
Vector2 Max(Vec2Param lhs, Vec2Param rhs);

///Linearly interpolate between the two vectors, the t-value is restricted to 
///the range [0, 1].
Vector2 Lerp(Vec2Param start, Vec2Param end, float tValue);

///Clamps the values of the elements of the given vector to the range [min, max]
void Clamp(Vec2Ptr vec, float min, float max);

}// namespace Math
