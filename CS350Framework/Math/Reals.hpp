///////////////////////////////////////////////////////////////////////////////
/// 
/// Authors: Joshua Davis, Benjamin Strukus
/// Copyright 2010-2012, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#pragma once

namespace Math
{

const unsigned cX = 0;
const unsigned cY = 1;
const unsigned cZ = 2;
const unsigned cW = 3;

const float cPi = 3.1415926535897932384626433832795f;
const float cTwoPi = 2.0f * cPi;

float PositiveMax();
float PositiveValueClosestToZero();
float NegativeValueClosestToZero();
float NegativeMin();
bool IsNegative(float number);
bool IsPositive(float number);
float GetSign(float val);
float Round(float val);
float Ceil(float val);
float Floor(float val);
float Abs(float val);
int Abs(int val);
float Sqrt(float val);
float Rsqrt(float val);
float Sq(float sqrt);
float Pow(float base, float exp);
float Log(float val);
float Log10(float val);
float FMod(float dividend, float divisor);
float Cos(float val);
float Sin(float val);
float Tan(float angle);
float ArcCos(float angle);
float ArcSin(float angle);
float ArcTan(float angle);
float ArcTan2(float y, float x);
float RadToDeg(float radians);
float DegToRad(float degrees);
bool LessThan(float lhs, float rhs);
bool LessThanOrEqual(float lhs, float rhs);
bool GreaterThan(float lhs, float rhs);
bool GreaterThanOrEqual(float lhs, float rhs);
bool IsValid(float val);
float DebugEpsilon();
bool DebugIsZero(float val);

template <typename T>
inline T Max(const T lhs, const T rhs)
{
  return lhs > rhs ? lhs : rhs;
}

template <typename T>
inline T Min(const T lhs, const T rhs) 
{
  return lhs > rhs ? rhs : lhs;
}

template <typename T>
inline T Clamp(const T x, const T xMin, const T xMax)
{
  return Max(xMin, Min(x, xMax));
}

template <typename T>
inline T Clamp(const T value) 
{
  return Clamp(value, T(0), T(1));
}

template <typename T>
inline T ClampIfClose(const T x, const T xMin, const T xMax, const T epsilon)
{
  float value = x < xMin && x > (xMin - epsilon) ? xMin : x;
  value = value > xMax && value < (xMax + epsilon) ? xMax : value;
  return value;
}

///Checks to see if x is within the interval of [xMin, xMax]
template <typename T>
inline bool InRange(const T x, const T xMin, const T xMax)
{
    return ((xMin <= x) && (x <= xMax));
}

///Checks to see if x is within the interval of (xMin, xMax)
template <typename T>
inline bool InBounds(const T x, const T xMin, const T xMax)
{
    return ((xMin < x) && (x < xMax));
}

template <typename T>
inline void Swap(T& a, T& b)
{
  T temp(a);
  a = b;
  b = temp;
}

template <typename Data, typename T>
inline Data Lerp(const Data& start, const Data& end, T interpolationValue)
{
  return (T(1.0) - interpolationValue) * start + interpolationValue * end;
}

}// namespace Math
