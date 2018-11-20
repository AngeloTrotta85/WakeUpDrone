/***************************************************************************
 * file:        MyCoord.h
 *
 * author:      Christian Frank
 *
 * copyright:   (C) 2004 Telecommunication Networks Group (TKN) at
 *              Technische Universitaet Berlin, Germany.
 *
 *              This program is free software; you can redistribute it
 *              and/or modify it under the terms of the GNU General Public
 *              License as published by the Free Software Foundation; either
 *              version 2 of the License, or (at your option) any later
 *              version.
 *              For further information see file COPYING
 *              in the top level directory
 ***************************************************************************
 * part of:     framework implementation developed by tkn
 **************************************************************************/

#ifndef __INET_COORD_H
#define __INET_COORD_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <sstream>
#include <algorithm>    // std::find
#include <vector>       // std::vector
#include <list>       // std::list
#include <cstdlib>
#include <ctime>
#include <math.h>

/* Constant for comparing doubles. Two doubles at most epsilon apart
   are declared equal.*/
#ifndef EPSILON
#define EPSILON     0.0001
#endif // ifndef EPSILON

#define qNaN        std::numeric_limits<double>::quiet_NaN()
#define sNaN        std::numeric_limits<double>::signaling_NaN()
#define NaN         qNaN

/**
 * @brief Class for storing 3D coordinates.
 *
 * Some comparison and basic arithmetic operators are implemented.
 *
 * @ingroup utils
 * @author Christian Frank
 */
class MyCoord
{
  public:
    /** @brief Constant with all values set to 0. */
    static const MyCoord NIL;
    static const MyCoord ZERO;
    static const MyCoord X_AXIS;
    static const MyCoord Y_AXIS;
    static const MyCoord Z_AXIS;

  public:
    /** @name x, y and z coordinate of the position. */
    /*@{*/
    double x;
    double y;
    double z;
    /*@}*/

  private:
    void copy(const MyCoord& other) { x = other.x; y = other.y; z = other.z; }

  public:
    /**
     * Tests whether two doubles are close enough to be declared equal.
     * Returns true if parameters are at most epsilon apart, false
     * otherwise
     */
    static bool close(double one, double two) { return fabs(one - two) < EPSILON; }

    /**
     * Returns the remainder r on division of dividend a by divisor n,
     * using floored division. The remainder r has the same sign as the divisor n.
     */
    static double modulo(double a, double n) { return a - n * floor(a / n); }


    /** @brief Default constructor. */
    MyCoord()
        : x(0.0), y(0.0), z(0.0) {}

    /** @brief Initializes a coordinate. */
    MyCoord(double x, double y, double z = 0.0)
        : x(x), y(y), z(z) {}

    /** @brief Initializes coordinate from other coordinate. */
    MyCoord(const MyCoord& other) { copy(other); }

    /** @brief Returns a string with the value of the coordinate. */
    std::string info() const;

    /** @brief Adds two coordinate vectors. */
    friend MyCoord operator+(const MyCoord& a, const MyCoord& b)
    {
        MyCoord tmp(a);
        tmp += b;
        return tmp;
    }

    /** @brief Subtracts two coordinate vectors. */
    friend MyCoord operator-(const MyCoord& a, const MyCoord& b)
    {
        MyCoord tmp(a);
        tmp -= b;
        return tmp;
    }

    /** @brief Multiplies a coordinate vector by a real number. */
    friend MyCoord operator*(const MyCoord& a, double f)
    {
        MyCoord tmp(a);
        tmp *= f;
        return tmp;
    }

    /** @brief Divides a coordinate vector by a real number. */
    friend MyCoord operator/(const MyCoord& a, double f)
    {
        MyCoord tmp(a);
        tmp /= f;
        return tmp;
    }

    /**
     * @brief Multiplies this coordinate vector by a real number.
     */
    MyCoord& operator*=(double f)
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }

    /**
     * @brief Divides this coordinate vector by a real number.
     */
    MyCoord& operator/=(double f)
    {
        x /= f;
        y /= f;
        z /= f;
        return *this;
    }

    /**
     * @brief Adds coordinate vector 'a' to this.
     */
    MyCoord& operator+=(const MyCoord& a)
    {
        x += a.x;
        y += a.y;
        z += a.z;
        return *this;
    }

    /**
     * @brief Dot product
     */
    float operator*(const MyCoord& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }

    /**
     * @brief Cross product
     */
    MyCoord operator%(const MyCoord& v) const
    {
        return MyCoord(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    /**
     * @brief Assigns coordinate vector 'other' to this.
     *
     * This operator can change the dimension of the coordinate.
     */
    MyCoord& operator=(const MyCoord& other)
    {
        if (this == &other)
            return *this;
        copy(other);
        return *this;
    }

    /**
     * @brief Subtracts coordinate vector 'a' from this.
     */
    MyCoord& operator-=(const MyCoord& a)
    {
        x -= a.x;
        y -= a.y;
        z -= a.z;
        return *this;
    }

    /**
     * @brief Tests whether two coordinate vectors are equal.
     *
     * Because coordinates are of type double, this is done through the
     * math::close function.
     */
    friend bool operator==(const MyCoord& a, const MyCoord& b)
    {
        return close(a.x, b.x) && close(a.y, b.y) && close(a.z, b.z);
    }

    /**
     * @brief Tests whether two coordinate vectors are not equal.
     *
     * Negation of the operator==.
     */
    friend bool operator!=(const MyCoord& a, const MyCoord& b) { return !(a == b); }

    /**
     * @brief Returns the distance to MyCoord 'a'.
     */
    double distance(const MyCoord& a) const
    {
        MyCoord dist(*this - a);
        return dist.length();
    }

    /**
     * @brief Returns distance^2 to MyCoord 'a' (omits calling square root).
     */
    double sqrdist(const MyCoord& a) const
    {
        MyCoord dist(*this - a);
        return dist.squareLength();
    }

    /**
     * @brief Returns the squared distance on a torus of this to MyCoord 'b' (omits calling square root).
     */
    double sqrTorusDist(const MyCoord& b, const MyCoord& size) const;

    /**
     * @brief Returns the square of the length of this Coords position vector.
     */
    double squareLength() const { return x * x + y * y + z * z; }

    /**
     * @brief Returns the length of this Coords position vector.
     */
    double length() const { return sqrt(squareLength()); }

    /**
     * @brief Updates the length of this position vector to be 1.
     */
    void normalize() { *this /= length(); }

    /**
     * @brief Checks if this coordinate is inside a specified rectangle.
     *
     * @param lowerBound The upper bound of the rectangle.
     * @param upperBound The lower bound of the rectangle.
     */
    bool isInBoundary(const MyCoord& lowerBound, const MyCoord& upperBound) const
    {
        return lowerBound.x <= x && x <= upperBound.x &&
               lowerBound.y <= y && y <= upperBound.y &&
               lowerBound.z <= z && z <= upperBound.z;
    }

    bool isNil() const
    {
        return this == &NIL;
    }

    /**
     * Returns true if this coordinate is unspecified.
     */
    bool isUnspecified() const
    {
        return std::isnan(x) && std::isnan(y) && std::isnan(z);
    }

    /**
     * @brief Returns the minimal coordinates.
     */
    MyCoord min(const MyCoord& a)
    {
        return MyCoord(this->x < a.x ? this->x : a.x,
                this->y < a.y ? this->y : a.y,
                this->z < a.z ? this->z : a.z);
    }

    /**
     * @brief Returns the maximal coordinates.
     */
    MyCoord max(const MyCoord& a)
    {
        return MyCoord(this->x > a.x ? this->x : a.x,
                this->y > a.y ? this->y : a.y,
                this->z > a.z ? this->z : a.z);
    }

    /**
     * @brief Returns the angle between the two vectors.
     */
    double angle(const MyCoord& a)
    {
        return acos(*this * a / length() / a.length());
    }
};

inline std::ostream& operator<<(std::ostream& os, const MyCoord& MyCoord)
{
    return os << "(" << MyCoord.x << ", " << MyCoord.y << ", " << MyCoord.z << ")";
}

inline std::string MyCoord::info() const
{
    std::stringstream os;
    os << *this;
    return os.str();
}


#endif // ifndef __INET_COORD_H

