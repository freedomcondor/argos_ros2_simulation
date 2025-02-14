#ifndef VECTOR3_H
#define VECTOR3_H

namespace swarmMathLib {
   class CVector3;
   class CQuaternion;
}

#include <array>
#include <vector>
#include <iostream>
#include <cmath>

namespace swarmMathLib {

   /**
    * A 3D vector class.
    */
   class CVector3 {
   
   public:

      /** The <em>x</em> axis */
      static const CVector3 X;

      /** The <em>y</em> axis */
      static const CVector3 Y;

      /** The <em>z</em> axis */
      static const CVector3 Z;

      /** The zero vector (0,0,0) */
      static const CVector3 ZERO;

   public:

      /**
       * Class constructor.
       * It initializes the vector to (0,0,0).
       * @see ZERO
       */
      CVector3() :
         m_fX(0.0),
         m_fY(0.0),
         m_fZ(0.0) {
      }

      /**
       * Class constructor.
       * It initializes the vector from Cartesian coordinates.
       * @param f_x The <em>x</em> coordinate.
       * @param f_y The <em>y</em> coordinate.
       * @param f_z The <em>z</em> coordinate.
       * @see Set()
       */
      CVector3(double f_x,
               double f_y,
               double f_z) :
         m_fX(f_x),
         m_fY(f_y),
         m_fZ(f_z) {
      }

      /**
       * Class constructor.
       * It initializes the vector from Cartesian coordinates.
       * @param arr_coordinates The coordinates.
       * @see Set()
       */
      CVector3(const std::array<double, 3>& arr_coordinates) {
         Set(arr_coordinates);
      }

      /**
       * Returns the <em>x</em> coordinate of this vector.
       * @return The <em>x</em> coordinate of this vector.
       */
      inline double GetX() const {
         return m_fX;
      }

      /**
       * Sets the <em>x</em> coordinate of this vector.
       * @param f_x The new <em>x</em> coordinate of this vector.
       */
      inline void SetX(const double f_x) {
         m_fX = f_x;
      }

      /**
       * Returns the <em>y</em> coordinate of this vector.
       * @return The <em>y</em> coordinate of this vector.
       */
      inline double GetY() const {
         return m_fY;
      }

      /**
       * Sets the <em>y</em> coordinate of this vector.
       * @param f_y The new <em>y</em> coordinate of this vector.
       */
      inline void SetY(const double f_y) {
         m_fY = f_y;
      }

      /**
       * Returns the <em>z</em> coordinate of this vector.
       * @return The <em>z</em> coordinate of this vector.
       */
      inline double GetZ() const {
         return m_fZ;
      }

      /**
       * Sets the <em>z</em> coordinate of this vector.
       * @param f_z The new <em>z</em> coordinate of this vector.
       */
      inline void SetZ(const double f_z) {
         m_fZ = f_z;
      }

      /**
       * Sets the vector contents from Cartesian coordinates.
       * @param f_x The new <em>x</em> coordinate of this vector.
       * @param f_y The new <em>y</em> coordinate of this vector.
       * @param f_z The new <em>z</em> coordinate of this vector.
       */
      inline void Set(const double f_x,
                      const double f_y,
                      const double f_z) {
         m_fX = f_x;
         m_fY = f_y;
         m_fZ = f_z;
      }

      /**
       * Sets the vector contents from Cartesian coordinates.
       * @param arr_coordinates The new coordinates.
       */
      inline void Set(const std::array<double, 3>& arr_coordinates) {
         m_fX = arr_coordinates[0];
         m_fY = arr_coordinates[1];
         m_fZ = arr_coordinates[2];
      }

      /**
       * Returns the square length of this vector.
       * @return The square length of this vector.
       */
      inline double SquareLength() const {
         return std::pow(m_fX, 2) + std::pow(m_fY, 2) + std::pow(m_fZ, 2);
      }

      /**
       * Returns the length of this vector.
       * @return The length of this vector.
       */
      inline double Length() const {
         return sqrt(SquareLength());
      }

      /**
       * Normalizes this vector.
       * After this method is called, the vector has length 1. If the vector
       * is (0,0,0), this call results in a division by zero error.
       * @return A reference to this vector.
       */
      inline CVector3& Normalize() {
         *this /= Length();
         return *this;
      }

      /**
       * Rotates this vector by the given quaternion.
       * @param c_quaternion The quaternion to use.
       * @return A reference to this vector.
       * @see CQuaternion
       */
      CVector3& Rotate(const CQuaternion& c_quaternion);

      /**
       * Returns the dot product between this vector and the passed one.
       * 向量点乘
       * @param c_vector3 The other vector.
       * @return The dot product between this vector and the passed one.
       */
      inline double DotProduct(const CVector3& c_vector3) const {
         return
            m_fX * c_vector3.m_fX +
            m_fY * c_vector3.m_fY +
            m_fZ * c_vector3.m_fZ;
      }

      /**
       * Calculates the cross product between this vector and the passed one.
       * 向量叉乘
       * This method modifies this vector.
       * @param c_vector3 The other vector.
       * @return A reference to this vector.
       */
      inline CVector3& CrossProduct(const CVector3& c_vector3) {
         double fNewX, fNewY, fNewZ;
         fNewX = m_fY * c_vector3.m_fZ - m_fZ * c_vector3.m_fY;
         fNewY = m_fZ * c_vector3.m_fX - m_fX * c_vector3.m_fZ;
         fNewZ = m_fX * c_vector3.m_fY - m_fY * c_vector3.m_fX;
         m_fX = fNewX;
         m_fY = fNewY;
         m_fZ = fNewZ;
         return *this;
      }

      /**
       * Negates this vector.
       * After this method is called, this vector contains (-<em>x</em>,-<em>y</em>,-<em>z</em>).
       * @return A reference to this vector.
       */
      inline CVector3& Negate() {
         m_fX = -m_fX;
         m_fY = -m_fY;
         m_fZ = -m_fZ;
         return *this;
      }

      /**
       * Returns a Cartesian coordinate of this vector.
       * This method returns the value by copy.
       * @param un_index The desired coordinate, with 0 mapping to <em>x</em>, 1 mapping to <em>y</em>, and 2 mapping to <em>z</em>.
       * @return A Cartesian coordinate of this vector.
       * @throws CARGoSException if the given index is out of bounds.
       */
      inline double operator[](int un_index) const {
         switch(un_index) {
            case 0: return m_fX;
            case 1: return m_fY;
            case 2: return m_fZ;
            default: throw "vector3 index out of bound";
         }
      }

      /**
       * Returns a Cartesian coordinate of this vector.
       * This method returns the value by reference.
       * @param un_index The desired coordinate, with 0 mapping to <em>x</em>, 1 mapping to <em>y</em>, and 2 mapping to <em>z</em>.
       * @return A Cartesian coordinate of this vector.
       * @throws CARGoSException if the given index is out of bounds.
       */
      inline double& operator[](int un_index) {
         switch(un_index) {
            case 0: return m_fX;
            case 1: return m_fY;
            case 2: return m_fZ;
            default: throw "vector3 index out of bound";
         }
      }

      /**
       * Returns <tt>true</tt> if this vector and the passed one are equal.
       * This method checks all the coordinates for equality.
       * @param c_vector3 The other vector.
       * @return <tt>true</tt> if this vector and the passed one are equal.
       */
      inline bool operator==(const CVector3& c_vector3) const {
         return m_fX == c_vector3.m_fX && m_fY == c_vector3.m_fY && m_fZ == c_vector3.m_fZ;
      }

      /**
       * Returns <tt>true</tt> if this vector and the passed one are not equal.
       * This method checks all the coordinates for equality.
       * @param c_vector3 The other vector.
       * @return <tt>true</tt> if this vector and the passed one are not equal.
       */
      inline bool operator!=(const CVector3& c_vector3) const {
         return !((*this) == c_vector3);
      }

      /**
       * Returns <tt>true</tt> if this vector is smaller than the passed one.
       * This method checks all the coordinates, and returns <tt>true</tt> only
       * if the condition is true for all of them.
       * @param c_vector3 The other vector.
       * @return <tt>true</tt> if this vector is smaller than the passed one.
       */
      inline bool operator<(const CVector3& c_vector3) const {
         return m_fX < c_vector3.m_fX && m_fY < c_vector3.m_fY && m_fZ < c_vector3.m_fZ;
      }

      /**
       * Returns <tt>true</tt> if this vector is smaller than or equal to the passed one.
       * This method checks all the coordinates, and returns <tt>true</tt> only
       * if the condition is true for all of them.
       * @param c_vector3 The other vector.
       * @return <tt>true</tt> if this vector is smaller than or equal to the passed one.
       */
      inline bool operator<=(const CVector3& c_vector3) const {
         return m_fX <= c_vector3.m_fX && m_fY <= c_vector3.m_fY && m_fZ <= c_vector3.m_fZ;
      }

      /**
       * Returns <tt>true</tt> if this vector is greater than the passed one.
       * This method checks all the coordinates, and returns <tt>true</tt> only
       * if the condition is true for all of them.
       * @param c_vector3 The other vector.
       * @return <tt>true</tt> if this vector is greater than the passed one.
       */
      inline bool operator>(const CVector3& c_vector3) const {
         return m_fX > c_vector3.m_fX && m_fY > c_vector3.m_fY && m_fZ > c_vector3.m_fZ;
      }

      /**
       * Returns <tt>true</tt> if this vector is greater than or equal to the passed one.
       * This method checks all the coordinates, and returns <tt>true</tt> only
       * if the condition is true for all of them.
       * @param c_vector3 The other vector.
       * @return <tt>true</tt> if this vector is greater than or equal to the passed one.
       */
      inline bool operator>=(const CVector3& c_vector3) const {
         return m_fX >= c_vector3.m_fX && m_fY >= c_vector3.m_fY && m_fZ >= c_vector3.m_fZ;
      }

      /**
       * Returns a negated copy of this vector.
       * @return A negated copy of this vector.
       */
      inline CVector3 operator-() const {
         return CVector3(-m_fX, -m_fY, -m_fZ);
      }

      /**
       * Sums the passed vector to this vector.
       * @param c_vector3 The other vector.
       * @returns A reference to this vector.
       */
      inline CVector3& operator+=(const CVector3& c_vector3) {
         m_fX += c_vector3.m_fX;
         m_fY += c_vector3.m_fY;
         m_fZ += c_vector3.m_fZ;
         return *this;
      }

      /**
       * Subtracts the passed vector from this vector.
       * @param c_vector3 The other vector.
       * @returns A reference to this vector.
       */
      inline CVector3& operator-=(const CVector3& c_vector3) {
         m_fX -= c_vector3.m_fX;
         m_fY -= c_vector3.m_fY;
         m_fZ -= c_vector3.m_fZ;
         return *this;
      }

      /**
       * Multiplies this vector by the given value.
       * @param f_value The wanted value.
       * @return A reference to this vector.
       */
      inline CVector3& operator*=(double f_value) {
         m_fX *= f_value;
         m_fY *= f_value;
         m_fZ *= f_value;
         return *this;
      }

      /**
       * Divides this vector by the given value.
       * @param f_value The wanted value.
       * @return A reference to this vector.
       */
      inline CVector3& operator/=(double f_value) {
         m_fX /= f_value;
         m_fY /= f_value;
         m_fZ /= f_value;
         return *this;
      }

      /**
       * Returns a new vector containing the sum between this vector and the passed one.
       * @param c_vector3 The other vector.
       * @return A new vector containing the sum between this vector and the passed one.
       */
      inline CVector3 operator+(const CVector3& c_vector3) const {
         CVector3 cResult(*this);
         cResult += c_vector3;
         return cResult;
      }

      /**
       * Returns a new vector containing the subtraction between this vector and the passed one.
       * @param c_vector3 The other vector.
       * @return A new vector containing the subtraction between this vector and the passed one.
       */
      inline CVector3 operator-(const CVector3& c_vector3) const {
         CVector3 cResult(*this);
         cResult -= c_vector3;
         return cResult;
      }

      /**
       * Returns a new vector containing the multiplication between this vector and the passed value.
       * @param f_value The wanted value.
       * @return A new vector containing the multiplication between this vector and the passed value.
       */
      inline CVector3 operator*(double f_value) const {
         CVector3 cResult(*this);
         cResult *= f_value;
         return cResult;
      }

      /**
       * Returns a new vector containing the division between this vector and the passed value.
       * @param f_value The wanted value.
       * @return A new vector containing the division between this vector and the passed value.
       */
      inline CVector3 operator/(const double f_value) const {
         CVector3 cResult(*this);
         cResult /= f_value;
         return cResult;
      }

      /**
       * Returns a new vector containing the multiplication between the passed value and the passed vector.
       * @param f_value The value.
       * @param c_vector3 The vector.
       * @return A new vector containing the multiplication between the passed value and the passed vector.
       */
      inline friend CVector3 operator*(double f_value,
                                       const CVector3& c_vector3) {
         return c_vector3 * f_value;
      }

      /**
       * Serializes the contents of the passed vector onto a stream.
       * @param c_os The stream.
       * @param c_vector The vector.
       * @return The new state of the stream.
       */
      inline friend std::ostream& operator<<(std::ostream& c_os,
                                             const CVector3& c_vector3) {
         c_os << c_vector3.m_fX << ","
              << c_vector3.m_fY << ","
              << c_vector3.m_fZ;
         return c_os;
      }

   private:
      
      /** The Cartesian <em>x</em> coordinate */
      double m_fX;

      /** The Cartesian <em>y</em> coordinate */
      double m_fY;

      /** The Cartesian <em>z</em> coordinate */
      double m_fZ;

   };

   /****************************************/
   /****************************************/

   /** 
    * Computes the square distance between the passed vectors.
    * @param c_v1 The first vector
    * @param c_v2 The second vector
    * @return The square distance between the passed vectors
    */
   inline double SquareDistanceBetweenVector3s(const CVector3& c_v1, const CVector3& c_v2) {
      return (c_v1 - c_v2).SquareLength();
   }

   /** 
    * Computes the distance between the passed vectors.
    * @param c_v1 The first vector
    * @param c_v2 The second vector
    * @return The distance between the passed vectors
    */
   inline double DistanceBetweenVector3s(const CVector3& c_v1, const CVector3& c_v2) {
      return (c_v1 - c_v2).Length();
   }

   /** 
    * Calculates the average point of a std::vector of CVector3s
    * @param c_v1 The first vector
    * @param c_v2 The second vector
    * @return The distance between the passed vectors
    */
   inline CVector3 AverageVector3s(const std::vector<CVector3>& vector3s) {
      if (vector3s.size() == 0) return CVector3(0,0,0);
      CVector3 cAverageVector = CVector3(0,0,0);
      for (const CVector3& vec : vector3s)
         cAverageVector += vec;
      cAverageVector /= vector3s.size();
      return cAverageVector;
   }

/****************************************/
/****************************************/

}

#endif
