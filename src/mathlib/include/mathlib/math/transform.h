#ifndef CTRANSFORM_H
#define CTRANSFORM_H

#define _USE_MATH_DEFINES  // for std::M_PI

#include <mathlib/math/vector3.h>
#include <mathlib/math/quaternion.h>

namespace swarmMathLib {

   class CTransform {

   public:
      CTransform() {
         m_Position = CVector3();
         m_Orientation = CQuaternion();
      }

      CTransform(CVector3 _position, CQuaternion _orientation) {
         m_Position = _position;
         m_Orientation = _orientation;
      }

      inline CTransform& operator*=(const CTransform& c_transform) {
         this->m_Position = this->m_Position + CVector3(c_transform.m_Position).Rotate(this->m_Orientation);
         this->m_Orientation = this->m_Orientation * c_transform.m_Orientation;
         return *this;
      }

      inline CTransform operator*(const CTransform& c_transform) const {
         CTransform result;
         result.m_Position = this->m_Position + CVector3(c_transform.m_Position).Rotate(this->m_Orientation);
         result.m_Orientation = this->m_Orientation * c_transform.m_Orientation;
         return result;
      }

      inline CVector3 operator*(const CVector3& c_vector) const {
         CVector3 result;
         result = this->m_Position + CVector3(c_vector).Rotate(this->m_Orientation);
         return result;
      }

      /** A x B = C
       *  C - A = B
       */
      inline CTransform operator-(const CTransform& c_transform) const {
         CTransform result;
         result = c_transform.Inverse() * (*this);
         return result;
      }

      inline CTransform Inverse() const {
         CTransform result;
         result.m_Orientation = m_Orientation.Inverse();
         result.m_Position = CVector3(-m_Position).Rotate(result.m_Orientation);
         return result;
      }

      /**
       * Serializes the contents of the passed quaternion into a stream as Euler angles
       * in the Z,Y,X format in degrees.
       * @param c_os The stream.
       * @param c_transform The quaternion.
       * @return The new state of the stream.
       */
      inline friend std::ostream& operator<<(std::ostream& c_os, const CTransform& c_transform) {
         c_os << "("
              << c_transform.m_Position
              << "),("
              << c_transform.m_Orientation
              << ")";
         return c_os;
      }
   
   public :
      CVector3 m_Position;
      CQuaternion m_Orientation;
   };

}

#endif
