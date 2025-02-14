#ifndef CQUATERNION_H
#define CQUATERNION_H

#define _USE_MATH_DEFINES  // for std::M_PI

#include <mathlib/math/vector3.h>

namespace swarmMathLib {

   class CQuaternion {

   public:
      CQuaternion() {
         m_fValues[0] = 1.0;
         m_fValues[1] = 0.0;
         m_fValues[2] = 0.0;
         m_fValues[3] = 0.0;
      }

      CQuaternion(const CQuaternion& c_quaternion) {
         *this = c_quaternion;
      }

      CQuaternion(double f_double,
                  double f_img1,
                  double f_img2,
                  double f_img3) {
         m_fValues[0] = f_double;
         m_fValues[1] = f_img1;
         m_fValues[2] = f_img2;
         m_fValues[3] = f_img3;
      }

      CQuaternion(const double& f_radians,
                  const CVector3& c_vector3) {
         FromAngleAxis(f_radians, c_vector3);
      }

      inline CQuaternion(const CVector3& c_vector1,
                         const CVector3& c_vector2) {
         BetweenTwoVectors(c_vector1, c_vector2);
      }

      inline double GetW() const {
         return m_fValues[0];
      }

      inline double GetX() const {
         return m_fValues[1];
      }

      inline double GetY() const {
         return m_fValues[2];
      }

      inline double GetZ() const {
         return m_fValues[3];
      }

      inline void SetW(double f_w) {
         m_fValues[0] = f_w;
      }

      inline void SetX(double f_x) {
         m_fValues[1] = f_x;
      }

      inline void SetY(double f_y) {
         m_fValues[2] = f_y;
      }

      inline void SetZ(double f_z) {
         m_fValues[3] = f_z;
      }

      inline void Set(double f_w,
                      double f_x,
                      double f_y,
                      double f_z) {
         m_fValues[0] = f_w;
         m_fValues[1] = f_x;
         m_fValues[2] = f_y;
         m_fValues[3] = f_z;
      }

      inline CQuaternion Conjugate() const {
         return CQuaternion(m_fValues[0],
                            -m_fValues[1],
                            -m_fValues[2],
                            -m_fValues[3]);
      }

      inline CQuaternion Inverse() const {
         return CQuaternion(m_fValues[0],
                            -m_fValues[1],
                            -m_fValues[2],
                            -m_fValues[3]);
      }

      inline double Length() const {
         return ::sqrt(SquareLength());
      }

      inline double SquareLength() const {
         return
            std::pow(m_fValues[0], 2) +
            std::pow(m_fValues[1], 2) +
            std::pow(m_fValues[2], 2) +
            std::pow(m_fValues[3], 2);
      }

      inline CQuaternion& Normalize() {
         double fInvLength = 1.0 / Length();
         m_fValues[0] *= fInvLength;
         m_fValues[1] *= fInvLength;
         m_fValues[2] *= fInvLength;
         m_fValues[3] *= fInvLength;
         return *this;
      }

      /**
       * 给转动角度和转轴，生成Quaternion
       */
      inline CQuaternion& FromAngleAxis(const double f_angle,
                                        const CVector3& c_vector) {
         double cHalfAngle = f_angle * 0.5;
         double fSin, fCos;
#ifdef ARGOS_SINCOS
         SinCos(cHalfAngle, fSin, fCos);
#else
         fSin = std::sin(cHalfAngle);
         fCos = std::cos(cHalfAngle);
#endif
         m_fValues[0] = fCos;
         m_fValues[1] = c_vector.GetX() * fSin;
         m_fValues[2] = c_vector.GetY() * fSin;
         m_fValues[3] = c_vector.GetZ() * fSin;
         return *this;
      }

      /**
       * 从Quaternion生成转动角度和转轴
       */
      inline void ToAngleAxis(double& f_angle,
                              CVector3& c_vector) const {
         double fSquareLength =
            std::pow(m_fValues[1], 2) +
            std::pow(m_fValues[2], 2) +
            std::pow(m_fValues[3], 2);
         if(fSquareLength > 0.0f) {
            f_angle = 2.0f * std::acos(m_fValues[0]);
            double fInvLength = 1.0f / ::sqrt(fSquareLength);
            c_vector.Set(m_fValues[1] * fInvLength,
                         m_fValues[2] * fInvLength,
                         m_fValues[3] * fInvLength);
         }
         else {
            /* By default, to ease the support of robot orientation, no rotation refers to the Z axis */
            f_angle = 0;
            c_vector = CVector3::Z;
         }
      }

      /**
       * 给欧拉角，生成Quaternion
       * 注意欧拉角的顺序，先z
       */
      inline CQuaternion& FromEulerAngles(double& f_z_angle,
                                          double& f_y_angle,
                                          double& f_x_angle) {
         (*this) = CQuaternion(f_x_angle, CVector3::X) *
            CQuaternion(f_y_angle, CVector3::Y) *
            CQuaternion(f_z_angle, CVector3::Z);
         return (*this);
      }

      inline void ToEulerAngles(double& f_z_angle,
                                double& f_y_angle,
                                double& f_x_angle) const {
         /* With the ZYX convention, gimbal lock happens when
            cos(y_angle) = 0, that is when y_angle = +- pi/2
            In this condition, the Z and X axis overlap and we
            lose one degree of freedom. It's a problem of the
            Euler representation of rotations that is not
            present when we deal with quaternions.
            For reasons of speed, we consider gimbal lock
            happened when fTest > 0.499 and when fTest < -0.499.
         */
         /* Computed to understand if we have gimbal lock or not */
         double fTest =
            m_fValues[1] * m_fValues[3] +
            m_fValues[0] * m_fValues[2];

         if(fTest > 0.49999f) {
            /* Gimbal lock */
            f_x_angle = 0;
            f_y_angle = M_PI_2; // pi / 2
            f_z_angle = std::atan2(2.0f * (m_fValues[1] * m_fValues[2] + m_fValues[0] * m_fValues[3]),
                                   1.0f - 2.0f * (m_fValues[1] * m_fValues[1] + m_fValues[3] * m_fValues[3]));
         }
         else if(fTest < -0.49999f) {
            /* Gimbal lock */
            f_x_angle = 0;
            f_y_angle = -M_PI_2;
            f_z_angle = std::atan2(2.0f * (m_fValues[1] * m_fValues[2] + m_fValues[0] * m_fValues[3]),
                                   1.0f - 2.0f * (m_fValues[1] * m_fValues[1] + m_fValues[3] * m_fValues[3]));
         }
         else {
            /* Normal case */
            double fSqValues[4] = {
               std::pow(m_fValues[0], 2),
               std::pow(m_fValues[1], 2),
               std::pow(m_fValues[2], 2),
               std::pow(m_fValues[3], 2)
            };
            
            f_x_angle = std::atan2(2.0 * (m_fValues[0] * m_fValues[1] - m_fValues[2] * m_fValues[3]),
                                   fSqValues[0] - fSqValues[1] - fSqValues[2] + fSqValues[3]);
            f_y_angle = std::sin(2.0 * (m_fValues[1] * m_fValues[3] + m_fValues[0] * m_fValues[2]));
            f_z_angle = std::atan2(2.0 * (m_fValues[0] * m_fValues[3] - m_fValues[1] * m_fValues[2]),
                                   fSqValues[0] + fSqValues[1] - fSqValues[2] - fSqValues[3]);
         }
      }

      /**
       * 给两个向量，算怎么转能从一个转到另一个，生成Quaternion
       * 这里面的计算有待商榷，谨慎使用
       */
      inline CQuaternion& BetweenTwoVectors(const CVector3& c_vector1,
                                            const CVector3& c_vector2) {
         double fProd =
            c_vector1.DotProduct(c_vector2) /
            std::sqrt(c_vector1.SquareLength() * c_vector2.SquareLength());
         if(fProd > 0.999999f) {
            /* The two vectors are parallel, no rotation */
            m_fValues[0] = 1.0;
            m_fValues[1] = 0.0;
            m_fValues[2] = 0.0;
            m_fValues[3] = 0.0;
         }
         else if(fProd < -0.999999f) {
            /* The two vectors are anti-parallel */
            /* We need to set an arbitrary rotation axis */
            /* To find it, we calculate the cross product of c_vector1 with either X or Y,
               depending on which is not coplanar with c_vector1 */
            CVector3 cRotAxis = c_vector1;
            if(std::abs(c_vector1.DotProduct(CVector3::X)) < 0.999999) {
               /* Use the X axis */
               cRotAxis.CrossProduct(CVector3::X);
            }
            else {
               /* Use the Y axis */
               cRotAxis.CrossProduct(CVector3::Y);
            }
            /* The wanted quaternion is a rotation around cRotAxis by 180 degrees */
            FromAngleAxis(M_PI, cRotAxis);
         }
         else {
            /* The two vectors are not parallel nor anti-parallel */
            m_fValues[0] = std::sqrt(c_vector1.SquareLength() * c_vector2.SquareLength()) + fProd;
            CVector3 cCrossProd(c_vector1);
            cCrossProd.CrossProduct(c_vector2);
            m_fValues[1] = cCrossProd.GetX();
            m_fValues[2] = cCrossProd.GetY();
            m_fValues[3] = cCrossProd.GetZ();
            Normalize();
         }
         return *this;
      }

      inline bool operator==(const CQuaternion& c_quaternion) {
         return (m_fValues[0] == c_quaternion.m_fValues[0] &&
                 m_fValues[1] == c_quaternion.m_fValues[1] &&
                 m_fValues[2] == c_quaternion.m_fValues[2] &&
                 m_fValues[3] == c_quaternion.m_fValues[3]);
      }      

      inline CQuaternion& operator=(const CQuaternion& c_quaternion) {
         if (&c_quaternion != this) {
            m_fValues[0] = c_quaternion.m_fValues[0];
            m_fValues[1] = c_quaternion.m_fValues[1];
            m_fValues[2] = c_quaternion.m_fValues[2];
            m_fValues[3] = c_quaternion.m_fValues[3];
         }
         return *this;
      }

      inline CQuaternion& operator+=(const CQuaternion& c_quaternion) {
         m_fValues[0] += c_quaternion.m_fValues[0];
         m_fValues[1] += c_quaternion.m_fValues[1];
         m_fValues[2] += c_quaternion.m_fValues[2];
         m_fValues[3] += c_quaternion.m_fValues[3];
         return *this;
      }

      inline CQuaternion& operator-=(const CQuaternion& c_quaternion) {
         m_fValues[0] -= c_quaternion.m_fValues[0];
         m_fValues[1] -= c_quaternion.m_fValues[1];
         m_fValues[2] -= c_quaternion.m_fValues[2];
         m_fValues[3] -= c_quaternion.m_fValues[3];
         return *this;
      }

      inline CQuaternion& operator*=(const CQuaternion& c_quaternion) {
         double newv[4];
         newv[0] = m_fValues[0] * c_quaternion.m_fValues[0] -
            m_fValues[1] * c_quaternion.m_fValues[1] -
            m_fValues[2] * c_quaternion.m_fValues[2] -
            m_fValues[3] * c_quaternion.m_fValues[3];
         newv[1] = m_fValues[0] * c_quaternion.m_fValues[1] +
            m_fValues[1] * c_quaternion.m_fValues[0] +
            m_fValues[2] * c_quaternion.m_fValues[3] -
            m_fValues[3] * c_quaternion.m_fValues[2];
         newv[2] = m_fValues[0] * c_quaternion.m_fValues[2] -
            m_fValues[1] * c_quaternion.m_fValues[3] +
            m_fValues[2] * c_quaternion.m_fValues[0] +
            m_fValues[3] * c_quaternion.m_fValues[1];
         newv[3] = m_fValues[0] * c_quaternion.m_fValues[3] +
            m_fValues[1] * c_quaternion.m_fValues[2] -
            m_fValues[2] * c_quaternion.m_fValues[1] +
            m_fValues[3] * c_quaternion.m_fValues[0];
         m_fValues[0] = newv[0];
         m_fValues[1] = newv[1];
         m_fValues[2] = newv[2];
         m_fValues[3] = newv[3];
         return *this;
      }

      inline CQuaternion operator+(const CQuaternion& c_quaternion) const {
         CQuaternion result(*this);
         result += c_quaternion;
         return result;
      }

      inline CQuaternion operator-(const CQuaternion& c_quaternion) const {
         CQuaternion result(*this);
         result -= c_quaternion;
         return result;
      }

      inline CQuaternion operator*(const CQuaternion& c_quaternion) const {
         CQuaternion result(*this);
         result *= c_quaternion;
         return result;
      }

      /**
       * Serializes the contents of the passed quaternion into a stream as Euler angles
       * in the Z,Y,X format in degrees.
       * @param c_os The stream.
       * @param c_quaternion The quaternion.
       * @return The new state of the stream.
       */
      inline friend std::ostream& operator<<(std::ostream& c_os, const CQuaternion& c_quaternion) {
         double fZAngle, fYAngle, fXAngle;
         c_quaternion.ToEulerAngles(fZAngle, fYAngle, fXAngle);        
         c_os << fZAngle << ","
              << fYAngle << ","
              << fXAngle;
         return c_os;
      }
      
   private:

      double m_fValues[4];
   };

}

#endif
