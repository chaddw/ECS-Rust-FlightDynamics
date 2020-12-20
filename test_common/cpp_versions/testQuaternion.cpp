
#include <iostream>

#include "Quaternion.hpp"
#include "Vector.hpp"

#include<iomanip>

using namespace std;

void print_vector(Vector v)
{
   cout << v.x << " " << v.y << " " << v.z << endl;
}

int main()
{
   cout << "--------------\n";
   cout << "Tests: Methods\n";
   cout << "--------------\n";

   {  // Magnitude
      Quaternion q(2,2,2,2);
      cout << "Magnitude of q(2,2,2,2) : " << q.magnitude() << endl;
   }

   {  // operator+=
      Quaternion q1(2,1,2,3);
      Quaternion q2(3,1,2,3);
      q1 += q2;
      cout << "operator+= : "; print_vector(q1.get_vector());
      cout << "           : " << q1.get_scalar() << endl;
   }

   {  // operator*= scalar multiplication
      Quaternion q(2,1,2,3);
      q *= 3;
      cout << "operator*= : "; print_vector(q.get_vector());
      cout << "           : " << q.get_scalar() << endl;
   }

   {  // operator/= scalar division
      Quaternion q(2,1,2,3);
      q /= 3.0;
      cout << "operator/= : "; print_vector(q.get_vector());
      cout << "           : " << q.get_scalar() << endl;
   }

   {  // operator~ conjugate of the quaternion
      Quaternion q1(2,1,2,3);
      Quaternion q2(~q1);
      cout << "operator~  : "; print_vector(q2.get_vector());
      cout << "           : " << q2.get_scalar() << endl;
   }

   cout << "------------------------------\n";
   cout << "Tests: Functions and Operators\n";
   cout << "------------------------------\n";

   {
      // operator* multiply quaternions
      Quaternion q1(2,1,2,3);
      Quaternion q2(3,1,2,3);
      q1 = q1 * q2;
      cout << "operator*= multiply quaternions : "; print_vector(q1.get_vector());
      cout << "                                : " << q1.get_scalar() << endl;

   }

   {
      // operator* multiply vector and quaternion
      Quaternion q1(2,1,2,3);
      Vector v(3,1,2);
      q1 = q1 * v;
      cout << "operator*= multiply quaternion by vector : "; print_vector(q1.get_vector());
      cout << "                                         : " << q1.get_scalar() << endl;

   }

   {
      // qvrotate rotate quaternion by vector
      Quaternion q1(2,1,2,3);
      Vector v(3,1,2);
      v = QVRotate(q1, v);
      cout << "qvrotate   : " << v.x << " "
                              << v.y << " "
                              << v.z  << endl;
   }

   {
      // make quaternion from euler angles
      Quaternion q = MakeQFromEulerAngles(5, 7, 10);
      cout << "make q from euler angles : "; print_vector(q.get_vector());
      cout << "                         : " << q.get_scalar() << endl;

   }

      {
      // make euler angles from quaternion
      Quaternion q(2,1,2,3);
      Vector v = MakeEulerAnglesFromQ(q);
      cout << "make euler angles from q : " << v.x << " "
                                            << v.y << " "
                                            << v.z  << endl;
   }


   return 0;
}
