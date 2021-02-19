#include <iostream>
#include "Quaternion.hpp"
#include "Vector.hpp"
#include <iomanip>

using namespace std;

void print_vector(Vector v)
{
   cout << v.x << " " << v.y << " " << v.z << endl;
}

int main()
{

   //Max out the precision
   std::cout << setprecision(100);

   cout << "--------------\n";
   cout << "Tests: Methods\n";
   cout << "--------------\n";

   {  // Magnitude test 1
      Quaternion q(2,2,2,2);
      cout << "Magnitude of q(2,2,2,2) : " << q.magnitude() << endl;
   }

   {  // Magnitude test 2
      Quaternion q(8.1, 15.25, 0.1, 2.89);
      cout << "Magnitude of q(8.1, 15.25, 0.1, 2.89) : " << q.magnitude() << endl;
   }


   {  // operator+= test 1
      Quaternion q1(2,1,2,3);
      Quaternion q2(3,1,2,3);
      q1 += q2;
      cout << "operator+= test 1 : "; print_vector(q1.get_vector());
      cout << "                  : " << q1.get_scalar() << endl;
   }
   {  // operator+= test 2
      Quaternion q1(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      Quaternion q2(0.20345166, -0.35279512, -0.034243274, -0.91267216);
      q1 += q2;
      cout << "operator+= test 2 : "; print_vector(q1.get_vector());
      cout << "                  : " << q1.get_scalar() << endl;
   }

   {  // operator*= scalar multiplication test 1
      Quaternion q(2,1,2,3);
      q *= 3;
      cout << "operator*= test 1 : "; print_vector(q.get_vector());
      cout << "                  : " << q.get_scalar() << endl;
   }
      {  // operator*= scalar multiplication test 2
      Quaternion q(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      q *= 0.016666668;
      cout << "operator*= test 2 : "; print_vector(q.get_vector());
      cout << "                  : " << q.get_scalar() << endl;
   }

   {  // operator/= scalar division test 1
      Quaternion q(2,1,2,3);
      q /= 3.0;
      cout << "operator/= test 1 : "; print_vector(q.get_vector());
      cout << "                  : " << q.get_scalar() << endl;
   }

   {  // operator/= scalar division test 2
      Quaternion q(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      q /= 0.016666668;
      cout << "operator/= test 2 : "; print_vector(q.get_vector());
      cout << "                  : " << q.get_scalar() << endl;
   }

   {  // operator~ conjugate of the quaternion test 1
      Quaternion q1(2,1,2,3);
      Quaternion q2(~q1);
      cout << "operator~ test 1 : "; print_vector(q2.get_vector());
      cout << "                 : " << q2.get_scalar() << endl;
   }

      {  // operator~ conjugate of the quaternion test 2
      Quaternion q1(0.3199454, 0.04186167, -0.2620119, -0.9095231);
      Quaternion q2(~q1);
      cout << "operator~ test 2 : "; print_vector(q2.get_vector());
      cout << "                 : " << q2.get_scalar() << endl;
   }

   cout << "------------------------------\n";
   cout << "Tests: Functions and Operators\n";
   cout << "------------------------------\n";

   {
      // operator* multiply quaternions
      Quaternion q1(2,1,2,3);
      Quaternion q2(3,1,2,3);
      q1 = q1 * q2;
      cout << "operator*= multiply quaternions test 1 : "; print_vector(q1.get_vector());
      cout << "                                       : " << q1.get_scalar() << endl;

   }

      {
      // operator* multiply quaternions
      Quaternion q1(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      Quaternion q2(0.3199454, 0.04186167, -0.2620119, -0.9095231);
      q1 = q1 * q2;
      cout << "operator*= multiply quaternions test 2 : "; print_vector(q1.get_vector());
      cout << "                                       : " << q1.get_scalar() << endl;

   }

   {
      // operator* multiply vector and quaternion test 1
      Quaternion q1(2,1,2,3);
      Vector v(3,1,2);
      q1 = q1 * v;
      cout << "operator*= multiply quaternion by vector test 1  : "; print_vector(q1.get_vector());
      cout << "                                                 : " << q1.get_scalar() << endl;

   }

   {
      // operator* multiply vector and quaternion test 2
      Quaternion q1(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      Vector v(127.105736, -13.1427, -10.31398);
      q1 = q1 * v;
      cout << "operator*= multiply quaternion by vector test 2 : "; print_vector(q1.get_vector());
      cout << "                                                : " << q1.get_scalar() << endl;

   }

   {
      // qvrotate rotate quaternion by vector test 1
      Quaternion q1(2,1,2,3);
      Vector v(3,1,2);
      v = QVRotate(q1, v);
      cout << "qvrotate test 1  : " << v.x << " "
                                    << v.y << " "
                                    << v.z << endl;
   }

   {
      // qvrotate rotate quaternion by vector test 2
      Quaternion q1(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      Vector v(127.105736, -13.1427, -10.31398);
      v = QVRotate(q1, v);
      cout << "qvrotate test 2  : " << v.x << " "
                                    << v.y << " "
                                    << v.z << endl;
   }


   {
      // make quaternion from euler angles test 1
      Quaternion q = MakeQFromEulerAngles(5, 7, 10);
      cout << "make q from euler angles test 1 : "; print_vector(q.get_vector());
      cout << "                                : " << q.get_scalar() << endl;

   }

   {
      // make quaternion from euler angles test 2
      Quaternion q = MakeQFromEulerAngles(-2.790951251983642578125, -4.475101947784423828125, 4.585966587066650390625);
      cout << "make q from euler angles test 2 : "; print_vector(q.get_vector());
      cout << "                                : " << q.get_scalar() << endl;

   }



   {
      // make euler angles from quaternion test 1
      Quaternion q(2,1,2,3);
      Vector v = MakeEulerAnglesFromQ(q);
      cout << "make euler angles from q test 1: "  << v.x << " "
                                                   << v.y << " "
                                                   << v.z  << endl;
   }


   {
      // make euler angles from quaternion test 2
      Quaternion q(0.99817944, -0.022753572, -0.039973494, 0.03901701);
      
      Vector v = MakeEulerAnglesFromQ(q);
      cout << "make euler angles from q test 2: "  << v.x << " "
                                                   << v.y << " "
                                                   << v.z  << endl;
                               
   }

   {
      // make euler angles from quaternion test 3
      Quaternion q(0.3199454, 0.04186167, -0.2620119, -0.9095231);
      
      Vector v = MakeEulerAnglesFromQ(q);
      cout << "make euler angles from q test 3: "  << v.x << " "
                                                   << v.y << " "
                                                   << v.z  << endl;

                                 
   }

   {
      // make euler angles from quaternion test 4
      Quaternion q(0.20345166, -0.35279512, -0.034243274, -0.91267216);
      
      Vector v = MakeEulerAnglesFromQ(q);
      cout << "make euler angles from q test 4: "  << v.x << " "
                                                   << v.y << " "
                                                   << v.z  << endl;

                                 
   }






   return 0;
}
