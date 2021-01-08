
#include <iostream>
#include "Vector.hpp"
#include <iomanip>
#include <limits>

using namespace std;


int main()
{  
   //Max out the precision
   std::cout << setprecision(std::numeric_limits<float>::digits);


   cout << "--------------\n";
   cout << "Tests: Methods\n";
   cout << "--------------\n";

   {
      Vector v(1,1,1);
      cout << "Magnitude v(1,1,1): " << v.magnitude() << endl;
   }

   {
      Vector v(1,2,3);
      cout << "Magnitude v(1,2,3): " << v.magnitude() << endl;
   }

   {
      Vector v(1,1,1);
      v.normalize();
      cout << "Normalize v(1,1,1)"  << " v.x: " << v.x
                                    << " v.y: " << v.y
                                    << " v.z: " << v.z  << endl;
   }

   {
      Vector v(3,3,3);
      v.normalize();
      cout << "Normalize v(3,3,3)"  << " v.x: " << v.x
                                    << " v.y: " << v.y
                                    << " v.z: " << v.z  << endl;
   }

   {
      Vector v(1,2,3);
      v.normalize();
      cout << "Normalize v(1,2,3)"  << " v.x: " << v.x
                                    << " v.y: " << v.y
                                    << " v.z: " << v.z  << endl;
   }

   {
      Vector v(1,2,3);
      v.reverse();
      cout << "Reverse v(1,2,3)" << " v.x: " << v.x
                                 << " v.y: " << v.y
                                 << " v.z: " << v.z  << endl;
   }


   cout << "------------------------------\n";
   cout << "Tests: Functions and Operators\n";
   cout << "------------------------------\n";

   {
      Vector v(10,20,30), u(1,2,3);
      Vector uv = v+u;
      cout << "operator+ : uv = v(10,20,30) + u(1,2,3)" << " uv.x: " << uv.x
                                                        << " uv.y: " << uv.y
                                                        << " uv.z: " << uv.z  << endl;
   }

   {
      Vector v(10,20,30), u(1,2,3);
      Vector uv = v-u;
      cout << "operator- : uv = v(10,20,30) - u(1,2,3)" << " uv.x: " << uv.x
                                                        << " uv.y: " << uv.y
                                                        << " uv.z: " << uv.z  << endl;
   }

   {
      // cross product
      Vector v(1,0,0), u(0,1,1);
      Vector uv = v^u;
      cout << "operator^ : uv = v(1,0,0) ^ u(0,1,1)" << " uv.x: " << uv.x
                                                     << " uv.y: " << uv.y
                                                     << " uv.z: " << uv.z  << endl;
   }

   {
      // dot product
      Vector v(10,20,30), u(1,2,3);
      float uv = v*u;
      cout << "operator* : uv = v(10,20,30) * u(1,2,3)" << " uv: " << uv << endl;
      }


   {
      Vector v(1,2,3);
      Vector u{3 * v};
      cout << "operator* : u = 3 * v(1,2,3)" << " u.x: " << u.x
                                             << " u.y: " << u.y
                                             << " u.z: " << u.z  << endl;
   }

   {
      Vector v(1,2,3);
      Vector u{v / 3};
      cout << "operator* : u = v(1,2,3) / 3" << " u.x: " << u.x
                                             << " u.y: " << u.y
                                             << " u.z: " << u.z  << endl;
   }

   return 0;
}
