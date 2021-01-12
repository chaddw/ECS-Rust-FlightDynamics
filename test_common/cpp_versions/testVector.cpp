
#include <iostream>
#include "Vector.hpp"
#include <iomanip>

using namespace std;


int main()
{  
   //Max out the precision
   std::cout << setprecision(100);


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
      Vector v(63.73516082763671875, -0.004545502364635467529296875, -8.11302661895751953125);
      cout << "Magnitude v(63.73516082763671875, -0.004545502364635467529296875, -8.11302661895751953125): " << v.magnitude() << endl;
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
      Vector v(-0.061048544943332672119140625, 0.0, 0.998134791851043701171875);
      v.normalize();
      cout << "Normalize v(-0.061048544943332672119140625, 0.0, 0.998134791851043701171875)"  << " v.x: " << v.x
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
      Vector v(0.000291615608148276805877685546875,
            -0.001196154276840388774871826171875,
            0.0007719345740042626857757568359375),
            
            u(0.0000101931509561836719512939453125,
            -0.005297116935253143310546875,
            0.000000784755684435367584228515625);

      Vector uv = v+u;
      cout << "operator+ : uv = v() + u()" << " uv.x: " << uv.x
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
      Vector v(0.000291615608148276805877685546875,
            -0.001196154276840388774871826171875,
            0.000771934574004262685775756835937), 
            
            u(0.0000101931509561836719512939453125,
            -0.005297116935253143310546875,
            0.000000784755684435367584228515625);

      Vector uv = v-u;
      cout << "operator- : uv = v() - u()" << " uv.x: " << uv.x
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
      // cross product
      Vector v(1.01931509561836719512939453125e-05,
               -0.005297116935253143310546875,
               7.84755684435367584228515625e-07),
                            
               u(1.81550502777099609375,
                  0.0,
                  -0.6772263050079345703125);

      Vector uv = v^u;
      cout << "operator^ : uv = v() ^ u()" << " uv.x: " << uv.x
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
      // dot product
      Vector v(-0.9932043552398681640625,
            -3.16645900966250337660312652587890625e-06,
            0.116383351385593414306640625), 
      
            u(0.0, 0.0, 1.0);

      float uv = v*u;
      cout << "operator* : uv = v() * u()" << " uv: " << uv << endl;
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
