
#include <iostream>
#include "Matrix3x3.hpp"
#include <iomanip>
#include <limits>

using namespace std;

void print_matrix(Matrix3x3 m)
{
   cout << endl << m.e11 << " " << m.e12 << " " << m.e13 
        << endl << m.e21 << " " << m.e22 << " " << m.e23
        << endl << m.e31 << " " << m.e32 << " " << m.e33 << endl;
}

int main()
{

   //Max out the precision
   std::cout << setprecision(std::numeric_limits<float>::digits);

   cout << "--------------\n";
   cout << "Tests: Methods\n";
   cout << "--------------\n";


   {  // inverse
      Matrix3x3 m(1,2,3,4,5,6,7,8,9);
      cout << "Inverse Test: Matrix3x3:";
      print_matrix(m);
      cout << "Inverse:";
      print_matrix(m.inverse());
   }

      {  // inverse
      Matrix3x3 m(2549.629150390625, -0.0, 166.91925048828125, -0.0, 2024.4990234375, -0.0, 166.91925048828125, -0.0, 4414.73388671875);
      cout << "Inverse Test: Matrix3x3:";
      print_matrix(m);
      cout << "Inverse:";
      print_matrix(m.inverse());
   }


   cout << "------------------------------\n";
   cout << "Tests: Functions and Operators\n";
   cout << "------------------------------\n";

   {  // multiply matrix by vector
      Matrix3x3 m(1,2,3,4,5,6,7,8,9);
      cout << "Multiply matrix by vector test: Matrix3x3:";
      print_matrix(m);

      cout << "Vector: (3,3,3)" << std::endl;
      Vector v = (m * Vector(3,3,3));

      cout << "Matrix * Vector : "  << " v.x: " << v.x
                                    << " v.y: " << v.y
                                    << " v.z: " << v.z  << endl;
   }

   {  // multiply matrix by vector
      Matrix3x3 m(2549.629150390625, -0.0, 166.91925048828125, -0.0, 2024.4990234375, -0.0, 166.91925048828125, -0.0, 4414.73388671875);
      cout << "Multiply matrix by vector test: Matrix3x3:";
      print_matrix(m);

      cout << "Vector: (0.000029893242754042148590087890625, 0.063622482120990753173828125, -0.000000184518285095691680908203125)" << std::endl;
      Vector v = (m * Vector(0.000029893242754042148590087890625, 0.063622482120990753173828125, -0.000000184518285095691680908203125));

      cout << "Matrix * Vector : "  << " v.x: " << v.x
                                    << " v.y: " << v.y
                                    << " v.z: " << v.z  << endl;
   }

   

   return 0;
}
