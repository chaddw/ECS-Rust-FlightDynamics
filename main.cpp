
#include <iostream>
#include <cstdio>

#include "BourgFDM.hpp"

const int FRAME_RATE{30}; // frame rate

int main()
{
   std::cout << "Flight Simulator\n";
   const float dt{1.0f / static_cast<float>(FRAME_RATE)};
   float current_time{};

   BourgFDM fdm;


   int current_frame = 0;

   for (int i{}; i < 900; i++) 
   {
       current_frame = i + 1;
       fdm.zero_rudder();
       fdm.zero_ailerons();
       fdm.zero_elevators();
       fdm.zero_flaps();

        //FOR EQUIVALENCY TESTS: set flight controls artificially based on current frame
        //TEST 1 DO NOTHING

       //TEST 2 THRUST
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.dec_thrust();
        //}

        //TEST 3 ROLL 
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.inc_thrust();
        //}
        //else if (current_frame >= 6 && current_frame <= 246) //pitch up 8 seconds to stablize airplane
        //{
        //    fdm.pitch_up();
        //}
        //else if (current_frame >= 247 && current_frame <= 307) //roll right for 2 seconds
        //{
        //    fdm.roll_right();
        //}


        //TEST 4 PITCH
         //if (current_frame >= 1 && current_frame <= 5)
         //{
         //    fdm.inc_thrust();
         //}

         //fdm.pitch_up();


       //TEST 5 YAW
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.inc_thrust();
        //}
        //else if (current_frame >= 6 && current_frame <= 246) //pitch up 8 seconds to stablize airplane
        //{
        //    fdm.pitch_up();
        //}
        //else if (current_frame % 9 == 0)
        //{
        //    fdm.right_rudder();
        //}
       
        //TEST 6 FLAPS
       //fdm.flaps_down();


       current_time += dt;

      fdm.update(dt);
//      fdm.update2(dt);
      std::cout << "Simulation time : " << current_time << ", frames: " << current_frame << std::endl;
      std::cout << "Roll   : " << fdm.euler_angles.x << std::endl;
      std::cout << "Pitch  : " << -fdm.euler_angles.y << std::endl;
      std::cout << "Yaw    : " << fdm.euler_angles.z << std::endl;
      std::cout << "Alt    : " << fdm.position.z << std::endl;
      std::cout << "Thrust : " << fdm.thrust_force << std::endl;
      std::cout << "Speed  : " << fdm.speed / 1.688f << std::endl; // divide by 1.688 to convert ft/s to knots
      std::cout << "Position.x : " << fdm.position.x << std::endl;
      std::cout << "Position.y : " << fdm.position.y << std::endl;
      std::cout << "Position.z (alt) : " << fdm.position.z << std::endl;


      if (fdm.flaps) std::cout << "Flaps!\n";
      if (fdm.stalling) std::cout << "Stall!\n";

      std::cout << "===================================================\n";


   }

   return 0;
}

