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

        //TEST 2 PITCH UP 900 FRAMES (30 SECONDS)
        //if (current_frame >= 1 && current_frame <= 5)
        // {
        //    fdm.inc_thrust();
        // }
        // else if (current_frame % 2 == 0)
        // {
        //    fdm.pitch_up();
        // }

        //TEST 3 ROLL RIGHT 900 FRAMES (30 SECONDS)
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.inc_thrust();
        //}
        //else if (current_frame % 2 == 0)
        //{
        //    fdm.roll_right();
        //}

       //TEST 4 YAW RIGHT 900 FRAMES (30 SECONDS)
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.inc_thrust();
        //}
        //else if (current_frame % 2 == 0)
        //{
        //    fdm.right_rudder();
        //}
       
        //TEST 5 FLAPS DOWN 900 FRAMES (30 SECONDS)
        //fdm.flaps_down()


       
        //MORE COMPLICATED TESTS ARE POSSIBLE
       //TEST x (old)
       //if (current_frame >= 1 && current_frame <= 30)
       //{
       //    fdm.roll_right();
       //}
       //else if (current_frame >= 901 && current_frame <= 930)
       //{
       //    fdm.roll_left();

       //}
       //else if (current_frame >= 931 && i+ 1 <= 935)
       //{
       //    fdm.inc_thrust();
       //}
       //else if (current_frame >= 936 && current_frame <= 1800)
       //{
       //     fdm.pitch_up();
       //}

       //TEST y (old)
       //if (current_frame >= 1 && current_frame <= 10)
       //{
       //    fdm.inc_thrust();
       //}
       //else if (current_frame >= 11 && current_frame <= 600)
       //{
       //    fdm.pitch_up();
       //}
       //else if (current_frame >= 601 && current_frame <= 900)
       //{
       //    fdm.left_rudder();
       //}
       //else if (current_frame >= 901 && current_frame <= 910)
       //{
       //    fdm.pitch_down();
       //}
       //else if (current_frame >= 911 && current_frame <= 1200)
       //{
       //    fdm.pitch_up();
       //}
       //else if (current_frame >= 1201 && current_frame <= 1500)
       //{
       //    fdm.right_rudder();
       //}
       //else if (current_frame >= 1501 && current_frame <= 1510)
       //{
       //    fdm.dec_thrust();
       //}
       // //1511 to 2100 none
       //else if (current_frame >= 2101 && current_frame <= 2700)
       //{
       //    fdm.flaps_down();
       //}


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


