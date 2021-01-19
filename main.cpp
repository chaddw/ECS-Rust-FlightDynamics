
#include <iostream>
#include <cstdio>

#include "BourgFDM.hpp"

const int FRAME_RATE{30}; // frame rate

int main()
{
   std::cout << "Flight Simulator\n";
   const float dt{1.0f / static_cast<float>(FRAME_RATE)};
   double current_time = 0.0;
   int current_frame = 0;

   BourgFDM fdm;

   while (current_time < 30.0) //total time
   {
       current_frame = current_frame + 1;
       current_time = current_time + dt;

       fdm.zero_rudder();
       fdm.zero_ailerons();
       fdm.zero_elevators();
       fdm.zero_flaps();

        //FOR EQUIVALENCY TESTS: set flight controls artificially based on current frame
        //TEST 1 DO NOTHING

        //TEST 2 ROLL 
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


        //TEST 3 PITCH
         //if (current_frame >= 1 && current_frame <= 5)
         //{
         //    fdm.inc_thrust();
         //}

         //fdm.pitch_up();


       //TEST 4 YAW
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.inc_thrust();
        //}
        //else if (current_frame >= 6 && current_frame <= 246) //pitch up 8 seconds to stablize airplane
        //{
        //    fdm.pitch_up();
        //}
        //else if (current_frame >= 247 && current_frame <= 307)//(current_frame % 9 == 0)
        //{
        //    fdm.right_rudder();
        //}
       
        //TEST 5 FLAPS
       //fdm.flaps_down();


       //TEST 6 EVERYTHING
        //if (current_frame >= 1 && current_frame <= 5)
        //{
        //    fdm.inc_thrust();
        //}
        //else if (current_frame >= 6 && current_frame <= 246)
        //{
        //    fdm.pitch_up();
        //}
        //else if (current_frame >= 247 && current_frame <= 307)
        //{
        //    fdm.left_rudder();
        //}
        //else if (current_frame >= 308 && current_frame <= 368)
        //{
        //    fdm.right_rudder();
        //}
        //else if (current_frame >= 369 && current_frame <= 469)
        //{
        //    fdm.roll_right();
        //}
        //else if (current_frame >= 470 && current_frame <= 530)
        //{
        //    fdm.roll_left();
        //}

        //else if (current_frame >= 531 && current_frame <= 546)
        //{
        //    fdm.pitch_down();
        //}
        //else if (current_frame >= 547 && current_frame <= 552)
        //{
        //    fdm.dec_thrust();
        //}
        //else if (current_frame >= 553 && current_frame <= 900)
        //{
        //    fdm.flaps_down();
        //}




      fdm.update(dt);

      std::cout << "Simulation time : " << current_time << ", frames: " << current_frame << std::endl;
      std::cout << "Position.x : " << fdm.position.x << std::endl;
      std::cout << "Position.y : " << fdm.position.y << std::endl;
      std::cout << "Position.z : " << fdm.position.z << std::endl;
      std::cout << "Roll   : " << fdm.euler_angles.x << std::endl;
      std::cout << "Pitch  : " << -fdm.euler_angles.y << std::endl;
      std::cout << "Yaw    : " << fdm.euler_angles.z << std::endl;
      std::cout << "Airpeed  : " << fdm.speed / 1.688f << std::endl; // divide by 1.688 to convert ft/s to knots

      if (fdm.flaps) std::cout << "Flaps!\n";
      if (fdm.stalling) std::cout << "Stall!\n";

      std::cout << "===============================================\n";

   }

   return 0;
}