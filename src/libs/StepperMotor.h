/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPERMOTOR_H
#define STEPPERMOTOR_H

#include "libs/Hook.h"
#include "Pin.h"

class StepTicker;
class Hook;

class StepperMotor {
    public:
        StepperMotor();
        StepperMotor(Pin& step, Pin& dir, Pin& en);


        void step();
        inline void unstep() { step_pin.set(0); };

        inline void enable(bool state) { en_pin.set(!state); };

        bool is_moving() { return moving; }
        void move_finished();
        void move( bool direction, unsigned int steps );
        void signal_move_finished();
        void set_speed( float speed );
        void update_exit_tick();
        void pause();
        void unpause();

        float get_steps_per_second()  const { return steps_per_second; }
        void set_steps_per_second(float ss) { steps_per_second= ss; }
        float get_steps_per_mm()  const { return steps_per_mm; }
        void change_steps_per_mm(float);
        void change_last_milestone(float);
        float get_last_milestone(void) const { return last_milestone_mm; }
        float get_current_position(void) const { return (float)current_position_steps/steps_per_mm; }

        int  steps_to_target(float);
        uint32_t get_steps_to_move() const { return steps_to_move; }
        uint32_t get_stepped() const { return stepped; }

        template<typename T> void attach( T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            Hook* hook = new Hook();
            hook->attach(optr, fptr);
            this->end_hook = hook;
        }

        template<typename T> void attach_signal_step(uint32_t step, T *optr, uint32_t ( T::*fptr )( uint32_t ) ){
            this->step_signal_hook->attach(optr, fptr);
            this->signal_step_number = step;
            this->signal_step = true;
        }

        friend class StepTicker;
        friend class Stepper;
        friend class Planner;
        friend class Robot;

    private:
        void init();
        Hook* end_hook;
        Hook* step_signal_hook;

        uint32_t signal_step_number;

        StepTicker* step_ticker;
        Pin step_pin;
        Pin dir_pin;
        Pin en_pin;

        float steps_per_second;
        float steps_per_mm;
        float max_rate;

        volatile int32_t current_position_steps;
        int32_t last_milestone_steps;
        float   last_milestone_mm;

        uint32_t steps_to_move;
        uint32_t stepped;
        uint32_t fx_counter;
        uint32_t fx_ticks_per_step;

        struct {
            bool direction:1;
            bool remove_from_active_list_next_reset:1;
            bool is_move_finished:1; // Whether the move just finished
            bool signal_step:1;
            bool paused:1;
            volatile bool moving:1;
        };

        // Called a great many times per second, to step if we have to now
        inline void tick() {
            // increase the ( fixed point ) counter by one tick 11t
            fx_counter += (uint32_t)(1<<16);

            // if we are to step now 10t
            if (fx_counter >= fx_ticks_per_step)
                step();
        };
};

#endif

