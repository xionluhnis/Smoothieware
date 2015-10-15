
#ifndef LOADCELL_MODULE_H
#define LOADCELL_MODULE_H

#include "libs/Module.h"
#include "libs/Pin.h"

#include "hx711.h"

#include <bitset>

class LoadCell : public Module{
    public:
        Endstops();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);
        uint32_t acceleration_tick(uint32_t dummy);

    private:
        void home(char axes_to_move);
        void do_homing_cartesian(char axes_to_move);
        void do_homing_corexy(char axes_to_move);
        void wait_for_homed(char axes_to_move);
        void wait_for_homed_corexy(int axis);
        void corexy_home(int home_axis, bool dirx, bool diry, float fast_rate, float slow_rate, unsigned int retract_steps);
        void back_off_home(char axes_to_move);
        void move_to_origin(char);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_idle(void *argument);

        float homing_position[3];
        float home_offset[3];
        uint8_t homing_order;
        std::bitset<3> home_direction;
        std::bitset<3> limit_enable;

        unsigned int  debounce_count;
        float  retract_mm[3];
        float  trim_mm[3];
        float  fast_rates[3];
        float  slow_rates[3];
        Pin    pins[6];
        volatile float feed_rate[3];
        struct {
            bool is_corexy:1;
            bool is_delta:1;
            bool is_scara:1;
            bool move_to_origin_after_home:1;
            uint8_t bounce_cnt:4;
            volatile char status:3;
        };
};

#endif
