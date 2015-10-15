
#ifndef LOADCELL_MODULE_H
#define LOADCELL_MODULE_H

#include "libs/Module.h"
#include "libs/Pin.h"

class LoadCell : public Module{
    public:
        LoadCell();
        void on_module_loaded();
        void on_gcode_received(void* argument);
        void on_config_reload(void* argument);

    private:
        void set_gain(char gain, bool apply = true);
        bool is_ready();
        long read();
        long read_average(char times = 10);
        float get_value(char times = 1);
        float get_units(char times = 1);
        void tare(char times = 10);
        void set_scale(float scale = 1.f);
        void set_offset(long offset = 0);
        void power_down();
        void power_up();


        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);

        char   gain;
        long   offset;
        float  scale;
        float  last;
        Pin    pins[2];
};

#endif
