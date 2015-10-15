#include "libs/Module.h"
#include "libs/Kernel.h"
#include "modules/communication/utils/Gcode.h"
#include "modules/robot/Conveyor.h"
#include "LoadCell.h"
#include "libs/nuts_bolts.h"
#include "libs/Pin.h"
#include "wait_api.h" // mbed.h lib
#include "Config.h"
#include "Planner.h"
#include "checksumm.h"
#include "utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "PublicDataRequest.h"
#include "LoadCellPublicAccess.h"
#include "StreamOutputPool.h"
#include "Pauser.h"

#include <ctype.h>
#include <sstream>

#define loadcell_enable_checksum      CHECKSUM("enable")
#define loadcell_data_pin_checksum    CHECKSUM("data_pin")
#define loadcell_clock_pin_checksum   CHECKSUM("clock_pin")
#define loadcell_gain_checksum        CHECKSUM("gain")
#define loadcell_scale_checksum       CHECKSUM("scale")
#define loadcell_offset_checksum      CHECKSUM("offset")

#define PIN_DATA 0
#define PIN_CLCK 1

LoadCell::LoadCell()
{
    gain = 1;
    offset = 0;
    scale = 1.0f;
}

void LoadCell::on_module_loaded()
{
    // Do not do anything if not enabled
    if ( THEKERNEL->config->value( loadcell_checksum, loadcell_enable_checksum )->by_default(false)->as_bool() == false ) {
        delete this;
        return;
    }

    register_for_event(ON_GCODE_RECEIVED);
    register_for_event(ON_GET_PUBLIC_DATA);
    register_for_event(ON_SET_PUBLIC_DATA);

    // Settings
    this->on_config_reload(this);
}

// Get config
void LoadCell::on_config_reload(void *argument)
{
    this->pins[PIN_DATA].from_string( THEKERNEL->config->value(loadcell_checksum, loadcell_data_pin_checksum  )->by_default("0.27" )->as_string())->as_input();
    this->pins[PIN_CLCK].from_string( THEKERNEL->config->value(loadcell_checksum, loadcell_clock_pin_checksum )->by_default("0.28" )->as_string())->as_output();

    // These are the old ones in steps still here for backwards compatibility
    this->set_gain(
        THEKERNEL->config->value(loadcell_checksum, loadcell_gain_checksum     )->by_default(128)->as_number(), false
    );
    this->offset = THEKERNEL->config->value(loadcell_checksum, loadcell_offset_checksum   )->by_default(0 )->as_number();
    this->scale  = THEKERNEL->config->value(loadcell_checksum, loadcell_scale_checksum    )->by_default(1.0f )->as_number();
}

void LoadCell::set_gain(char g, bool apply){
    switch(g){
        case 128: gain = 1; break; // channel A, gain factor 128
        case 64:  gain = 3; break; // channel A, gain factor 64
        case 32:  gain = 2; break; // channel B, gain factor 32
    }
    if(!apply) return;

    pins[PIN_CLCK].set(false);
    read();
}

bool LoadCell::is_ready()
{
    return !pins[PIN_DATA].get();
}

long LoadCell::read()
{
    // from bogde/HX711
    // @see https://github.com/bogde/HX711/blob/5c526afe67004a8354d5c52db5d58e4b8ceca645/HX711.cpp
    int b = 0;
    while(!is_ready() && b++ < 100000);
    if(b > 100000){
        return -1;
    }

    char data[3];
    // pulse the clock pin 24 times to read the data
    for(int j = 3; j--;) {
        for(int i = 8; i--;){
            pins[PIN_CLCK].set(true);
            // set bit to value
            bool value = pins[PIN_DATA].get();
            char bit = 1 << i;
            data[j] = data[j] & (0xFF ^ bit); // use mask to set bit to 0
            if(value) data[j] |= bit; // set bit to 1
            pins[PIN_CLCK].set(false);
        }
    }
    data[2] ^= 0x80;

    // set the channel and gain factor for the next reading using the clock pin
    for (int i = 0; i < gain; ++i) {
        pins[PIN_CLCK].set(true);
        pins[PIN_DATA].get();
        pins[PIN_CLCK].set(false);
    }

    return ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[0];
}

long LoadCell::read_average(char times)
{
    long sum = 0, N = 0;
    for(int i = 0; i < times; ++i){
        long r = read();
        if(r <= 0){
            // ++times;
        } else {
            ++N;
        }
    }
    return sum / N;
}

void LoadCell::tare(char times)
{
    long base = read_average(times);
    offset = base;
}

float LoadCell::get_value(char times)
{
    return read_average(times) - offset;
}

float LoadCell::get_units(char times)
{
    return last = get_value(times) / scale;
}

void LoadCell::power_down()
{
    pins[PIN_CLCK].set(false);
    pins[PIN_CLCK].set(true);
}
void LoadCell::power_up()
{
    pins[PIN_CLCK].set(false);
}

// Retrieve pressure and display it
void LoadCell::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    if (gcode->has_m) {
        switch (gcode->m) {
            case 150: { // get pressure
                char times = gcode->has_letter('T') ? char(gcode->get_value('T')) : 1;
                float pressure = get_units(times);
                gcode->stream->printf("Pressure: %f\n", pressure);
                gcode->mark_as_taken();
            }
            break;

            case 151: { // M151 Sscale Ooffset Again
                bool display = true;
                if(gcode->has_letter('A')){
                    gain = gcode->get_value('A');
                    display = false;
                }
                if(gcode->has_letter('O')){
                    offset = gcode->get_value('O');
                    display = false;
                }
                if(gcode->has_letter('S')){
                    scale = gcode->get_value('S');
                    display = false;
                }
                if(display){
                    gcode->stream->printf("Gain: %d\tOffset: %ld\tScale: %f\n", gain, offset, scale);
                    gcode->mark_as_taken();
                }
            } break;

            case 152: { // tare
                char times = gcode->has_letter('T') ? char(gcode->get_value('T')) : 1;
                tare(times);
                gcode->mark_as_taken();
            } break;

            case 153: { // simple read
                char times = gcode->has_letter('T') ? char(gcode->get_value('T')) : 1;
                long d = read_average(times);
                gcode->stream->printf("Read(%d): %ld\n", times, d);
                gcode->mark_as_taken();
            } break;

            case 154: { // loadcell debug
                int c = 0;
                while(!is_ready() && c++ < 100000);
                if(c > 100000){
                    gcode->stream->printf("Warning: not ready.\n");
                }
                char data[3];
                char bits[28] = { 0 };
                int b = 0;
                // pulse the clock pin 24 times to read the data
                for(int j = 3; j--;) {
                    for(int i = 8; i--;){
                        pins[PIN_CLCK].set(true);
                        // set bit to value
                        bool value = pins[PIN_DATA].get();
                        bits[b++] = value ? '1' : '0';
                        char bit = 1 << (7 - i);
                        data[j] = data[j] & (0xFF ^ bit); // use mask to set bit to 0
                        if(value) data[j] |= bit; // set bit to 1
                        pins[PIN_CLCK].set(false);
                    }
                    bits[b++] = ' ';
                }
                data[2] ^= 0x80;
            
                // set the channel and gain factor for the next reading using the clock pin
                for (int i = 0; i < gain; ++i) {
                    pins[PIN_CLCK].set(true);
                    pins[PIN_DATA].get(); // to take some time
                    pins[PIN_CLCK].set(false);
                }
            
                long r = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[0];
                gcode->stream->printf("Read: %ld for bits=%s\n", r, bits);
            } break;

        }
    }
}

void LoadCell::on_get_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(loadcell_checksum)) return;

    if(pdr->second_element_is(pressure_checksum)) {
        get_units(); // read pressure and store it in $last_value
        pdr->set_data_ptr(&this->last); // TODO add parameter for $times != 1
        pdr->set_taken();

    }
}

void LoadCell::on_set_public_data(void* argument){
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(loadcell_checksum)) return;

    /*
    if(pdr->second_element_is(trim_checksum)) {
        float *t= static_cast<float*>(pdr->get_data_ptr());
        this->trim_mm[0]= t[0];
        this->trim_mm[1]= t[1];
        this->trim_mm[2]= t[2];
        pdr->set_taken();

    }else if(pdr->second_element_is(home_offset_checksum)) {
        float *t= static_cast<float*>(pdr->get_data_ptr());
        if(!isnan(t[0])) this->home_offset[0]= t[0];
        if(!isnan(t[1])) this->home_offset[1]= t[1];
        if(!isnan(t[2])) this->home_offset[2]= t[2];
    }
    */
}
