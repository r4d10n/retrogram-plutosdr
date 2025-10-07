/*

          _                                      /\/|     _       _        _________________
         | |                                    |/\/     | |     | |      /  ___|  _  \ ___ \
 _ __ ___| |_ _ __ ___   __ _ _ __ __ _ _ __ ___    _ __ | |_   _| |_ ___ \ `--.| | | | |_/ /
| '__/ _ \ __| '__/ _ \ / _` | '__/ _` | '_ ` _ \  | '_ \| | | | | __/ _ \ `--. \ | | |    /
| | |  __/ |_| | | (_) | (_| | | | (_| | | | | | | | |_) | | |_| | || (_) /\__/ / |/ /| |\ \
|_|  \___|\__|_|  \___/ \__, |_|  \__,_|_| |_| |_| | .__/|_|\__,_|\__\___/\____/|___/ \_| \_|
                         __/ |                     | |
                        |___/                      |_|

Wideband Spectrum analyzer on your terminal/ssh console with ASCII art.
Hacked from Ettus UHD RX ASCII Art DFT code for ADALM PLUTO.
Peak Hold feature by henrikssn [https://github.com/henrikssn]

*/
//
// Copyright 2010-2011,2014 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//


#include "ascii_art_dft.hpp" //implementation
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <curses.h>
#include <iostream>
#include <complex>
#include <cstdlib>
#include <chrono>
#include <thread>

#include <iio.h>

namespace po = boost::program_options;
using std::chrono::high_resolution_clock;

// A class to hold a tunable parameter
class Parameter
{
    public:
        Parameter(std::string name, double &value, double min, double max, double step, int x, int y)
            : name(name), value(value), min(min), max(max), step(step), x(x), y(y) {}

        void draw(WINDOW *w, bool selected)
        {
            if(selected)
                wattron(w, A_REVERSE);
            mvwprintw(w, y, x, "%s: ", name.c_str());
            if (value > 1e6)
                wprintw(w, "%3.3fM", value/1e6);
            else if (value > 1e3)
                wprintw(w, "%3.3fK", value/1e3);
            else
                wprintw(w, "%3.3f", value);

            if(selected)
                wattroff(w, A_REVERSE);
        }

        bool is_coord_in(int cx, int cy)
        {
            int len = name.length() + 10;
            return (cy == y && cx >= x && cx <= x + len);
        }

        std::string name;
        double &value;
        double min, max, step;
        int x, y;
};

static struct iio_context *scan(void)
{
    struct iio_scan_context *scan_ctx;
    struct iio_context_info **info;
    struct iio_context *ctx = NULL;
    unsigned int i;
    ssize_t ret;

    scan_ctx = iio_create_scan_context(NULL, 0);
    if (!scan_ctx) {
        fprintf(stderr, "Unable to create scan context\n");
        return NULL;
    }

    ret = iio_scan_context_get_info_list(scan_ctx, &info);
    if (ret < 0) {
        char err_str[1024];
        iio_strerror(-ret, err_str, sizeof(err_str));
        fprintf(stderr, "Scanning for IIO contexts failed: %s\n", err_str);
        goto err_free_ctx;
    }

    if (ret == 0) {
        printf("No IIO context found.\n");
        goto err_free_info_list;
    }

    if (ret == 1) {
        ctx = iio_create_context_from_uri(iio_context_info_get_uri(info[0]));
    } else {
        fprintf(stderr, "Multiple contexts found. Please select one using --uri:\n");

        for (i = 0; i < (size_t) ret; i++) {
            fprintf(stderr, "\t%d: %s [%s]\n", i,
                iio_context_info_get_description(info[i]),
                iio_context_info_get_uri(info[i]));
        }
    }

    err_free_info_list:
        iio_context_info_list_free(info);
    err_free_ctx:
        iio_scan_context_destroy(scan_ctx);

    return ctx;
}

// A function to apply a parameter change to the hardware
void apply_parameter_change(Parameter &p, iio_device *phy, iio_channel *rxch)
{
    if (p.name == "Freq")
        iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", p.value);
    else if (p.name == "Rate")
        iio_channel_attr_write_longlong(rxch, "sampling_frequency", p.value);
    else if (p.name == "BW")
        iio_channel_attr_write_longlong(rxch, "rf_bandwidth", p.value);
    else if (p.name == "Gain")
        iio_channel_attr_write_longlong(rxch, "hardwaregain", p.value);
}

int main(int argc, char *argv[])
{

    //variables to be set by po
    std::string uri;
    size_t num_bins;
    double rate, freq, step, gain, bw, frame_rate;
    float ref_lvl, dyn_rng;
    bool show_controls;
    bool peak_hold;

    ssize_t ret;
    char attr_buf[1024];

    int ch;
    bool loop = true;

    //setup the program options
    po::options_description desc("\nAllowed options");
    desc.add_options()
        ("help", "help message")
        ("uri", po::value<std::string>(&uri)->default_value("ip:192.168.2.1"), "pluto device uri")
        // hardware parameters
        ("rate", po::value<double>(&rate)->default_value(10e6), "rate of incoming samples (sps) [2.5e6..1..61.44e6]")
        ("freq", po::value<double>(&freq)->default_value(100e6), "RF center frequency in Hz [70e6..1..6000e6] ")
        ("gain", po::value<double>(&gain)->default_value(73), "gain for the RF chain [-3..1..73]")
        ("bw", po::value<double>(&bw)->default_value(56e6), "RF filter bandwidth in Hz [200000..1..56000000]")
        // display parameters
        ("num-bins", po::value<size_t>(&num_bins)->default_value(512), "the number of bins in the DFT")
        ("frame-rate", po::value<double>(&frame_rate)->default_value(15), "frame rate of the display (fps)")
        ("peak-hold", po::value<bool>(&peak_hold)->default_value(false), "enable peak hold")
        ("ref-lvl", po::value<float>(&ref_lvl)->default_value(0), "reference level for the display (dB)")
        ("dyn-rng", po::value<float>(&dyn_rng)->default_value(80), "dynamic range for the display (dB)")
        ("step", po::value<double>(&step)->default_value(1e6), "tuning step for rate/bw/freq")
        ("show-controls", po::value<bool>(&show_controls)->default_value(true), "show the keyboard controls")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::cout << boost::format("retrogram~plutosdr - ASCII Art Spectrum Analysis for PlutoSDR") << std::endl;

    //print the help message
    if (vm.count("help") or vm.count("h")){
        std::cout << boost::format("%s") % desc << std::endl;
        return EXIT_FAILURE;
    }

    //create pluto device instance
    std::cout << std::endl;
    std::cout << boost::format("Creating the pluto device with options: %s...") % uri << std::endl << std::endl;

    struct iio_context *ctx;

    if (vm.count("uri"))
        ctx = iio_create_context_from_uri(uri.c_str());
    else
        ctx = scan();

    if (!ctx) {
        fprintf(stderr, "Unable to create IIO context\n");
        return EXIT_FAILURE;
    }

    struct iio_device *phy;
    struct iio_device *dev;
    struct iio_channel *rx0_i, *rx0_q, *rxch;
    struct iio_buffer *rxbuf;


    phy = iio_context_find_device(ctx,"ad9361-phy");

    if (!phy) {
        fprintf(stderr, "Device [ad9361-phy] not found\n");
        iio_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    dev = iio_context_find_device(ctx, "cf-ad9361-lpc");

    if (!dev) {
        fprintf(stderr, "Device [cf-ad9361-lpc] not found\n");
        iio_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    rx0_i = iio_device_find_channel(dev, "voltage0", 0);
    rx0_q = iio_device_find_channel(dev, "voltage1", 0);

    rxch = iio_device_find_channel(phy, "voltage0", false);

    iio_channel_enable(rx0_i);
    iio_channel_enable(rx0_q);

    //set the sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6);
    iio_channel_attr_write_longlong(rxch, "sampling_frequency",rate); // RX baseband sample rate

    ret = iio_channel_attr_read(rxch, "sampling_frequency", attr_buf, sizeof(attr_buf));
    if (ret > 0) std::cout << boost::format(" >> Actual RX Rate: %f Msps...") % (atol(attr_buf)/1e6) << std::endl << std::endl;
    else std::cout << "Error reading RX Rate" << std::endl << std::endl;

    //set the center frequency
    std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6);
    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);  // RX LO

    ret = iio_channel_attr_read(iio_device_find_channel(phy, "altvoltage0", true), "frequency", attr_buf, sizeof(attr_buf));
    if (ret > 0) std::cout << boost::format(" >> Actual RX Freq: %f MHz...") % (atol(attr_buf)/1e6) << std::endl << std::endl;
    else std::cout << "Error reading RX Freq" << std::endl << std::endl;

    //set the rf gain
    std::cout << boost::format("Setting RX Gain: %f dB...") % gain;
    iio_channel_attr_write(rxch, "gain_control_mode", "manual"); // RX gain change only in manual mode
    iio_channel_attr_write_longlong(rxch, "hardwaregain", gain); // RX gain

    ret = iio_channel_attr_read(rxch, "hardwaregain", attr_buf, sizeof(attr_buf));
    if (ret > 0) std::cout << boost::format(" >> Actual RX Gain: %f ...") % (atol(attr_buf)) << std::endl << std::endl;
    else std::cout << "Error reading RX Gain" << std::endl << std::endl;

    //set the RF filter bandwidth

    std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw/1e6);
    iio_channel_attr_write_longlong( rxch, "rf_bandwidth", bw); // RF bandwidth

    ret = iio_channel_attr_read(rxch, "rf_bandwidth", attr_buf, sizeof(attr_buf));
    if (ret > 0) std::cout << boost::format(" >> Actual RX Bandwidth: %f MHz ...") % (atol(attr_buf)/1e6) << std::endl << std::endl;
    else std::cout << "Error reading RX Bandwidth" << std::endl << std::endl;


    std::this_thread::sleep_for(std::chrono::seconds(1)); //allow for some setup time

    std::vector<std::complex<float> > buff(num_bins);

    rxbuf = iio_device_create_buffer(dev, num_bins, false);

    if (!rxbuf)
    {
        perror("Could not create RX buffer");
        iio_context_destroy(ctx);
        return EXIT_FAILURE;
    }

    //------------------------------------------------------------------
    //-- Initialize
    //------------------------------------------------------------------
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    keypad(stdscr, TRUE);
    mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);
    mouseinterval(0);

    double d_num_bins = num_bins;
    double d_dyn_rng = dyn_rng;
    double d_ref_lvl = ref_lvl;
    double d_frame_rate = frame_rate;

    std::vector<Parameter> params;
    params.push_back(Parameter("Freq", freq, 70e6, 6e9, step, 2, 1));
    params.push_back(Parameter("Rate", rate, 2.5e6, 61.44e6, step, 24, 1));
    params.push_back(Parameter("BW", bw, 200e3, 56e6, step, 44, 1));
    params.push_back(Parameter("Gain", gain, -3, 73, 1, 64, 1));

    params.push_back(Parameter("Dyn Rng", d_dyn_rng, 10, 120, 10, 2, 3));
    params.push_back(Parameter("Ref Lvl", d_ref_lvl, -100, 50, 10, 24, 3));
    params.push_back(Parameter("FPS", d_frame_rate, 1, 60, 1, 44, 3));
    params.push_back(Parameter("Bins", d_num_bins, 32, 8192, 2, 60, 3));
    params.push_back(Parameter("Step", step, 1e3, 10e6, 2, 74, 3));
    int selected_param = -1;

    auto next_refresh = high_resolution_clock::now();

    void *p_dat, *p_end, *t_dat;
    ptrdiff_t p_inc;
    float i,q;

    //------------------------------------------------------------------
    //-- Main loop
    //------------------------------------------------------------------

    ascii_art_dft::log_pwr_dft_type last_lpdft;

    while (loop){

        num_bins = d_num_bins;
        dyn_rng = d_dyn_rng;
        ref_lvl = d_ref_lvl;
        frame_rate = d_frame_rate;
        params[0].step = step;
        params[1].step = step;
        params[2].step = step;

        buff.clear();

        iio_buffer_refill(rxbuf);

        p_inc = iio_buffer_step(rxbuf);
        p_end = iio_buffer_end(rxbuf);

        for(p_dat = iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc, t_dat += p_inc)
        {
            i = ((float)((int16_t*)p_dat)[0])/2048;
            q = ((float)((int16_t*)p_dat)[1])/2048;

            buff.push_back(std::complex<float> ( i,  q ));
        }

        // Return early to save CPU if peak hold is disabled and no refresh is required.
        if (!peak_hold && high_resolution_clock::now() < next_refresh) {
            continue;
        }

        //calculate the dft and create the ascii art frame
        ascii_art_dft::log_pwr_dft_type lpdft(
            ascii_art_dft::log_pwr_dft(&buff.front(), buff.size())
        );

        // For peak hold, compute the max of last DFT and current one
        if (peak_hold && last_lpdft.size() == lpdft.size()) {
            for (size_t i = 0; i < lpdft.size(); ++i) {
                lpdft[i] = std::max(lpdft[i], last_lpdft[i]);
            }
        }
        last_lpdft = lpdft;

        //check and update the display refresh condition
        if (high_resolution_clock::now() < next_refresh) {
            continue;
        }
        next_refresh =
            high_resolution_clock::now()
            + std::chrono::microseconds(int64_t(1e6/frame_rate));

        std::string frame;
        try
        {
                 frame = ascii_art_dft::dft_to_plot(
                    lpdft, COLS, (show_controls ? LINES-5 : LINES),
                    rate,
                    freq,
                    dyn_rng, ref_lvl
                );
        }
        catch(...){}

        std::string header = std::string((COLS-26)/2, '-');
    	std::string border = std::string((COLS), '-');

        //curses screen handling: clear and print frame
        clear();
        if (show_controls)
        {
            mvprintw(0, 0, "%s-={ retrogram~plutosdr }=-%s-",header.c_str(),header.c_str());
            for(unsigned int i=0; i < params.size(); i++)
                params[i].draw(stdscr, i == selected_param);

            mvprintw(2, 0, "Peak hold: %s", peak_hold ? "On" : "Off");
            mvprintw(4, 0, "%s", border.c_str());
        }
        printw("%s\n", frame.c_str());

        //curses key handling: no timeout, any key to exit
        timeout(0);
        ch = getch();

        if (ch == 'q' || ch == 'Q')
            loop = false;
        else if (ch == 'h' || ch == 'H')
            peak_hold = !peak_hold;
        else if (ch == 'c' || ch == 'C')
            show_controls = !show_controls;
        else if (ch == '+' || ch == '=')
        {
            if (selected_param != -1)
            {
                Parameter &p = params[selected_param];
                p.value += p.step;
                if(p.value > p.max) p.value = p.max;
                apply_parameter_change(p, phy, rxch);
            }
        }
        else if (ch == '-' || ch == '_')
        {
            if (selected_param != -1)
            {
                Parameter &p = params[selected_param];
                p.value -= p.step;
                if(p.value < p.min) p.value = p.min;
                apply_parameter_change(p, phy, rxch);
            }
        }
        else if (ch == KEY_UP || ch == KEY_RIGHT)
        {
            freq += step;
            if(freq > params[0].max) freq = params[0].max;
            iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);
        }
        else if (ch == KEY_DOWN || ch == KEY_LEFT)
        {
            freq -= step;
            if(freq < params[0].min) freq = params[0].min;
            iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);
        }
        else if(ch == KEY_MOUSE)
        {
            MEVENT event;
            if(getmouse(&event) == OK)
            {
                if(event.bstate & BUTTON1_PRESSED)
                {
                    selected_param = -1;
                    for(unsigned int i=0; i < params.size(); i++)
                    {
                        if(params[i].is_coord_in(event.x, event.y))
                        {
                            selected_param = i;
                            break;
                        }
                    }
                }
                else if(event.bstate & BUTTON4_PRESSED) // scroll up
                {
                    if(selected_param != -1)
                    {
                        Parameter &p = params[selected_param];
                        p.value += p.step;
                        if(p.value > p.max) p.value = p.max;
                        apply_parameter_change(p, phy, rxch);
                    }
                    else
                    {
                        if (event.bstate & BUTTON_SHIFT)
                        {
                            rate += step;
                            if(rate > params[1].max) rate = params[1].max;
                            iio_channel_attr_write_longlong(rxch, "sampling_frequency",rate);
                        }
                        else
                        {
                            freq += step;
                            if(freq > params[0].max) freq = params[0].max;
                            iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);
                        }
                    }
                }
                else if(event.bstate & BUTTON5_PRESSED) // scroll down
                {
                    if(selected_param != -1)
                    {
                        Parameter &p = params[selected_param];
                        p.value -= p.step;
                        if(p.value < p.min) p.value = p.min;
                        apply_parameter_change(p, phy, rxch);
                    }
                    else
                    {
                        if (event.bstate & BUTTON_SHIFT)
                        {
                            rate -= step;
                            if(rate < params[1].min) rate = params[1].min;
                            iio_channel_attr_write_longlong(rxch, "sampling_frequency",rate);
                        }
                        else
                        {
                            freq -= step;
                            if(freq < params[0].min) freq = params[0].min;
                            iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);
                        }
                    }
                }
            }
        }
    }

    //------------------------------------------------------------------
    //-- Cleanup
    //------------------------------------------------------------------

    if (rxbuf) { iio_buffer_destroy(rxbuf); }

    if (rx0_i) { iio_channel_disable(rx0_i); }
    if (rx0_q) { iio_channel_disable(rx0_q); }

    if (rxch) { iio_channel_disable(rxch); }

    if (ctx) { iio_context_destroy(ctx); }

    curs_set(true);

    endwin(); //curses done

    //finished
    std::cout << std::endl << (char)(ch) << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
