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

int main(int argc, char *argv[])
{
    
    //variables to be set by po
    std::string uri; 
    size_t num_bins;
    double rate, freq, step, gain, bw, frame_rate;
    float ref_lvl, dyn_rng;
    bool show_controls;
    
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
        ("rate", po::value<double>(&rate)->default_value(10e6), "rate of incoming samples (sps) [r-R] [2.5e6..1..61.44e6]")
        ("freq", po::value<double>(&freq)->default_value(100e6), "RF center frequency in Hz [f-F] [70e6..1..6000e6] ")
        ("gain", po::value<double>(&gain)->default_value(73), "gain for the RF chain [g-G] [-3..1..73]")
        ("bw", po::value<double>(&bw)->default_value(56e6), "RF filter bandwidth in Hz [b-B] [200000..1..56000000]")
        // display parameters
        ("num-bins", po::value<size_t>(&num_bins)->default_value(512), "the number of bins in the DFT [n-N]")
        ("frame-rate", po::value<double>(&frame_rate)->default_value(15), "frame rate of the display (fps) [s-S]")
        ("ref-lvl", po::value<float>(&ref_lvl)->default_value(0), "reference level for the display (dB) [l-L]")
        ("dyn-rng", po::value<float>(&dyn_rng)->default_value(80), "dynamic range for the display (dB) [d-D]")
        ("step", po::value<double>(&step)->default_value(1e6), "tuning step for rate/bw/freq [t-T]")
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
    
    auto next_refresh = high_resolution_clock::now();

    void *p_dat, *p_end, *t_dat;
    ptrdiff_t p_inc;
    float i,q;

    //------------------------------------------------------------------
    //-- Main loop
    //------------------------------------------------------------------
    while (loop){
        
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

        //check and update the display refresh condition
        if (high_resolution_clock::now() < next_refresh) {
            continue;
        }
        next_refresh =
            high_resolution_clock::now()
            + std::chrono::microseconds(int64_t(1e6/frame_rate));

        //calculate the dft and create the ascii art frame
        ascii_art_dft::log_pwr_dft_type lpdft(
            ascii_art_dft::log_pwr_dft(&buff.front(), buff.size())
        );
        std::string frame = ascii_art_dft::dft_to_plot(
            lpdft, COLS, (show_controls ? LINES-5 : LINES),
            rate, 
            freq, 
            dyn_rng, ref_lvl
        );

        std::string header = std::string((COLS-26)/2, '-');
    	std::string border = std::string((COLS), '-');
    
        //curses screen handling: clear and print frame
        clear();        
        if (show_controls)
        {
            printw("%s-={ retrogram~plutosdr }=-%s-",header.c_str(),header.c_str());
            printw("[f-F]req: %4.3f MHz   |   [r-R]ate: %2.2f Msps   |   [b-B]w: %2.2f MHz"
                   "   |   [g-G]ain: %2.0f dB\n\n", freq/1e6, rate/1e6, bw/1e6, gain);
            printw("[d-D]yn Range: %2.0f dB    |   Ref [l-L]evel: %2.0f dB   |   fp[s-S] :"
                   " %2.0f   |   [n-N]um bins: %d   |   [t-T]uning step: %3.3f M\n", 
                   dyn_rng, ref_lvl, frame_rate, num_bins,step/1e6, show_controls);
            printw("%s", border.c_str());
        }        
        printw("%s\n", frame.c_str());

        //curses key handling: no timeout, any key to exit
        timeout(0);
        ch = getch();

        switch(ch)
        {
            case 'r':
            {
                ret = iio_channel_attr_read(rxch, "sampling_frequency", attr_buf, sizeof(attr_buf));
                if (ret > 0)        
                {
                    rate = atof(attr_buf);
                    
                    if ((rate - step) < 3e6) rate = 3e6; // avoid fractions
                    else rate -= step;
                    
                    iio_channel_attr_write_longlong(rxch, "sampling_frequency", rate);
                }
                break;
            }

            case 'R':
            {
                ret = iio_channel_attr_read(rxch, "sampling_frequency", attr_buf, sizeof(attr_buf));
                if (ret > 0)        
                {
                    rate = atof(attr_buf);
                    
                    if ((rate + step) > 61e6) rate = 61e6; // avoid fractions
                    else rate += step;

                    iio_channel_attr_write_longlong(rxch, "sampling_frequency", rate);
                }
                break;
            }

            case 'b':
            {
                ret = iio_channel_attr_read(rxch, "rf_bandwidth", attr_buf, sizeof(attr_buf));
                if (ret > 0)        
                {
                    bw = atof(attr_buf);

                    if ((bw - step)  < 200e3) bw = 200e3;
                    else  bw -= step;

                    iio_channel_attr_write_longlong(rxch, "rf_bandwidth", bw);
                }
                break;
            }

            case 'B':
            {
                ret = iio_channel_attr_read(rxch, "rf_bandwidth", attr_buf, sizeof(attr_buf));
                if (ret > 0)        
                {
                    bw = atof(attr_buf);
                    
                    if ((bw + step) > 56e6) bw = 56e6;
                    else bw += step;

                    iio_channel_attr_write_longlong(rxch, "rf_bandwidth", bw);
                }
                break;
            }

            case 'g':
            {
                ret = iio_channel_attr_read(rxch, "hardwaregain", attr_buf, sizeof(attr_buf));            
                if (ret > 0)        
                {
                    gain = atof(attr_buf);
                    
                    if (gain >= 0) gain -= 1;

                    iio_channel_attr_write(rxch, "gain_control_mode", "manual"); // RX gain change only in manual mode   
                    iio_channel_attr_write_longlong(rxch,"hardwaregain", gain); // RX gain    
                }
                break;
            }
            
            case 'G':
            {
                ret = iio_channel_attr_read(rxch, "hardwaregain", attr_buf, sizeof(attr_buf));            
                if (ret > 0)        
                {
                    gain = atof(attr_buf);
                    
                    if (gain <= 72) gain += 1;

                    iio_channel_attr_write(rxch, "gain_control_mode", "manual"); // RX gain change only in manual mode   
                    iio_channel_attr_write_longlong(rxch,"hardwaregain", gain); // RX gain    
                }
                break;
            }

            case 'f':
            {
                ret = iio_channel_attr_read(iio_device_find_channel(phy, "altvoltage0", true), "frequency", attr_buf, sizeof(attr_buf));
                
                if (ret > 0)        
                {
                    freq = atof(attr_buf);
                    
                    if ((freq - step) < 70e6) freq = 70e6;
                    else freq -= step;
                    
                    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);  // RX LO
                }
                break;
            }

            case 'F':
            {
                ret = iio_channel_attr_read(iio_device_find_channel(phy, "altvoltage0", true), "frequency", attr_buf, sizeof(attr_buf));
                
                if (ret > 0)        
                {
                    freq = atof(attr_buf);

                    if ((freq + step) > 6e9) freq = 6e9;
                    else freq += step;
                    
                    iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);  // RX LO
                }
                break;
            }

            case 'l': { ref_lvl -= 10; break; }            
            case 'L': { ref_lvl += 10; break; }
            case 'd': { dyn_rng -= 10; break; }
            case 'D': { dyn_rng += 10; break; }
            case 's': { if (frame_rate > 1) frame_rate -= 1; break;}
            case 'S': { frame_rate += 1; break; }
            case 'n': { if (num_bins > 2) num_bins /= 2; break;}
            case 'N': { num_bins *= 2; break; }
            case 't': { if (step > 1) step /= 2; break; }
            case 'T': { step *= 2; break; }
            case 'c': { show_controls = false; break; }
            case 'C': { show_controls = true; break; }
     
            case '\033':    // '\033' '[' 'A'/'B'/'C'/'D' -- Up / Down / Right / Left Press
            {
                getch();
                switch(getch())
                {
    		        case 'A':
                    case 'C':
                        ret = iio_channel_attr_read(iio_device_find_channel(phy, "altvoltage0", true), "frequency", attr_buf, sizeof(attr_buf));
                        
                        if (ret > 0)        
                        {
                            freq = atof(attr_buf);
                            if (freq >= 6e9 || freq <= 70e6) continue;
                            
                            freq += step;
                        
                            iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);  // RX LO
                        }

                        break;

    		        case 'B':
                    case 'D':                    
                        ret = iio_channel_attr_read(iio_device_find_channel(phy, "altvoltage0", true), "frequency", attr_buf, sizeof(attr_buf));
                        
                        if (ret > 0)        
                        {
                            freq = atof(attr_buf);
                            if (freq >= 6e9 || freq <= 70e6) continue;
                            
                            freq -= step;
                        
                            iio_channel_attr_write_longlong(iio_device_find_channel(phy, "altvoltage0", true), "frequency", freq);  // RX LO
                        }
                        
                        break;
                }
            }
            
            case 'q':
            case 'Q':
            {
                loop = false;
                break;
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