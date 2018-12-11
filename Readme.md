# retrogram~plutosdr 

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

![retrogram~plutosdr](https://imgur.com/6tB0f9V.jpg)

Pan & Zoom spectrum using keyboard controls [decrement-Increment]. [Full feature demo](https://www.youtube.com/watch?v=JnrknBrvYjw)

* Center Frequency using keys [f-F] 
* Sampling rate    using keys [r-R]
* Bandwidth 	   using keys [b-B]
* Gain 		   using keys [g-G]
* Reference level  using keys [l-L] 
* Dynamic Range    using keys [d-D]
* Frame rate       using keys [s-S]
* DFT bin count    using keys [n-N]
* Tuning step	   using keys [t-T]

Tuning step applies for decrementing / Incrementing Center Frequency, Sampling Rate and Bandwidth.

---
	retrogram~plutosdr - ASCII Art Spectrum Analysis for PlutoSDR	
	Allowed options:
	  --help                      help message
	  --uri arg (=ip:192.168.2.1) pluto device uri
	  --rate arg (=10000000)      rate of incoming samples (sps) [r-R] 
	                              [2.5e6..1..61.44e6]
	  --freq arg (=100000000)     RF center frequency in Hz [f-F] [70e6..1..6000e6]
	                               
	  --gain arg (=73)            gain for the RF chain [g-G] [-3..1..73]
	  --bw arg (=56000000)        RF filter bandwidth in Hz [b-B] 
	                              [200000..1..56000000]
	  --num-bins arg (=512)       the number of bins in the DFT [n-N]
	  --frame-rate arg (=15)      frame rate of the display (fps) [s-S]
	  --ref-lvl arg (=0)          reference level for the display (dB) [l-L]
	  --dyn-rng arg (=80)         dynamic range for the display (dB) [d-D]
	  --step arg (=1000000)       tuning step for rate/bw/freq [t-T]

## Requires: libiio, libcurses, libboost-program-options
	
	sudo apt install libiio-dev libncurses5-dev libboost-program-options-dev

## Build:

For running a Linux host with Pluto connected via USB / Ethernet / Wifi.

	make

Compiling static binary for arm (raspi / pluto - requires libtinfo-dev libgpm-dev):

	make arm

Static binaries for running natively on Pluto can be built on Raspberry Pi (or cross-compiled). 
The static binary can then be scp'ed to Pluto after build - steps shown below. 

## Run:

	./retrogram-plutosdr --rate 5e6 --bw 5e6 --freq 100e6 --step 1e6

## Max rate/bw/gain:

	./retrogram-plutosdr --rate 61e6 --bw 56e6 --gain 73 --freq 935e6 --step 5e6

## Run natively on Pluto (static binary - requires 'linux' terminfo files):

	wget https://github.com/r4d10n/retrogram-plutosdr/raw/master/retrogram-plutosdr.arm
	scp retrogram-plutosdr.arm root@192.168.2.1:/tmp

	scp /usr/share/terminfo/l/linux root@192.168.2.1:/tmp         (or)  
	wget https://github.com/r4d10n/retrogram-plutosdr/raw/master/linux 
	scp linux root@192.168.2.1:/tmp

	ssh root@192.168.2.1

<inside pluto shell>	

	cd /tmp
	mkdir /usr/share/terminfo
	mkdir /usr/share/terminfo/l
	cp linux /usr/share/terminfo/l
	export TERM=linux
	chmod +x ./retrogram-plutosdr.arm
	./retrogram-plutosdr.arm --rate 61e6 --bw 56e6 --gain 73 --freq 935e6 --step 5e6

---

## TODO:

* Generic support for osmosdr / rtlsdr devices
* Direct Freq Entry / parameter change 
* Mouse tuning
* Modularize / Optimize with std::vector transform
* Waterfall (!) / Markers / Demodulators (!!) :)
* HTML output / Web(sockets)-server
