# Pico Drone

A small drone built using off-the-shelf parts, a Raspberry Pi Pico 2, and a 3D printed frame.
This was mostly a learning project to gain firsthand experience in designing and implementing sensor fusion algorithms, namely the Multiplicative Extended Kalman Filter (MEKF).

## Project Status

I have decided to add a simulation component. This enables sim2real testing and validation which I find is a much more rewarding and comprehensive path towards building this drone. I also believe that I will learn more by adding this aspect, and that is my ultimate goal.

I have moved the development of this repo to a new repo, which will be published soon once the basics are setup. I want this to be as self-contained as possible and make the setup as painless as possible, so that anyone can clone the repo, buy the hardware, and build and fly their own drones. 


## Prerequisites 
### Tools and Skills

Basic soldering skills are necessary to complete this project. As such, basic soldering tools (e.g., soldering iron, solder, and flux) are required.


### Bill of Materials

The following components were used in this project. As project develops, the BOM will be updated:

| Part | Quantity |
| :-- | :-- |
| Raspberry Pi Pico 2 | 1 |
| MPU9250 Module| 1 |
| 4S 1300mAh Lipo Battery| 1 |
| RS2205 2300kV BLDC Motors | 4 |
| Littlebee 20A ESC | 4 |
| 5x4x3 Propellers (2 CW and 2 CCW) | 4 |
| Matek Systems PDB-XT60 | 1 |

*CW - Clockwise and CCW - Counter clockwise*


I was lucky to find this bundle on [Amazon](https://a.co/d/jiEhTFR) (not an affiliate link).
But it seems the price has gone up since I bought it. There is another bundle that is cheaper that *probably* [works](https://a.co/d/3QJOrdP) (again, not affiliate link).


## Installing

Clone the repo or download the project ZIP file and unzip it somewhere on your machine.
Then you can build the binaries using CMake inside the project root directory

```
mkdir build
cd build
CMake -G "Unix Makefiles" ..
make
```

### Flashing the Firmware

You can easily flash the firmware onto the Pico using `picotool` if you have it installed. 
Just connect your Pico to the PC via USB cable and run `picotool load main.uf2`.

Otherwise, you can manually flash by dragging the `main.uf2` inside your `build` folder. 
Plug your Pi to the computer via USB while holding down the `BOOTSEL` button. Drag and drop the `main.uf2` file into the Pi directly through file explorer. 


## Contributing

Feel free to use, modify, or contribute to your heart's desire! Any feedback is also greatly appreciated. :)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Inspired by [Tim Hanewich's Pi Pico Drone project](https://timhanewich.medium.com/taking-flight-with-the-raspberry-pi-pico-micropython-diy-quadcopter-drone-61ed4f7ee746)
* Big thanks to Matthew Hampsey for his amazingly detailed explanation and example of the [MEKF](https://matthewhampsey.github.io/blog/2020/07/18/mekf)
