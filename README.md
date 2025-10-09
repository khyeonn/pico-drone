# Pico Drone

A small drone built using off-the-shelf parts, a Raspberry Pi Pico 2, and a 3D printed frame.
This was mostly a learning project to gain firsthand experience in designing and implementing sensor fusion algorithms, namely the Multiplicative Extended Kalman Filter (MEKF).

## Project Status

The project is still a work in progress. As of Oct 9, 2025, I am working on designing the frame.


## Prerequisites 
### Tools and Skills

Basic soldering skills are necessary to complete this project. As such, basic soldering tools (e.g., soldering iron, solder, and flux) are required.


### Bill of Materials

The following components were used in this project:

| Part | Quantity |
| :-- | :-- |
| Raspberry Pi Pico 2 | 1 |
| MPU9250 Module| 1 |
| 4S 1300mAh Lipo Battery| 1 |
| RS2205 2300kV BLDC Motors | 4 |
| 5x4x3 Propellers (2 CW and 2 CCW) | 4 |
| Matek Systems PDB-XT60 | 1 |

*CW - Clockwise and CCW - Counter clockwise*


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
