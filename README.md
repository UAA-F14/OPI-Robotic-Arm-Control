# ORANGE-ARM-CONTROLL
<img width="2154" height="1044" alt="image" src="https://github.com/user-attachments/assets/cc148a82-939a-450d-9254-3bda78d64a3c" />


4-Servos robotic arm controlled by PS4 controller via Bluetooth, C-based, using `evdev` for real-time input reading.



**Control**
| Entrada | Acción |
|---|---|
| Left Stick  | Servo X (base) |
| Right Stick | Servo Y |
| Twist wrist/ tilt controller | Trim added to the sticks (gyroscope) |
| L1 | Reset gyro rim |
| R2 Trigger | Gripper (fully pressed = closed) |

**Connect DS4 to the Orange Pi**

```bash
bluetoothctl
agent NoInputNoOutput
default-agent
scan on
pair <MAC>
trust <MAC>
connect <MAC>
```



**Compilar (Orange Pi, `code/`)**

```bash
cd code
cmake -S . -B build
cmake --build build
./build/EX2BrazoPS4
```


## Directory structure
```
├── code
│   ├── build
│   │   ├── EX2BrazoPS4
│   ├── CMakeLists.txt
│   ├── inc
│   │   ├── gpio.h
│   │   ├── ps4.h
│   │   └── pwm.h
│   ├── main.c
│   └── src
│       ├── gpio.c
│       ├── ps4.c
│       └── pwm.c
├── design
│   ├── 3DDesign
│   │   ├── 449747-brazo-robotico-robotic-arm-33f3bc05-b693-46d1-aa83-425e5f54e855.pdf
│   │   ├── brazo-robotico-robotic-arm-model_files.zip
│   │   └── KKP3_ARM.gcode
│   └── KicadDesign
│       ├── KicadDesign.kicad_pcb
│       ├── KicadDesign.kicad_pro
│       └── KicadDesign.kicad_sch
└── README.md
```



- `Design.kicad_sch` / `.kicad_pcb` / `.kicad_pro` — KiCad design
- `code/` Orange Pi code
  - `inc/` — headers (`gpio.h`, `pwm.h`, `ps4.h`)
  - `src/` — implementation (`gpio.c`, `pwm.c`, `ps4.c`)
  - `main.c` — main program: reads the controller with `ps4.h` and moves the 3 servos with `pwm.h`
  - `CMakeLists.txt` — CMake build file

![3D render](render.png)



## Credit
Built as coursework with [DasReyxr](https://github.com/DasReyxr)  and [Kevin Lara](https://github.com/Gyonyu)





## Template
```
mkdir code design doc scripts sim src
```

```
tree >> README.md
```

