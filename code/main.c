// EX2BrazoPS4: brazo de 4 servos (codo X, codo Y, gripper, servo D-pad)
// controlado en vivo con un DS4 -- usa P8PS4 para leer el control y
// P6PWMLIBRARY para manejar los servos por PWM. Es la version con
// hardware real de lo que se probo primero en la simulacion 3D
// (ds4_arm3d.c).
//
// IMPORTANTE: cada servo necesita su propio pin de PWM por hardware, y
// cada canal (0, 2, 3, 4...) requiere su propio overlay de device tree
// activado en /boot/orangepiEnv.txt (ver la tabla de pines en
// P6PWMLIBRARY/include/pwm.h). Aqui se asume que ya estan activados
// los overlays de SERVO_X_CHANNEL, SERVO_Y_CHANNEL, SERVO_GRIP_CHANNEL
// y SERVO_DPAD_CHANNEL -- si no, pwm_open() va a fallar para esos
// canales y el programa lo va a reportar y salir.
//
// Ejemplo en /boot/orangepiEnv.txt:
//   overlays=pwm0 pwm2 pwm3 pwm4
//
// Compilar (desde CCodes, con P6PWMLIBRARY y P8PS4 como hermanas):
//   ver EX2BrazoPS4/CMakeLists.txt
#include "ps4.h"
#include "pwm.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define SERVO_X_CHANNEL    0  // pin fisico 29 (PWM0-0), overlay "pwm0"
#define SERVO_Y_CHANNEL    3  // pin fisico 35 (PWM0-3), overlay "pwm3"
#define SERVO_GRIP_CHANNEL 2  // pin fisico 33 (PWM0-2), overlay "pwm2"
#define SERVO_DPAD_CHANNEL 4  // pin fisico 37 (PWM0-4), overlay "pwm4"

#define SERVO_PERIOD_NS 20000000 // 20 ms -> 50 Hz, estandar servo de hobby

#define ANGLE_MIN   0.0
#define ANGLE_MAX 180.0

// Velocidad del servo del D-pad: grados por segundo mientras se
// mantiene presionado. Al soltar, el servo se queda donde esta.
#define DPAD_SPEED_DEG_S 90.0

// Ver ds4_arm3d.c para la explicacion completa de por que el
// giroscopio se usa como "trim" que se suma al stick, y no como unico
// control (da velocidad angular, no un angulo -- hay que integrarlo en
// el tiempo y eso se desvia solo si no se calibra). GYRO_SENS es una
// ganancia ajustada a ojo.
#define GYRO_SENS 1200.0

static volatile sig_atomic_t running = 1;

static void on_signal(int sig)
{
    (void)sig;
    running = 0;
}

static double clamp(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static long angle_to_duty_ns(double angle_deg)
{
    angle_deg = clamp(angle_deg, ANGLE_MIN, ANGLE_MAX);
    long pulse_ns = 1000000L + (long)(angle_deg / 180.0 * 1000000.0); // 1 ms..2 ms
    return SERVO_PERIOD_NS - pulse_ns;
}

static long long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int main(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (ps4_init() != 0)
        return 1;
    ps4_calibrate_gyro();

    pwm_channel *servo_x    = pwm_open(SERVO_X_CHANNEL);
    pwm_channel *servo_y    = pwm_open(SERVO_Y_CHANNEL);
    pwm_channel *servo_grip = pwm_open(SERVO_GRIP_CHANNEL);
    pwm_channel *servo_dpad = pwm_open(SERVO_DPAD_CHANNEL);
    if (!servo_x || !servo_y || !servo_grip || !servo_dpad) {
        fprintf(stderr, "No se pudo abrir uno de los canales PWM -- "
                        "revisa que los overlays de los 4 canales esten activos.\n");
        if (servo_x)    pwm_close(servo_x);
        if (servo_y)    pwm_close(servo_y);
        if (servo_grip) pwm_close(servo_grip);
        if (servo_dpad) pwm_close(servo_dpad);
        ps4_cleanup();
        return 1;
    }
    pwm_set_period(servo_x, SERVO_PERIOD_NS);
    pwm_set_period(servo_y, SERVO_PERIOD_NS);
    pwm_set_period(servo_grip, SERVO_PERIOD_NS);
    pwm_set_period(servo_dpad, SERVO_PERIOD_NS);
    pwm_enable(servo_x);
    pwm_enable(servo_y);
    pwm_enable(servo_grip);
    pwm_enable(servo_dpad);

    ps4_state ps4;
    memset(&ps4, 0, sizeof(ps4));

    double trim_x_deg = 0, trim_y_deg = 0;
    double angle_dpad = 90.0; // arranca centrado
    long long last_ms = now_ms();

    printf("Brazo listo. Stick izq = servo X, stick der = servo Y, "
           "R2 = gripper, D-pad arriba/abajo = servo 4, "
           "L1 = reinicia el trim del giroscopio. Ctrl+C para salir.\n");

    while (running) {
        ps4_poll(&ps4);

        long long now = now_ms();
        double dt = (now - last_ms) / 1000.0;
        last_ms = now;
        if (dt > 0.1) dt = 0.1; // evita saltos si el loop se atrasa

        if (ps4.btn_l1) {
            trim_x_deg = 0;
            trim_y_deg = 0;
        }

        double stick_x_deg = (ps4.stick_lx + 1.0) / 2.0 * 180.0;
        double stick_y_deg = (ps4.stick_ry + 1.0) / 2.0 * 180.0;

        trim_x_deg += ps4.gyro_z * GYRO_SENS * dt;
        trim_y_deg += ps4.gyro_x * GYRO_SENS * dt;

        double angle_x    = clamp(stick_x_deg + trim_x_deg, ANGLE_MIN, ANGLE_MAX);
        double angle_y    = clamp(stick_y_deg + trim_y_deg, ANGLE_MIN, ANGLE_MAX);
        double angle_grip = ps4.trigger_r2 * 180.0; // R2 a fondo = pinza cerrada

        // D-pad: en evdev, "arriba" normalmente da dpad_y = -1, por eso
        // el signo menos. Si se mueve al reves, cambia -= por +=.
        // Para usar izquierda/derecha, cambia dpad_y por dpad_x.
        angle_dpad -= ps4.dpad_y * DPAD_SPEED_DEG_S * dt;
        angle_dpad = clamp(angle_dpad, ANGLE_MIN, ANGLE_MAX);

        pwm_set_duty_cycle(servo_x, angle_to_duty_ns(angle_x));
        pwm_set_duty_cycle(servo_y, angle_to_duty_ns(angle_y));
        pwm_set_duty_cycle(servo_grip, angle_to_duty_ns(angle_grip));
        pwm_set_duty_cycle(servo_dpad, angle_to_duty_ns(angle_dpad));

        usleep(20000); // 20 ms, al ritmo del periodo del servo
    }

    printf("\n");
    pwm_close(servo_x);
    pwm_close(servo_y);
    pwm_close(servo_grip);
    pwm_close(servo_dpad);
    ps4_cleanup();
    return 0;
}
