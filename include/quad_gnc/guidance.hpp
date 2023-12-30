// Majority of the code below is adapted from https://www.keithlantz.net/2011/10/a-linux-c-joystick-object/

// The code from quad_gnc.cpp should read axes 0:yaw, 1:throttle, 2:roll and 3:pitch from the controller.
// The buttons 0 and 1 should correspond to disarm or arm state of the quad. For the case of simulation,
// they would correspond to simulation running or not running.
// You should map these axes appropriately as per your needs using jstest-gtk app.

#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <math.h>
#include <linux/joystick.h>
#include <vector>
#include <unistd.h>

#ifndef __GUIDANCE_HEADER__
#define __GUIDANCE_HEADER__

struct joystick_state
{
    std::vector<signed short> button;
    std::vector<signed short> axis;
};

struct joystick_position
{
    float theta, r, x, y;
};

class Joystick
{
private:
    pthread_t thread;
    bool active;
    int joystick_fd;
    js_event *joystick_ev;
    joystick_state *joystick_st;
    __u32 version;
    __u8 axes;
    __u8 buttons;
    char name[256];

protected:
public:
    Joystick();
    ~Joystick();
    static void *loop(void *obj);
    void readEv();
    joystick_position joystickPosition(int n);
    bool buttonPressed(int n);
};
#endif // __GUIDANCE_HEADER__