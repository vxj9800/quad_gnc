#include <quad_gnc/guidance.hpp>

Joystick::Joystick()
{
    active = false;
    joystick_fd = 0;
    joystick_ev = new js_event();
    joystick_st = new joystick_state();
    joystick_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (joystick_fd > 0)
    {
        ioctl(joystick_fd, JSIOCGNAME(256), name);
        ioctl(joystick_fd, JSIOCGVERSION, &version);
        ioctl(joystick_fd, JSIOCGAXES, &axes);
        ioctl(joystick_fd, JSIOCGBUTTONS, &buttons);
        std::cout << "   Name: " << name << std::endl;
        std::cout << "Version: " << version << std::endl;
        std::cout << "   Axes: " << (int)axes << std::endl;
        std::cout << "Buttons: " << (int)buttons << std::endl;
        joystick_st->axis.reserve(axes);
        joystick_st->button.reserve(buttons);
        active = true;
        pthread_create(&thread, 0, &Joystick::loop, this);
    }
}

Joystick::~Joystick()
{
    if (joystick_fd > 0)
    {
        active = false;
        pthread_join(thread, 0);
        close(joystick_fd);
    }
    delete joystick_st;
    delete joystick_ev;
    joystick_fd = 0;
}

void *Joystick::loop(void *obj)
{
    while (reinterpret_cast<Joystick *>(obj)->active)
        reinterpret_cast<Joystick *>(obj)->readEv();
    return 0;
}

void Joystick::readEv()
{
    int bytes = read(joystick_fd, joystick_ev, sizeof(*joystick_ev));
    if (bytes > 0)
    {
        joystick_ev->type &= ~JS_EVENT_INIT;
        if (joystick_ev->type & JS_EVENT_BUTTON)
        {
            // Make the button state latching
            // Meaning, if the value was previously 1 then don't update it
            joystick_st->button[joystick_ev->number] = joystick_st->button[joystick_ev->number] ? 1 : joystick_ev->value;
        }
        if (joystick_ev->type & JS_EVENT_AXIS)
        {
            joystick_st->axis[joystick_ev->number] = joystick_ev->value;
        }
    }
}

joystick_position Joystick::joystickPosition(int n)
{
    joystick_position pos;

    if (n > -1 && n < axes)
    {
        int i0 = n * 2, i1 = n * 2 + 1;
        float x0 = joystick_st->axis[i0] / 32767.0f, y0 = -joystick_st->axis[i1] / 32767.0f;
        float x = x0 * sqrt(1 - pow(y0, 2) / 2.0f), y = y0 * sqrt(1 - pow(x0, 2) / 2.0f);

        pos.x = x0;
        pos.y = y0;

        pos.theta = atan2(y, x);
        pos.r = sqrt(pow(y, 2) + pow(x, 2));
    }
    else
    {
        pos.theta = pos.r = pos.x = pos.y = 0.0f;
    }
    return pos;
}

bool Joystick::buttonPressed(int n)
{
    bool buttonState = false;
    if (n > -1 && n < buttons)
    {
        // Get the button state
        buttonState = joystick_st->button[n];

        // Unlatch the button on reading
        joystick_st->button[n] = 0;
    }
    return buttonState;
}

void Joystick::getTRPY(double &thrust, double &roll, double &pitch, double &yaw)
{
    // Get RC input or desired state values
    joystick_position rt = joystickPosition(0), yp = joystickPosition(1);

    // Convert values to actual roll and pitch values
    thrust = (rt.y + 1) / 2; // Limit the value in 0 to 1 range
    roll = rt.x * rollRange;
    pitch = yp.y * pitchRange;
    yaw = -yp.x; // Minus sign is applied because left position on joystick should represent ccw rotation
}

bool Joystick::getArmState()
{
    static bool quadArmed = false;
    quadArmed = buttonPressed(1) ? 1 : quadArmed; // If button 1 was pressed then quad is armed
    quadArmed = buttonPressed(0) ? 0 : quadArmed; // If button 0 was pressed then quad is not armed
    return quadArmed;
}