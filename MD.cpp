#include <MD.hpp>

MD::MD(PinName pwm, PinName dir, double max_duty, bool default_dir)
    : _pwm(pwm), _dir(dir)
{
    _max_duty = max_duty;
    _default_dir = default_dir;

    //  frequency
    _pwm.period(1.0 / 20000);
}

void MD::drive(double power)
{

    _dir = power >= 0 ? _default_dir : !_default_dir;
    
    //  pwm
    _pwm.write(abs(power) > _max_duty ? _max_duty : abs(power));
}