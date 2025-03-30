def vehicleProperties(mass, mmoi, com2TVC, servo_lim, servo_rate_lim):
    return type("Vehicle", (), {
        "mass": mass,
        "mmoi": mmoi,
        "com2TVC": com2TVC,
        "servo_lim": servo_lim,
        "servo_rate_lim": servo_rate_lim
    })

