def powertrain(throttle,
               brake,
               rpm_table,
               torque_max_table,
               gear_ratio,
               diff,
               diff_ni,
               transmition_ni,
               gear_selection,
               engine_inertia,
               axel_inertia,
               gearbox_inertia,
               shaft_inertia,
               wheel_inertia,
               max_brake_torque,
               brake_bias,
               acc_x,
               wheel_w_vel,
               gear,
               vx):

    # Update Engine RPM
    rpm = gear_ratio[gear] * diff * \
        np.mean(wheel_w_vel)   # Wheel vx to engine RPM
    if rpm < rpm_table[0]:
        rpm = rpm_table[0]
    if rpm > rpm_table[-1]:
        rpm = rpm_table[-1]

    # Calculate torque provided by the engine based on the engine RPM
    # how much torque is available thoughout the rpm range
    torque_interpolation = interp1d(rpm_table, torque_max_table)
    # Max torque available at rpm
    torque_available = torque_interpolation(rpm)
    # find the torque delivered by te engine
    engine_torque = throttle * torque_available

    # Gearbox up or down shifting
    if vx > gear_selection[int(throttle * 10)][gear]:
        gear = gear + 1
        if gear >= gear_ratio.size:
            gear = gear_ratio.size - 1
    elif vx < 0.8 * gear_selection[int(throttle * 10)][gear - 1]:
        gear = gear - 1
        if gear < 1:
            gear = 1

    traction_torque = (engine_torque * gear_ratio[gear] * diff * diff_ni * transmition_ni) - (
        (engine_inertia + axel_inertia + gearbox_inertia) * gear ** 2 + shaft_inertia * gear_ratio[gear] * diff ** 2 + wheel_inertia) * acc_x

    # --------------------Break Torque -------------------------
    brake_torque = brake * max_brake_torque

    # -------------------- Total Torque -------------------------
    powertrain_net_torque = (traction_torque - brake_torque) * brake_bias

    return [rpm, gear, powertrain_net_torque]
