clear;
close all;
clc;

warning ('off', 'all');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultAxesFontSize', 14);



max_pwm = 255;
max_voltage = 12;

t_sample = 0.012;
serial_scaling_factor = 0.001;

cutoff_frequency = 20;

controllers_str_list = ["rot vel none", "rotation", "cascade", "state space", ...
    "state space tr"];
controller_str = "state space tr";

record_folder_name = "records general";
record_range = "A2:L2001";


if controller_str == "rot vel none"
    record_file_name = "friction identification with 12 ms sampling.txt";

elseif controller_str == "rotation"
    record_file_name = "model identification with 5.7 ms sampling.txt";

elseif controller_str == "cascade"
    record_file_name = "cascade control with 12 ms sampling.txt";

elseif controller_str == "state space"
    record_file_name = "state space and Kalman with 12 ms sampling.txt";

elseif controller_str == "state space tr"
    record_file_name = "state space and tracking with 12 ms sampling.txt";

end


record_file_rel_path = record_folder_name + "/" + record_file_name;

record_array = table2array(readtable(record_file_rel_path, 'ReadVariableNames', ...
    false, 'Range', record_range));



time = (record_array(:, 1) - record_array(1, 1)) * serial_scaling_factor - 0.08;

if controller_str == "rot vel none"
    ref_rot_vel = record_array(:, 2) * 2;

    raw_rot_vel = record_array(:, 3) * serial_scaling_factor;

    filtering_denom = tfdata(c2d(tf(cutoff_frequency, [1, cutoff_frequency]), t_sample), 'v');
    filt_rot_vel = filter(filtering_denom(2), [1, -1 + filtering_denom(2)], raw_rot_vel);

    plot_one_variable(time, ref_rot_vel, 'reference motor rotational velocity (rad/s)', 'k');
    plot_one_variable(time, filt_rot_vel, 'motor rotational velocity (rad/s)', 'b');

elseif controller_str == "rotation"
    ref_rotation = record_array(:, 2) * serial_scaling_factor;

    raw_err = record_array(:, 3) * serial_scaling_factor;
    raw_err_vel = record_array(:, 4) * serial_scaling_factor;

    plot_one_variable(time, ref_rotation, 'reference motor rotational angle (rad)', 'k');
    plot_one_variable(time, raw_err, 'motor rotational angle error (rad)', 'b');
    plot_one_variable(time, raw_err_vel, 'motor rotational velocity error (rad/s)', 'b');

elseif controller_str == "cascade"
    ref_offset = record_array(:, 2) * serial_scaling_factor;

    raw_err = record_array(:, 3) * serial_scaling_factor;
    err = record_array(:, 4) * serial_scaling_factor;

    raw_err_vel = record_array(:, 5) * serial_scaling_factor;
    err_vel = record_array(:, 6) * serial_scaling_factor;

    raw_inner_err = record_array(:, 7) * serial_scaling_factor;
    inner_err = record_array(:, 8) * serial_scaling_factor;

    raw_inner_err_vel = record_array(:, 9) * serial_scaling_factor;
    inner_err_vel = record_array(:, 10) * serial_scaling_factor;

    plot_one_variable(time, ref_offset, 'reference ball position (m)', 'k');
    plot_two_variables(time, raw_err, err, 'ball position error (m)');
    plot_two_variables(time, raw_err_vel, err_vel, 'ball velocity error (m/s)');
    plot_two_variables(time, raw_inner_err, inner_err, 'motor rotational angle error (rad)');
    plot_two_variables(time, raw_inner_err_vel, inner_err_vel, ...
        'motor rotational velocity error (rad/s)');

elseif (controller_str == "state space") || (controller_str == "state space tr")
    ref_offset = record_array(:, 2) * serial_scaling_factor;

    raw_offset = record_array(:, 3) * serial_scaling_factor;
    offset = record_array(:, 4) * serial_scaling_factor;

    offset_vel = record_array(:, 5) * serial_scaling_factor;

    raw_rotation = record_array(:, 6) * serial_scaling_factor;
    rotation = record_array(:, 7) * serial_scaling_factor;

    rotation_vel = record_array(:, 8) * serial_scaling_factor;
    
    if controller_str == "state space" 
        plot_one_variable(time, ref_offset, 'reference ball position (m)', 'k');
        plot_two_variables(time, raw_offset, offset, 'ball position (m)');
    else
        plot_ref_and_two_variables(time, ref_offset, raw_offset, offset, 'ball position (m)');
    end

    plot_one_variable(time, offset_vel, 'ball velocity (m/s)', 'r');
    plot_two_variables(time, raw_rotation, rotation, 'motor rotational angle (rad)');
    plot_one_variable(time, rotation_vel, 'motor rotational velocity (rad/s)', 'r');

end


pwm_model = record_array(:, end-1);
voltage_model = pwm_model / max_pwm * max_voltage;

pwm_real = record_array(:, end);
voltage_real = pwm_real / max_pwm * max_voltage;

plot_one_variable(time, voltage_model, 'modeled motor voltage (V)', 'm');
plot_one_variable(time, voltage_real, 'real motor voltage (V)', '#FFA500');


set(0, 'defaultAxesFontSize', 'default');




function plot_one_variable(t_real, y_real, label_for_y, color_for_y)
    figure('Name', 'Response of the real system', 'NumberTitle', 'off');
    set(gcf, 'Color', 'w'); hold on;

    plot(t_real, y_real, 'Color', color_for_y);

    xlabel('time (s)'); ylabel(label_for_y);
    grid minor; grid on;
end


function plot_two_variables(t_real, y_real_1, y_real_2, label_for_y)
    figure('Name', 'Responses of the real system', 'NumberTitle', 'off');
    set(gcf, 'Color', 'w'); hold on;

    plot(t_real, y_real_1, 'Color', 'b', 'DisplayName', 'measuring');
    plot(t_real, y_real_2, 'Color', 'r', 'DisplayName', 'estimation');

    xlabel('time (s)'); ylabel(label_for_y);
    grid minor; grid on;
    legend('Location', 'southeast');
end

function plot_ref_and_two_variables(t_real, y_real_ref, y_real_1, y_real_2, label_for_y)
    figure('Name', 'Responses of the real system', 'NumberTitle', 'off');
    set(gcf, 'Color', 'w'); hold on;

    plot(t_real, y_real_ref, 'Color', 'k', 'DisplayName', 'reference', 'LineStyle', '--');
    plot(t_real, y_real_1, 'Color', 'b', 'DisplayName', 'measuring');
    plot(t_real, y_real_2, 'Color', 'r', 'DisplayName', 'estimation');

    xlabel('time (s)'); ylabel(label_for_y);
    grid minor; grid on;
    legend('Location', 'southeast');
end