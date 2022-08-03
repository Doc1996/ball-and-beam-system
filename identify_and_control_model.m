%%  RESET SCRIPT


clear;
close all;
clc;

warning ('off', 'all');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultAxesFontSize', 14);



%% MOTOR MODEL IDENTIFICATION DATA


global t_real u_real y_real y_real_0;

serial_scaling_factor = 0.001;

record_folder_name = "records_general";
record_file_name = "model identification with 5.7 ms sampling.txt";
record_range = "A2:E530";

record_file_rel_path = record_folder_name + "/" + record_file_name;
record_array = table2array(readtable(record_file_rel_path, 'ReadVariableNames', ...
    false, 'Range', record_range));


t_readed = (record_array(:, 1) - record_array(1, 1)) * serial_scaling_factor;
y_readed = record_array(:, 2) * serial_scaling_factor;
u_readed = record_array(:, 3);

t_real = transpose(linspace(t_readed(1), t_readed(end), length(t_readed)));
u_real = interp1(t_readed, u_readed, t_real);
y_real = interp1(t_readed, y_readed, t_real);
y_real_0 = y_real(1);

t_sample = t_real(end) - t_real(end - 1);


figure('Name', 'Responses of the real system', 'NumberTitle', 'off');
set(gcf, 'Color', 'w'); hold on;

plot(t_readed, u_readed, 'DisplayName', 'excitation');
plot(t_readed, y_readed, 'DisplayName', 'real response');

xlabel('time (s)'); ylabel('amplitude');
xlim([0, 2]); grid minor; grid on;
legend('Location', 'southeast');



%% PARAMETRIC MOTOR MODEL IDENTIFICATION


% DC motor armature circuit equations:
% V = R * I + L * d(I)/d(t) + K_e * omega
% K_m * I = J * d(omega)/d(t) + B * omega

% V = I * (L * s + R) + K_e * omega
% I = (J * s + B) * omega / K_m

% V = ((J * s + B) * (L * s + R) + K_e * K_m) / K_m * omega

% omega / V = K_m / ((J * L) * s^2 + (J * R + B * L) * s + (B * R + K_e * K_m))

% the ideal DC motor model for rotational velocity is second order with real poles and
% no zeros

% criteria suitable for determining the transfer function are the IAE and ISE criteria
% because they have no time constraints


global criterion_str model_order_str;

criteria_str_list = ["IAE", "ISE"];
model_orders_str_list = ["1", "2"];

G_model_red_IAE_cell = cell(1, length(model_orders_str_list));
G_model_red_ISE_cell = cell(1, length(model_orders_str_list));


close all;

figure('Name', 'Responses of the identified systems', 'NumberTitle', 'off');
set(gcf, 'Color', 'w'); hold on;

plot(t_real, y_real, 'DisplayName', 'real response');

xlabel('time (s)'); ylabel('motor rotational velocity (rad/s)');
xlim([0, 2]); grid minor; grid on;
legend('Location', 'southeast');


for i = 1:length(criteria_str_list)
    criterion_str = criteria_str_list(i);

    for j = 1:length(model_orders_str_list)
        model_order_str = model_orders_str_list(j);
    
        if model_order_str == "1"
            zeros_0 = [];
            poles_0 = -5;
        elseif model_order_str == "2"
            zeros_0 = [];
            poles_0 = [-5; -10];
        end
    
        [num_of_G_0, den_of_G_0] = zp2tf(zeros_0, poles_0, 1);
        coeffs_0 = den_of_G_0;


        opt_function_options = optimset('Display', 'final');
        opt_function_options = optimset(opt_function_options, 'MaxIter', 1000);
        [coeffs, ~] = fminsearch('optimize_model', coeffs_0, opt_function_options);
    
        if model_order_str == "1"
            num_of_G = [0, 1];
            den_of_G = coeffs(1:2);
        elseif model_order_str == "2"
            num_of_G = [0, 0, 1];
            den_of_G = coeffs(1:3);
        end


        G_model = tf(num_of_G, den_of_G);
        G_model_red = minreal(G_model);

        [num_of_G_red, den_of_G_red] = tfdata(G_model_red, 'v');
        [zeros_red, poles_red, gain_red] = tf2zp(num_of_G_red, den_of_G_red);

        t_ideal = t_real;
        y_ideal = lsim(G_model, u_real, t_real, y_real_0);

        fit_percent = (1 - norm(y_real - y_ideal) / norm(y_real - mean(y_real)));


        if criterion_str == "IAE"
            G_model_red_IAE_cell{j} = G_model_red;
        elseif criterion_str == "ISE"
            G_model_red_ISE_cell{j} = G_model_red;
        end

        if model_order_str == "1"
            plot(t_real, y_ideal, 'DisplayName', sprintf(['%s identification for s = '...
            '%0.2f (%0.2f%% fit)'], criterion_str, poles_red(1), fit_percent * 100));
        elseif model_order_str == "2"
            plot(t_real, y_ideal, 'DisplayName', sprintf(['%s identification for s_1 = '...
            '%0.2f and s_2 = %0.2f (%0.2f%% fit)'], criterion_str, poles_red(2), ...
            poles_red(1), fit_percent * 100));
        end
    end
end


% for satisfactory accuracy and maximum simplicity, the first-order model of either IAE or
% ISE criteria is selected



%% MODELING OF THE BALL AND BEAM SYSTEM


% equation of the ball and beam system:
% (J / R^2 + m) * d(x)^2/d(t) + m * g * sin(phi) - m * x * (d(phi)/d(t))^2 = 0

% (d(phi)/d(t))^2 is ignored, and sin(phi) is for small angles approximated by phi:
% (J / (m * R^2) + 1) * x * s^2 + g * phi = 0
% J = m * R^2 * 2 / 5

% x / phi = - 5/7 * g / s^2


close all;

grav = 9.81;

s = tf('s');
G_vel = ss(G_model_red_IAE_cell{1});

fprintf("\nMotor velocity state space:\n");
describe_model(G_vel)

G_pos = G_vel / s;

fprintf("\nMotor position state space:\n");
describe_model(G_pos)

G_ball = ss(- 5/7 * grav / s^2);
G_sys = G_ball * G_pos;

fprintf("\nball position state space:\n");
describe_model(G_sys)



%% DESIGN OF THE EXTENDED KALMAN FILTER


% the ZOH discretization best describes the real system because in control and measurement
% there is a wait between steps

% approximate discretization will be used because it is not possible to do otherwise for a
% non-linear system

% phi / V = tf(G_pos) = K1_vel / (T1_vel * s + 1) / s

% T1_vel * phi'' + phi' = K1_vel * V
% 7/5 * x'' + g * sin(phi) = 0

% x = x1
% x' = x2
% phi = x3
% phi' = x4

% x1' = x2
% x2' = - 5/7 * g * sin(x3)
% x3' = x4
% x4' = - (1 / T_vel) * x4 + (K_vel / T_vel) * V

% x_vec = f(x_vec) + g(x_vec) * u_vec;


[G_vel_num, G_vel_den] = tfdata(G_vel, 'v');
K1_vel = dcgain(G_vel);
T1_vel = 1 / G_vel_den(end);

t_sample = 0.012;

syms x1 x2 x3 x4;

x_sym = [x1; x2; x3; x4];
f_sym = [x2; - 5/7 * grav * sin(x3); x4; - x4 / T1_vel];
g_sym = [0; 0; 0; K1_vel / T1_vel];
h_sym = [x1; x3];


der_f_sym = simplify(jacobian(f_sym, x_sym));
der_h_sym = simplify(jacobian(h_sym, x_sym));

A_sym = der_f_sym;
B_sym = g_sym;
C_sym = der_h_sym;

Phi_sym = eye(length(x_sym)) + A_sym * t_sample;
Gamma_sym = B_sym * t_sample;
H_sym = C_sym;


fprintf("\nFunctions f, g and h:\n\n");
disp(vpa(f_sym, 5));
disp(vpa(g_sym, 5));
disp(vpa(h_sym, 5));

fprintf("\nMatrices Phi, Gamma and H:\n\n");
disp(vpa(Phi_sym, 5));
disp(vpa(Gamma_sym, 5));
disp(vpa(H_sym, 5));



%% DESIGN OF THE STATE SPACE CONTROLLER


% the pole transformation between domains has the form of z = e^(t_sample * s)


sigma_m = 0.3;
t_1 = 2.5;

zeta = sqrt((log(sigma_m))^2 / ((log(sigma_m))^2 + pi^2));

omega_n = 4.6/(t_1*zeta);

den_of_G_cl = [1, 2*zeta*omega_n, omega_n^2];
roots_of_G_cl_caract_poly = roots(den_of_G_cl);

s_p1 = roots_of_G_cl_caract_poly(1);
s_p2 = roots_of_G_cl_caract_poly(2);


poles_cont = [s_p1; s_p2; real(s_p1) * 8; real(s_p1) * 10];
poles_cont_tr = [s_p1; s_p2; real(s_p1) * 8; real(s_p1) * 10; real(s_p1) * 0.5];

poles_disc = exp(poles_cont * t_sample);
poles_disc_tr = exp(poles_cont_tr * t_sample);

x_lin = [0; 0; 0; 0];

H_lin = double(H_sym);

Phi_lin = double(subs(Phi_sym, x_sym, x_lin));
Phi_lin_tr = [Phi_lin, zeros(length(x_sym), 1); - H_lin(1, :), 1];

Gamma_lin = double(Gamma_sym);
Gamma_lin_tr = [Gamma_lin; 0];

K_lin = place(Phi_lin, Gamma_lin, poles_disc);
K_lin_tr = place(Phi_lin_tr, Gamma_lin_tr, poles_disc_tr);


fprintf("\nContinuous system poles:\n\n");
disp(vpa(poles_cont, 5));

fprintf("Discrete system poles:\n\n");
disp(vpa(poles_disc, 5));

fprintf("State space controller:\n\n");
disp(vpa(K_lin, 5));


fprintf("\nContinuous system tracking poles:\n\n");
disp(vpa(poles_cont_tr, 5));

fprintf("Discrete system tracking poles:\n\n");
disp(vpa(poles_disc_tr, 5));

fprintf("State space tracking controller:\n\n");
disp(vpa(K_lin_tr, 5));



%% DESIGN OF THE FEEDBACK LINEARIZED CONTROLLER


syms v;

Lf_h_sym = jacobian(h_sym, x_sym.') * f_sym;
Lg_h_sym = jacobian(h_sym, x_sym.') * g_sym;

Lf2_h_sym = jacobian(Lf_h_sym, x_sym.') * f_sym;
LgLf_h_sym = jacobian(Lf_h_sym, x_sym.') * g_sym;

Lf3_h_sym = jacobian(Lf2_h_sym, x_sym.') * f_sym;
LgLf2_h_sym = jacobian(Lf2_h_sym, x_sym.') * g_sym;

Lf4_h_sym = jacobian(Lf3_h_sym, x_sym.') * f_sym;
LgLf3_h_sym = jacobian(Lf3_h_sym, x_sym.') * g_sym;


z_sym = [h_sym; Lf_h_sym; Lf2_h_sym; Lf3_h_sym];
    
alpha_sym = Lf4_h_sym;
beta_sym = LgLf3_h_sym;

u_sym = (- alpha_sym + v) / beta_sym;


A_ext = [zeros(length(x_sym) - 1, 1), eye(length(x_sym) - 1); zeros(1, length(x_sym))];
B_ext = [zeros(length(x_sym) - 1, 1); 1];
C_ext = [1, 0, 0, 0;
         0, 0, 1, 0];

Phi_ext = eye(length(x_sym)) + A_ext * t_sample;
Gamma_ext = B_ext * t_sample;
H_ext = C_ext;

K_ext = place(Phi_ext, Gamma_ext, poles_disc);


fprintf("\nLie's derivatives:\n\n");
disp(vpa(Lf_h_sym, 5));
disp(vpa(Lg_h_sym , 5))

disp(vpa(Lf2_h_sym, 5));
disp(vpa(LgLf_h_sym , 5));

disp(vpa(Lf3_h_sym, 5));
disp(vpa(LgLf2_h_sym , 5));

disp(vpa(Lf4_h_sym, 5));
disp(vpa(LgLf3_h_sym , 5));

fprintf("\nFunction z:\n\n");
disp(vpa(z_sym , 5));

fprintf("\nFeedback linearized input:\n\n")
disp(vpa(u_sym , 5));

fprintf("\nFeedback linearized controller:\n\n");
disp(vpa(K_ext, 5));


set(0, 'defaultAxesFontSize', 'default');