function crit_value = optimize_model(coeffs)
    global t_real u_real y_real y_real_0 criterion_str model_order_str;

    if model_order_str == "1"
        num_of_G = [0, 1];
        den_of_G = coeffs(1:2);
    elseif model_order_str == "2"
        num_of_G = [0, 0, 1];
        den_of_G = coeffs(1:3);
    end
    
    G_model = tf(num_of_G, den_of_G);
    
    y_ideal = lsim(G_model, u_real, t_real, y_real_0);
    
    y_error = y_ideal - y_real;
    
    if criterion_str == "IAE"
        crit_value = trapz(t_real, abs(y_error));
    elseif criterion_str == "ISE"
        crit_value = trapz(t_real, y_error.^2);
    end

    t_id_final = t_real(end);

    if exist('t_exist', 'var') == 1
        if max(t_real) >= t_id_final
            crit_value = max(crit_value);
        end
    end
end