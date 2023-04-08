syms x1 x2 x3

simple_rot = eul2rotm([0.1, -0.2, 0.3]);
simple_linear = simple_rot * [x1; x2; x3];

simple_nonlinear = [x1; x2; x3^2];

nonlinear_0 = [sin(x1); cos(x2); tan(x3)];

nonlinear_1 = [exp(x1); x1^4 + x2^2; x3^3];

fns = [simple_linear, simple_nonlinear, nonlinear_0, nonlinear_1];
fn_names = ["simple linear", "simple nonlinear", ...
            "nonlinear_0", "nonlinear_1"];

x_init = [0;0;0];
j_init = eye(3);

N = 150;
steps = linspace(1, N, N);

for sigma = [0.001, 0.01, 0.05, 0.1]
    j_errors_fig = figure;
    hold on
    y_errors_fig = figure;
    hold on
    
    for i = 1 : length(fn_names)
        [j_errors, y_errors] =  simulate(fns(:, i), x1, x2, x3, x_init, ... 
                                         j_init, fn_names(i), sigma, N);
        figure(j_errors_fig);
        plot(steps, j_errors, 'DisplayName', fn_names(i), 'LineWidth', 2);
        hold on

        figure(y_errors_fig);
        plot(steps(2:N), y_errors(2:N), 'DisplayName', fn_names(i), 'LineWidth', 2);
        hold on
    end

    figure(j_errors_fig);
    grid on
    title_str = sprintf(['$$||J_{estimate} - J_{real}||$$' ...
                         '(frobenius norm) $$\\sigma = %0.3f$$'], sigma);
    title(title_str, 'interpreter','latex')
    legend

    file_name = sprintf('jacobian_error_sigma=%0.3f.png', sigma);
    saveas(gcf, file_name)

    figure(y_errors_fig);
    grid on
    title_str = sprintf(['$$||y_{estimate} - y_{real}||$$' ...
                         ' $$\\sigma = %0.3f$$'], sigma);
    title(title_str, 'interpreter','latex')
    legend

    file_name = sprintf('y_error_sigma=%0.3f.png', sigma);
    saveas(gcf, file_name)
end

close all

function [j_errors, y_errors] = simulate(f, x1, x2, x3, x_init, j_init, ...
                                         fn_name, sigma, N)
    rng('default');
    disp(fn_name)
    j_matrix = jacobian(f);
    
    x1 = x_init(1); x2 = x_init(2); x3 = x_init(3);

    x_val = x_init;
    y_val = double(subs(f));
    y_estimate = y_val;

    j_estimate = j_init;
    j_real = double(subs(j_matrix));
    
    j_errors = zeros(1, N);
    y_errors = zeros(1, N);
    for step = 1 : N
        j_errors(step) = norm(j_estimate - j_real, 'fro');
        y_errors(step) = norm(y_estimate - y_val);

        delta_x = normrnd(0, sigma, [3, 1]);
        x_val = x_val + delta_x;
        x1 = x_val(1); x2 = x_val(2); x3 = x_val(3);
       
        y_val_next = double(subs(f));
        delta_y = y_val_next - y_val;
        y_estimate = y_val + j_estimate * delta_x;
        y_val = y_val_next;

        j_estimate = j_estimate + ... 
                     (delta_y - j_estimate * delta_x) * delta_x' / ...
                     (delta_x' * delta_x);
        j_real = double(subs(j_matrix));
    end
end