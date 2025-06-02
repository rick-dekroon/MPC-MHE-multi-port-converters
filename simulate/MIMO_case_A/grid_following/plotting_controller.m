close all; clc;

vac_meas = out.vac.get('vac');
iac_meas = out.iac.get('y');
iac_ref = out.iac.get('r');

t = out.tout;
vac = squeeze(vac_meas.Values.Data);
y = squeeze(iac_meas.Values.Data);
r = squeeze(iac_ref.Values.Data);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen

yyaxis left
plot(t, vac, 'k-', 'LineWidth', 4);
ylabel('Voltage [V]', 'Interpreter', 'latex', 'FontSize', 32);
ylim([-325, 325])
set(gca, 'YColor', 'k');

yyaxis right
plot(t, y, 'b-', 'LineWidth', 4);
plot(t, r, 'r--', 'LineWidth', 4);
ylabel('Current [A]', 'Interpreter', 'latex', 'FontSize', 32);
ylim([-4, 4])
set(gca, 'YColor', 'k');

yt_right = get(gca, 'YTick');
n_ticks = numel(yt_right);
yyaxis left
yticks_left = linspace(-325, 325, n_ticks);
yticks(yticks_left);

xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 32);

title('AC Voltage with Measurement and Reference of $i_{ac}$', 'Interpreter', 'latex', 'FontSize', 32);

leg = legend('$v_{ac}$', '$y$', '$r$', 'Location', 'northeast');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);

set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');

yyaxis left
yl_left = ylim;
ylim(yl_left + [-0.01, 0.01] * range(yl_left));

yyaxis right
yl_right = ylim;
ylim(yl_right + [-0.01, 0.01] * range(yl_right));

hold off;

exportgraphics(gcf, 'figures/iac_controller.eps', 'ContentType', 'vector');

% Zoomed-in
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen

indexZoom = t >= 0.0095 & t <= 0.0105;
plot(t(indexZoom), y(indexZoom), 'b-', 'LineWidth', 4);
plot(t(indexZoom), r(indexZoom), 'r--', 'LineWidth', 4);

xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 32);
ylabel('Current [A]', 'Interpreter', 'latex', 'FontSize', 32);

title('Measurement and Reference of $i_{ac}$', 'Interpreter', 'latex', 'FontSize', 32);

leg = legend('$y$', '$r$', 'Location', 'northeast');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);

axis tight;
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');

hold off;

exportgraphics(gcf, 'figures/iac_controller_zoomed.eps', 'ContentType', 'vector');

iL_meas = out.iL.get('x');
iL_ref = out.iL.get('x_s');

t = squeeze(iL_meas.Values.Time);
x = squeeze(iL_meas.Values.Data);
x_s = squeeze(iL_ref.Values.Data);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen

plot(t, x(:,1), 'b-', 'LineWidth', 4);
plot(t, x(:,2), 'g-', 'LineWidth', 4);
plot(t, x_s(1,:), 'r--', 'LineWidth', 4);
plot(t, x_s(2,:), 'm--', 'LineWidth', 4);

ylabel('Current [A]', 'Interpreter', 'latex', 'FontSize', 32);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 32);

title('Measurement and Reference of $i_{L_1}$ and $i_{L_2}$', 'Interpreter', 'latex', 'FontSize', 32);

leg = legend('$x_1$', '$x_2$', '$x_1^{ss}$', '$x_2^{ss}$', 'Location', 'northeast');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);

axis tight;
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');

hold off;

exportgraphics(gcf, 'figures/iL_controller.eps', 'ContentType', 'vector');

iin_meas = out.iin.get('y');
iin_ref = out.iin.get('r');

t = squeeze(iin_meas.Values.Time);
y = squeeze(iin_meas.Values.Data);
r = squeeze(iin_ref.Values.Data);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen

plot(t, y, 'b-', 'LineWidth', 4);
plot(t, r, 'r--', 'LineWidth', 4);

ylabel('Current [A]', 'Interpreter', 'latex', 'FontSize', 32);
ylim([-1, 3]);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 32);

title('Measurement and Reference of $i_{in}$', 'Interpreter', 'latex', 'FontSize', 32);

leg = legend('$y$', '$r$', 'Location', 'northeast');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);

set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');

hold off;

exportgraphics(gcf, 'figures/iin_controller.eps', 'ContentType', 'vector');

u_signal = out.u.get('u');
u_s_signal = out.u.get('u_s');

t = squeeze(u_signal.Values.Time);
u = squeeze(u_signal.Values.Data);
u_s = squeeze(u_s_signal.Values.Data);

% Plot results
f = figure('Color', 'w'); hold on; grid on;
set(f, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);  % Full screen

plot(t, u(1,:), 'b-', 'LineWidth', 4);
plot(t, u(2,:), 'g-', 'LineWidth', 4);
plot(t, u_s(1,:), 'r--', 'LineWidth', 4);
plot(t, u_s(2,:), 'm--', 'LineWidth', 4);

ylabel('Duty Cycle [-]', 'Interpreter', 'latex', 'FontSize', 32);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 32);

title('Calculation and Steady-State Value of $u$', 'Interpreter', 'latex', 'FontSize', 32);

leg = legend('$u_1$', '$u_2$', '$u_1^{ss}$', '$u_2^{ss}$', 'Location', 'northeast');
set(leg, 'Interpreter', 'latex', 'FontSize', 48);

axis tight;
set(gca, 'FontSize', 24);
set(gca, 'TickLabelInterpreter', 'latex');

hold off;

exportgraphics(gcf, 'figures/u_controller.eps', 'ContentType', 'vector');
