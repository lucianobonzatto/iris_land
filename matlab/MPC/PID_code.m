%clear;

t_in_data = 0:0.01:10;  % Vetor de tempo ajustado
step_data = 2 * ones(size(t_in_data));
ramp_data = t_in_data;
seno_data = sin(2*pi*0.5*t_in_data);
step2_data = (t_in_data>=0)*1 + (t_in_data>=3)*1 + (t_in_data>=6)*(-2);

load('DATA.sid', '-mat');
%tfr = Model{1};
%tfz = Model{2};
%tfy = Model{4};

tfx = Model{3};
%tf_x_auto_gains = pidtune(tfx, 'PID');
%pidTuner(tfx, 'PID');
tf_x_auto_gains = C;
sys_x_closed = feedback(tf_x_auto_gains * tfx, 1);

%disp('=== Informações da Resposta ao Degrau (Eixo X) ===');
%disp(stepinfo(sys_x_closed));

in_data = seno_data;
[out_data, t_out_data] = lsim(sys_x_closed, in_data, t_in_data);

% Plotar entrada e saída
figure(1);
plot(t_in_data, in_data, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Entrada (Referência)');
hold on;
plot(t_out_data, out_data, 'r', 'LineWidth', 1.5, 'DisplayName', 'Saída do Sistema');
hold off;
title('Resposta do Sistema em Malha Fechada - Eixo X');
xlabel('Tempo (s)');
ylabel('Amplitude');
legend('show', 'Location', 'best');
grid on;
