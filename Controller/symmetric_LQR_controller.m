% Define matrices A, B, C, D
 
A = [8.56146015e-03, -2.10399095e-02,  8.42803062e-02, 0;
     1.61337624e-02,  9.36061499e-01,  0, -1.45380375e+01;
     0, 0, 0, -1.45380375e+01;
    -5.94627222e-04,  5.77467784e+01,  0, -7.83447148e+02];

B = [2.25872674e-04;
     1.39230795e-04;
     0;
     7.53349544e-01];

C = [1, 0, 0, 0;
     0, 1, 0, 0;
     0, 0, 1, 0;
     0, 0, 0, 14.5380375];

D = [0;
     0;
     0;
     0];

% Define Q and R matrices
Q = eye(4) * 10; 
R = 1; 

% Compute LQR controller
[K, S, P] = lqr(A, B, Q, R);

disp('get those gainz:')
disp(K);
% Compute the state-space representation
sys = ss(A, B, C, D);
sys_cl = feedback(sys,K);

x0 = [0.1,0.05,0.02,0];
%sys_cl = ss(A-K*B,B,C,D);

t = 0:0.1:100;
[y,tOut,x0] = initial(sys_cl,x0,t);
% Plot the response of the closed-loop system
figure;
plot(tOut, y);
xlabel('Time (s)');
ylabel('System Response');
title('Closed-Loop System Response');
grid on;

%initial states: Cruise speed, cruise AOA, cruise pitch angle, cruise pitch
%rate

%{
x0 = [116.4, 0.06, 0.05, 0.01];
xref = [116.5, 0.05, 0.04, 0];

% Simulation parameters
t = 0:0.01:100; % Time vector
num_steps = length(t);
x = zeros(num_steps, length(x0)); % Preallocate state trajectory
x(1, :) = x0'; % Set initial state

% Simulation loop
for i = 2:num_steps
    % Compute control input
    u = -K * (x(i-1, :)' - xref); % Control law
    % Update state using the continuous-time state-space model
    x(i, :) = (A * x(i-1, :)' + B * u) * 0.01 + x(i-1, :)'; % Euler integration
end

% Plot the results
figure;
plot(t, x(:, 1), 'b-', 'DisplayName', 'State 1 (Position)');
hold on;
plot(t, xref(1) * ones(num_steps, 1), 'r--', 'DisplayName', 'Reference State 1');
xlabel('Time (s)');
ylabel('State Value');
legend;
title('LQR Controller Tracking Reference State');
grid on;
%}







%{
poles = pole(sys_cl);
disp('Poles of the closed-loop system:');
disp(poles);
step(sys_cl);
%}


