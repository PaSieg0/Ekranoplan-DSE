%
v = 117;
m = 360000;
c = 8.321;
S = 510.97;
lh = 31.09;
mu_c = 49.12;

%
C_X0 = 0;
C_Xu = -0.0557934;
C_Xalpha = 0.1965;
C_Xalpha_dot = 0;
C_Xq = 0;
C_Xdelta_e = -0.0014;
C_Z0 = -1.76;
C_Zu = -1.0;
C_Zalpha = -5.7708;
C_z_alpha_dot = 0;
C_zq = 0;
C_zdelta_e = -0.000862978;
C_mu = 0;
C_malpha = -0.8445;
C_m_alpha_dot = -5.19584;
C_mq = -52.0266;
C_mdelta_e = -0.02811995;
K_yy = 2.3345;
x_cg = 0.25*c;


P_sym = [-2*mu_c*c/v, 0                 , 0                   , 0;
         0          , (C_z_alpha_dot-2*mu_c)*c/v, 0           , 0;
         0          , 0                 , -c/v        , 0;
         0          , C_m_alpha_dot*c/v , 0           , -2*mu_c*K_yy*c/v
        ];

Q_sym = [-C_Xu, -C_Xalpha, -C_Z0, 0;
         -C_Zu, -C_Zalpha, C_X0 , -(C_zq+2*mu_c);
         0    , 0        , 0    , -1;
         -C_mu, -C_malpha, 0    , -C_mq
         ];

R_sym = [-C_Xdelta_e;
         -C_zdelta_e;
         0.0;
         -C_mdelta_e
         ];

A_sym = P_sym \ Q_sym; 
B_sym = P_sym \ R_sym;

C_sym = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, 0;
         0, 0, 0, v/c
         ];

D_sym = [ 0;
          0;
          0;
          0
        ];

% You can display the resulting matrices (optional)
disp('P_sym =');
disp(P_sym);
disp('Q_sym =');
disp(Q_sym);
disp('R_sym =');
disp(R_sym);
disp('A_sym (State Matrix) =');
disp(A_sym);
disp('B_sym (Input Matrix) =');
disp(B_sym);
disp('C_sym (Output Matrix) =');
disp(C_sym);
disp('D_sym (Feedforward Matrix) =');
disp(D_sym);

sys = ss(A_sym,B_sym, C_sym,D_sym);
% Simulate the state-space system response to an input
t = 0:0.01:400; % Time vector
% --- Step Input Parameters ---
stepMagnitude = 0.1; % Example: A 0.1 radian (approx 5.7 degrees) elevator deflection
stepStartTime = 1;   % Start the step at 1 second
stepDuration = 1.5;    % Hold the step for 5 seconds

% Create the input vector 'u' for the step input
u = zeros(size(t)); % Initialize input to zero
% Apply the step input
u(t >= stepStartTime & t < (stepStartTime + stepDuration)) = stepMagnitude;

% Zero initial conditions as requested
initialConditions = [0; 0; 0; 0];
% Simulate the system response using lsim
% [y, t_out, x] = lsim(sys, u, t, initialConditions);
% If D_sym is all zeros, y will be C*x. If D_sym is non-zero, it handles direct input-output coupling.
[y, t_out, x] = lsim(sys, u, t, initialConditions);

% Plotting the results
figure; % Create a new figure window

% Define state variable names for clarity in plots
stateNames = {'V', 'AoA', 'pitch', 'qc/V'};

% Plot each state variable over time
for i = 1:size(x, 2)
    subplot(size(x, 2), 1, i); % Create a subplot for each state
    plot(t_out, x(:, i), 'b'); % Plot state variable
    hold on; % Keep current plot for adding the input line
    % Plot the input on the same subplot for context (scaled to fit)
    plot(t_out, u * (max(x(:,i))/max(u))/2, 'r--'); % Plot input, scaled roughly for visibility
    hold off;
    grid on;
    title(stateNames{i});
    xlabel('Time (s)');
    ylabel('Amplitude');
    legend('State Response', 'Elevator Input (scaled)', 'Location', 'best');
end

sgtitle(['Longitudinal Dynamic Stab (Elevator deflection: ' num2str(stepMagnitude) ', Duration: ' num2str(stepDuration) 's)']); % Super title for the figure