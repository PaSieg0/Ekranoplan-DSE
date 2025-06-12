m = 360000;
rho = 1.225;
mu_b = m/(1.225*510*60); 
b    = 60.35; 
v    = 117; 
K_xx = 0.052*8; 
K_xz = -0.066*8; 
K_zz = 0.249*8; 

% Asymmetric Aerodynamic Coefficients (stability derivatives)
C_Ybeta_dot = 0.08052;
C_nbeta_dot = -0.0217637;
C_Ybeta     = -1.427208;
CL          = 0.6; 
C_Yp        = 0.002808141;
C_Yr        = 0;
C_lbeta     = -0.506641;
C_lp        = -0.46;
C_lr        = 1.042857;
C_nbeta     = 0.42974;
C_np        = -0.0259782;

% Control Derivatives (Aileron and Rudder)
C_Ydelta_a = 0;
C_Ydelta_r = 0.000123356;
C_ldelta_a = -0.18009;
C_ldelta_r = -0.000738186;
C_ndelta_a = 0;
C_ndelta_r = -0.00201328;


%stuff
P_asym = [
    (C_Ybeta_dot-2*mu_b)*b/v, 0                  , 0                       , 0;
    0                       , -b/(2*v)           , 0                       , 0;
    0                       , 0                  , -4*mu_b*K_xx*b/v      , 4*mu_b*K_xz*b/v;
    C_nbeta_dot*b/v         , 0                  , 4*mu_b*K_xz*b/v         , -4*mu_b*K_zz*b/v
];

Q_asym = [
    -C_Ybeta, -CL, -C_Yp, -(C_Yr-4*mu_b);
    0       , 0  , -1   , 0;
    -C_lbeta, 0  , -C_lp, -C_lr;
    -C_nbeta, 0  , -C_np, -C_lr  % Assuming the last element was C_lr based on typical matrix structure
                                % If it was meant to be C_lp, adjust accordingly.
];

R_asym = [
    -C_Ydelta_a, -C_Ydelta_r;
    0.0        , 0.0;
    -C_ldelta_a, -C_ldelta_r;
    -C_ndelta_a, -C_ndelta_r
];

% In MATLAB, the backslash operator (\) is used for matrix division,
% which is equivalent to solving a linear system (e.g., A\B is equivalent to inv(A)*B).
A_asym = P_asym \ Q_asym;
B_asym = P_asym \ R_asym;


C_asym = [
    1.0, 0.0, 0.0, 0.0;
    0.0, 1.0, 0.0, 0.0;
    0.0, 0.0, v/b, 0.0;
    0.0, 0.0, 0.0, v/b
];

D_asym = [
    0.0, 0.0;
    0.0, 0.0;
    0.0, 0.0;
    0.0, 0.0
];

% You can display the resulting matrices (optional)
disp('P_asym =');
disp(P_asym);
disp('Q_asym =');
disp(Q_asym);
disp('R_asym =');
disp(R_asym);
disp('A_asym (Asymmetric State Matrix) =');
disp(A_asym);
disp('B_asym (Asymmetric Input Matrix) =');
disp(B_asym);
disp('C_asym (Asymmetric Output Matrix) =');
disp(C_asym);
disp('D_asym (Asymmetric Feedforward Matrix) =');
disp(D_asym);

sys = ss(A_asym,B_asym, C_asym,D_asym);
% --- Simulation Parameters ---
t = 0:0.01:100; % Time vector (increased for longer dynamics)

% --- Step Input Parameters for EACH Input ---
% Input 1: Aileron
stepMagnitude_aileron = 0.1; % Magnitude for aileron deflection (radians)
stepStartTime_aileron = 2;    % Start time of aileron input (s)
stepDuration_aileron  = 1;  % Duration of aileron input (s)

% Input 2: Rudder
stepMagnitude_rudder = 0.1;  % Magnitude for rudder deflection (radians)
stepStartTime_rudder = 2;    % Start time of rudder input (s)
stepDuration_rudder  = 1;    % Duration of rudder input (s)

% --- Define the 2-input vector 'u' ---
% 'u' must have size (number of time steps) x (number of inputs)
u = zeros(length(t), 2); % Initialize with zeros for 2 inputs

% Apply the aileron step input (column 1)
u(t >= stepStartTime_aileron & t < (stepStartTime_aileron + stepDuration_aileron), 1) = stepMagnitude_aileron;

% Apply the rudder step input (column 2)
u(t >= stepStartTime_rudder & t < (stepStartTime_rudder + stepDuration_rudder), 2) = stepMagnitude_rudder;

% Zero initial conditions as requested
initialConditions = [0; 0; 0; 0]; % [delta_beta; delta_phi; delta_p; delta_r]

% Simulate the system response using lsim
[y, t_out, x] = lsim(sys, u, t, initialConditions);

% Plotting the results
figure; % Create a new figure window

% Define state variable names for clarity in plots
stateNames = {'sideslip', 'roll', 'roll rate', 'yaw rate'};

% Plot each state variable over time
for i = 1:size(x, 2)
    subplot(size(x, 2), 1, i); % Create a subplot for each state
    plot(t_out, x(:, i), 'b'); % Plot state variable
    hold on; % Keep current plot for adding the input line

    % Plot the inputs on the same subplot for context (scaled to fit)
    % Only plot if there's actually an input
    input_legend_entries = {};
    if any(u(:, 1) ~= 0) % Aileron input (column 1)
        % Scale based on current state's max value for better visibility
        scale_factor_aileron = 0.5 * (max(abs(x(:,i))) / max(abs(u(:,1))));
        plot(t_out, u(:, 1) * scale_factor_aileron, 'r--');
        input_legend_entries = [input_legend_entries, 'Aileron Input'];
    end
    if any(u(:, 2) ~= 0) % Rudder input (column 2)
        % Scale based on current state's max value for better visibility
        scale_factor_rudder = 0.5 * (max(abs(x(:,i))) / max(abs(u(:,2))));
        plot(t_out, u(:, 2) * scale_factor_rudder, 'g:');
        input_legend_entries = [input_legend_entries, 'Rudder Input'];
    end

    if ~isempty(input_legend_entries)
        legend('State Response', input_legend_entries{:}, 'Location', 'best');
    else
        legend('State Response', 'Location', 'best');
    end
    hold off;
    grid on;
    title(stateNames{i});
    xlabel('Time (s)');
    ylabel('Amplitude');
end

sgtitle(['Asymmetric Dynamic Stab response']);