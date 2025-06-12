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
% C_lr is already defined, no need to duplicate if it's the same

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

Q = [1,0,0,0;
     0,1,0,0;
     0,0,1,0;
     0,0,0,1];

R = [1,0;
     0,1];

[K, S, P] = lqr(A_asym, B_asym, Q, R);

disp('get those gainz:')
disp(K);

sys_cl = feedback(sys,K);

x0 = [0.1, 0.1, 0.05, 0.05];

t = 0:0.1:200;

[y,tOut,x0] = initial(sys_cl,x0,t);

% Plot the response of the closed-loop system
stateNames = {'sideslip', 'roll', 'roll rate', 'yaw rate'};

% Create a new figure
figure;

% Get the number of states (columns) in y_
numStates = size(y, 2);

% Loop through each state and create a subplot
for i = 1:numStates
    subplot(numStates, 1, i); % Create a subplot: (rows, columns, current_plot_index)
    plot(tOut, y(:, i));    % Plot the i-th state (all rows, i-th column)
    xlabel('Time (s)');      % Label for x-axis
    ylabel(stateNames{i});   % Use the state name from the stateNames array
    title([stateNames{i} ]); % Title for each subplot
    grid on;                 % Enable grid
end

% Optional: Adjust overall figure properties if needed
sgtitle('Asymmetric dyamic stab with LQR controller');