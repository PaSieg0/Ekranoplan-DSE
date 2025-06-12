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
%K=[0,0,0,0];
disp('get those gainz:')
disp(K);
% Compute the state-space representation
sys = ss(A, B, C, D);
sys_cl = feedback(sys,K);

x0 = [118.4, 0.06, 0.05, 0.01];
xref = [116.5, 0.05, 0.04, 0];

error0 = x0-xref;

t = 0:0.1:200;
[y,tOut,error0] = initial(sys_cl,error0,t);
y_ = y+xref;
% Plot the response of the closed-loop system
stateNames = {'Velocity [m/s]', 'AoA [rad]', 'Pitch [rad]', 'Pitch Rate [rad/s]'};

% Create a new figure
figure;

% Get the number of states (columns) in y_
numStates = size(y_, 2);

% Loop through each state and create a subplot
for i = 1:numStates
    subplot(numStates, 1, i); % Create a subplot: (rows, columns, current_plot_index)
    plot(tOut, y_(:, i));    % Plot the i-th state (all rows, i-th column)
    xlabel('Time (s)');      % Label for x-axis
    ylabel(stateNames{i});   % Use the state name from the stateNames array
    title([stateNames{i} ]); % Title for each subplot
    grid on;                 % Enable grid
end

% Optional: Adjust overall figure properties if needed
sgtitle('Longitudinal stability response to non-equilibrium initial state with controller');






