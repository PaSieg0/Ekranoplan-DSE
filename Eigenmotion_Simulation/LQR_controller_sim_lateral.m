% Matrix A
A = [2.31823994e-01, -9.74590799e-02, -4.56131352e-04,  3.88318835e+00;
     -0.00000000e+00, -0.00000000e+00, -3.85703036e+00, -0.00000000e+00;
      2.19172552e+04, -0.00000000e+00,  1.98995680e+04, -4.51139078e+04;
     -1.87036564e+04,  4.75792683e+01,  1.12403837e+03,  1.80038056e+04];

% Matrix B
B = [-0.00000000e+00, -2.00369493e-05;
      -0.00000000e+00, -0.00000000e+00;
       7.79047545e+03,  3.19338669e+01;
      -0.00000000e+00,  8.71041312e+01];

% Matrix C
C = [1.00000000, 0.00000000, 0.00000000, 0.00000000;
      0.00000000, 1.00000000, 0.00000000, 0.00000000;
      0.00000000, 0.00000000, 1.92851518, 0.00000000;
      0.00000000, 0.00000000, 0.00000000, 1.92851518];

% Matrix D
D = [0.00000000, 0.00000000;
      0.00000000, 0.00000000;
      0.00000000, 0.00000000;
      0.00000000, 0.00000000];

% Define Q and R matrices
Q = eye(4) * 10; 
R = [1,0;
     0,1]; 

% Compute LQR controller
[K, S, P] = lqr(A, B, Q, R);
%K=[0,0,0,0;0,0,0,0];
disp('get those gainz:')
disp(K);

% Compute the state-space representation
sys_asym = ss(A, B, C, D);
sys_cl_asym = feedback(sys_asym,K);

x0 = [-0.05, 0.05, 0.01, -0.03];

t = 0:0.1:200;
[y,tOut,x0] = initial(sys_cl_asym,x0,t);

stateNames = {'Sideslip [rad]', 'Roll [rad]', 'Roll rate [rad/s]', 'Yaw Rate [rad/s]'};

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
