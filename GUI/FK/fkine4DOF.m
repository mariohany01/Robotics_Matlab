function P = fkine4DOF(DH, angles, varargin)
% Calculates forward kinematics for the 4-DOF robot, 
% mapping 4 variable angles to the 6-DOF structure.

% Extract DH parameters from the input matrix
a     = DH(1, :);
d     = DH(2, :);
alpha = DH(3, :);
% The 4th row of DH (theta) in the main script is symbolic/mixed,
% so we'll build the 'th' vector manually based on the script's logic.

th_in = zeros(1, 4);

% Check if 'angle_offset' was provided as a third argument
if ~isempty(varargin)
    angle_offset = varargin{1};
    % Apply offsets to the raw angles
    th_in(1) = angles(1) + angle_offset(1);
    th_in(2) = angles(2) + angle_offset(2); % This is Th_2 + pi/2
    th_in(3) = angles(3) + angle_offset(3);
    th_in(4) = angles(4) + angle_offset(4);
else
    % No offset provided; assume angles are already final
    % (This is for the loop in btn_Forward_Callback)
    th_in = angles;
end

% Assemble the full 6-joint angle vector
% Your GUI's Theta_1, 2, 3, 4 map to Joints 1, 2, 4, 5
% Joints 3 and 6 are fixed at pi/2
th = [th_in(1), th_in(2), pi/2, th_in(3), th_in(4), pi/2];

% Calculate the overall transformation matrix
T = eye(4);
for i = 1:6
    % Standard DH Transformation Matrix
    T_i = [cos(th(i)) -sin(th(i))*cos(alpha(i))  sin(th(i))*sin(alpha(i)) a(i)*cos(th(i));
           sin(th(i))  cos(th(i))*cos(alpha(i)) -cos(th(i))*sin(alpha(i)) a(i)*sin(th(i));
           0           sin(alpha(i))             cos(alpha(i))            d(i);
           0           0                         0                        1];
    T = T * T_i; % Chain multiplication
end

% Return just the position vector (x, y, z)
P = T(1:3, 4);

end