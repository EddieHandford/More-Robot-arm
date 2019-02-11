function T=computeT(q)
% computeT.m
% T = computeT(q)
% T is the 4x4x6 array of homogeneous transform matrices for all 6 links
%   e.g. T(:,:,3) is the matrix T23 (not T03!)
% q is a 6x1 vector of joint angles for the current time step, in radians

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------

% author: dr.3
% date: 16/11/2017

%Initialise parameters
T=zeros(4,4,6);

a = [0, 0.2, 0,   0, 0, 0  ];

d = [0.3, 0, 0, 0.2, 0, 0.1];
%planes angular offsets
alpha = [pi/2, 0, pi/2, -pi/2, pi/2, 0];


%Output each T matrix for each joint (generic form represented)
for i = 1:6
    T(:,:,i) = [cos(q(i)) , -sin(q(i))*cos(alpha(i)) , sin(q(i))*sin(alpha(i)), a(i)*cos(q(i));
        sin(q(i)), cos(q(i))*cos(alpha(i)) , -cos(q(i))*sin(alpha(i)), a(i)*sin(q(i));
        0, sin(alpha(i)) , cos(alpha(i)) , d(i);
        0,0,0,1];
end

    
    
    