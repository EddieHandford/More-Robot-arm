function C=computeC(J,T,qt)
% computeC.m
% C=computeC(J,T,qt)
% C  is a 6x1 vector of torques due to centrifugal, centripetal, Coriolis
%    and precessional forces for the current timestep.
% J  is a 4x4x6 array of pseudo-inertia matrices for the robot links.
% T  is a 4x4x6 array of link transform matrices for the robot for the
%    current time step.
    % qt is a 6x1 vector of joint velocities (angluar velocities) for the
%    current time step, in radians/s.

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------


% author: dr.3

%Set up intial blank matrix for all links gyroscopic forces

Ci = zeros(6, 6, 6);
C = zeros(6, 1);

%loop through to cover ever matrix element, previous result + the sum of
%the diagonal of the larger matrices (finding the frames derivartive (UIJ)
%and the second derivative (UIJK)
for a = 1:6
    for b = 1:6
        for c = 1:6
            for d = 1:6
                Ci(b, c, a) = Ci(b, c, a) + trace(computeUijk(T, d, b ,c)*J(:,:,d)*(computeUij(T,d,a))');
            end
        end
    end
    
    %Output matrix of gyroscopic forces
C(a) = qt'*Ci(:, :, a)*qt;
end


