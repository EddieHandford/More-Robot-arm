function U=computeUijk(T,i,j,k)
% computeUijk.m
% U=computeUijk(T,i,j,k)
% U is a 4x4 matrix describing the second derivative of the transform from
%   the base frame to the ith link with respect to the jth and kth
%   generalised coordinates.
% T is a 4x4x6 array of link transform matrices for the robot for the
%   current time step.
% i defines the link
% j  defines the first generalised coordinate
% k  defines the second generalised coordinate

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------

% author: dr.3


%4x4 identity
U = eye(4,4);
%derivative matrix
M = [0, -1, 0, 0;
     1, 0, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 0];
 
 
%If you are looking at a joint that is further on, then it is zero, as
%later joints do not affect previous joints.
 if i < j || i < k
     
     U = 0;
     
 else
     for N = 1:i
         
         if j==N && k~=N || k==N && j~=N
             
             U = U*M*T(:,:,N);
             
         elseif j==N && k==N
             
             U = U*(M^2)*T(:,:,N);
             
         else
             U = U*T(:,:,N);
             
         end
     end
 end
end

