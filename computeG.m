function G=computeG(T)
% computeG.m
% G=computeG(T)
% G is a 6x1 vector of torques due to gravity forces for the current
%   timestep.
% T is the 4x4x6 array of link transform matrices for the robot for the
%   current time step.

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------

% author: dr.3


%Inialise output to be blank column
G=[0;0;0;0;0;0];


%Matrix of centre of mass'
    r = [ 0, -.15 , 0 , 1 ;
        -.1 , 0 , 0 , 1 ;
        0, 0 , 0, 1 ;
        0 , 0.1 , 0 ,1 ;
        0,0,0,1;
        0,0,-.05,1];


%denisty in kg per m^3
density = 7850 ;
gravity = [0, 0, -9.81, 0];

%Volumes
Link1 = pi*0.02*0.02*0.3    ; 
Link2 = 0.01*0.03*0.2       ; 
Link3 = 0.03*0.03*0.03      ;
Link4 = 0.2*0.01*0.01*pi    ;
Link5 = 0.03*0.03*0.03      ; 
Link6 = 0.1*0.01*0.01*pi    ;

%Mass , put as 6by1
Mass1 = Link1*density;
Mass2 = Link2*density;
Mass3 = Link3*density;
Mass4 = Link4*density;
Mass5 = Link5*density;
Mass6 = Link6*density;


%Stack the mass' into a single row for ues in the gravitational loop
Mass = [Mass1,Mass2,Mass3,Mass4,Mass5,Mass6];


%For each link , find the new condiotions using previous and the mass with
%graivty and accleration states for the gravitational resultant forces
for N = 1:6
        for i = 1:6
            G(N) = G(N) - Mass(i)*gravity*computeUij(T,i,N)*r(i,:)';
        end
end
   %show the size of the matrices in comments 




