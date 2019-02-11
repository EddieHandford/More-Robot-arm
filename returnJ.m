function J=returnJ
% returnJ.m
% J=returnJ()
% J  is a 4x4x6 array of pseudo-inertia matrices for the robot links.

% Unit information:
% ME40331 Robotics Engineering, University of Bath
% Dynamics and Control lab 2016/2017
% Dr. Jon du Bois

%--------------------------------------------------------------------------
% DO NOT MOFIFY ABOVE THIS LINE!!!! YOUR CODE GOES BELOW THIS LINE.
%--------------------------------------------------------------------------

% author: dr.3
% date: dd/mm/yyyy

J=zeros(4,4,6);                    % initialise pseudo inertia matrix array

density = 7850 ; 
%Mass of each link
M = [ pi*0.02*0.02*0.3*density , 0.01*0.03*0.2*density ,...
      0.03*0.03*0.03*density , 0.2*0.01*0.01*pi*density ,... 
      0.03*0.03*0.03*density , 0.1*0.01*0.01*pi*density] ;

%intialise the inertial matrixes
IXX = zeros(1,6);
IYY = zeros(1,6);
IZZ = zeros(1,6);

%Center of Mass table
r = [ 0, -.15 , 0 , 1 ; 
    -.1 , 0 , 0 , 1 ; 
    0, 0 , 0, 1 ; 
    0 , 0.1 , 0 ,1 ;  
    0,0,0,1; 
    0,0,-.05,1]; 

%Dimensions of links split up nicely
%LINKS


   [ IYY(1) , IXX(1) , IZZ(1)   ] =  cylinder(M(1) , 0.3 , 0.02);
   [ IXX(2) , IYY(2) , IZZ(2)   ] =  cuboid(M(2) , 0.2 , 0.03 , 0.01);
   [ IXX(3) , IYY(3) , IZZ(3)   ] =  cuboid(M(3) , 0.03 , 0.03 , 0.03);
   [ IYY(4) , IXX(4) , IZZ(4)   ] =  cylinder(M(4) , 0.2 , 0.01);
   [ IXX(5) , IYY(5) , IZZ(5)   ] =  cuboid(M(5) , 0.03 , 0.03 , 0.03);
   [ IZZ(6) , IYY(6) , IXX(6)   ] =  cylinder(M(6) , 0.1 , 0.01);

    
    %LINKS CORRECTED AFTER PARRELELL AXIS THEOREM
    IXX(1) = AXIS(IXX(1) , M(1) , 0.15) ;
    IZZ(1) = AXIS(IZZ(1) , M(1) , 0.15) ;
    
    IYY(2) = AXIS(IYY(2) , M(2) , 0.1)  ;
    IZZ(2) = AXIS(IZZ(2) , M(2) , 0.1)  ;
  
    IZZ(4) = AXIS(IZZ(4) , M(4), 0.1)   ; 
    IXX(4) = AXIS(IXX(4) , M(4), 0.1)   ;
    
    IXX(6) = AXIS(IXX(6) , M(6) , 0.05) ;
    IYY(6) = AXIS(IYY(6) , M(6) , 0.05) ;

    
    for i =1:6
        J(:,:,i) = [0.5*(-IXX(i) + IYY(i) + IZZ(i)) , 0 , 0 , M(i)*r(i,1) ;
        0, 0.5*(IXX(i) - IYY(i) + IZZ(i)) , 0 , M(i)*r(i,2) ; 
        0 , 0, 0.5*(IXX(i) + IYY(i) - IZZ(i)) , M(i)*r(i,3) ;  
        M(i)*r(i,1) , M(i)*r(i,2) , M(i)*r(i,3) , M(i) ] ;
         
    end
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
function[ax,by,cz] = cylinder(mass , H , R)
  
    ax = 0.5*mass*R*R ;
    by = (1/12)*mass*(3*R*R + H*H) ;
    cz = by ; 

end

function[ax,by,cz] = cuboid(mass , a , b , c)
      
    ax = (1/12)*mass*(b*b + c*c);
    by = (1/12)*mass*(a*a + c*c);
    cz = (1/12)*mass*(b*b + a*a);

end

    function[IOUT] = AXIS(IIN , mass , dist)
       
        IOUT = IIN + mass*dist*dist;
        
    end

end


%   3x6

%Cyclinder
%IXX = 0.5*M*
