 function Jm = Jm(e)
 
 % JM The relation between Euler-angle rate and body-axis rates.
%   JM(E) gives the transition matrix from Euler-angle rate to body-axis 
%   rates (i.e., gyrometer true values) giving current euler angles 
%   E = [roll;pitch;yaw].
%
%   Ref: https://aviation.stackexchange.com/questions/83993/the-relation-between-euler-angle-rate-and-body-axis-rates

%   Copyright@ Qin Shi 2021.

r  = e(1); %roll
p  = e(2); %pitch
y  = e(3); %yaw

sr = sin(r); 
cr = cos(r);
sp = sin(p);
cp = cos(p);

Jm = [1 0 -sp;
      0 cr sr*cp;
      0 -sr cr*cp];

end