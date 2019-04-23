function [dt,dR,dv,dp] = relative_motion(t,omega,accel)
ba = [0;0;0];
bg = [0;0;0];
dRik = eye(3);
dvik = zeros(3,1);
dpik = zeros(3,1);
dt = diff(t);         % 1 by 20
for k = 1:length(dt)
    dRik = dRik*expm(R3_to_so3((omega(:,k)-bg)*dt(k)));
    dvik = dvik+dRik*(accel(:,k)-ba)*dt(k);
    dpik = dpik+dvik*dt(k)+1/2*dRik*(accel(:,k)-ba)*dt(k)^2;
end
dR = dRik;
dv = dvik;
dp = dpik;
end
