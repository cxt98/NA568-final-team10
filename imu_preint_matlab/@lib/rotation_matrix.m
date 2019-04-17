function R = rotation_matrix(omega,dt)
e1 = [1;0;0];
e2 = [0;1;0];
e3 = [0;0;1];
Gx = skew(e1);
Gy = skew(e2);
Gz = skew(e3);
G = omega(1)*dt*Gx+omega(2)*dt*Gy+omega(3)*dt*Gz;
R = expm(G);
end

function X = skew(x)
X = [0,    - x(3),   x(2);   
     x(3),   0,     -x(1);
     x(2),   x(1),   0];
end