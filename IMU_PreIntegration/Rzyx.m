function R = Rzyx(yaw,pitch,roll)
% Rotation matrix for yaw pitch roll angles
% Order of rotation R = Rz * Ry * Rx
% Extrinsic rotations; The XYZ system rotates, while xyz is fixed.
% Verified using MATLAB rotm = eul2rotm([yaw,pitch,roll], 'ZYX')

%{  
    Copyright (C) 2017  Maani Ghaffari Jadidi
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details. 
%}

R = Rz(yaw) * Ry(pitch) * Rx(roll);
end
function R = Rz(alpha)
R = [cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0;0 0 1];
end
function R = Ry(beta)
R = [cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)];
end
function R = Rx(gamma)
R = [1 0 0;0 cos(gamma) -sin(gamma);0 sin(gamma) cos(gamma)];
end
