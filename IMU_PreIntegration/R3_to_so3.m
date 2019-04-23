function X = R3_to_so3(theta)
% skew R3 vector to so3 lie algebra
X = [   0,         -theta(3),   theta(2);
        theta(3),   0,         -theta(1);
       -theta(2),   theta(1),   0];
end
