function X = R6_to_se3(zeta)
% hat operation
X = [R3_to_so3(zeta(1:3)),zeta(4:6);zeros(1,4)];
end

