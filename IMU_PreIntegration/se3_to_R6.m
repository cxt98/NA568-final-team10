function x = se3_to_R6(X)
% wedge: se(3) -> R^6
x = [so3_to_R3(X(1:3,1:3)); X(1:3,4)];
end

