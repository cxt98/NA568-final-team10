function x = so3_to_R3(X)
% unskew so3 lie algebra to R3 vector
x = [X(3,2); X(1,3); X(2,1)];
end
