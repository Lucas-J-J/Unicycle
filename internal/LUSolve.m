function [X] = LUSolve(A,b)
%LUSolve solves system of equations A X = b using LU factorization
%   A is coefficient matrix
%   b is RHS
%   function has been checked against linsolve()

[L,U,P] = lu(A);
y = L\(P*b);
X = U\y;

end

