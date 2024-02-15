classdef LU
    methods (Static)

    function [L,U] = decomp(A)
    % Computes the LU decomposition of square matrix A.
    %
    % Main Parameter
    n = size(A,1);
    % Work Matrices
    mat1 = A;
    mat2 = eye(n);
    % Produce LU factorization
    for pivot_index=1:n-1
       for row=pivot_index+1:n
           row_pivot=mat1(row,pivot_index)/mat1(pivot_index,pivot_index);
           mat2(row, pivot_index)=row_pivot;
           for col=pivot_index:n
              mat1(row,col)=mat1(row,col)-row_pivot*mat1(pivot_index,col);
           end
       end
    end
    % Report resutls
    U=mat1; L=mat2;
    end % LU

    function y = forward(L,b)
    % forward substitution for a lower-triangular matrix : [L]*{y} = {b}
    N = size(L,1);
    y(1,:) = b(1,:)/L(1,1);
    for m = 2:N
        y(m,:) = (b(m,:)-L(m,1:m - 1)*y(1:m-1,:))/L(m,m);
    end
    end % forwardSubs

    function x = backward(U,y)
    % backward substitution for a upper-triangular matrix : [U]*{x} = {y}
    N = size(U,2);
    x(N,:) = y(N,:)/U(N,N);
    for m = N-1:-1:1
        x(m,:) = (y(m,:) - U(m,m + 1:N)*x(m + 1:N,:))/U(m,m);
    end
    end % backwardSubs

    end
end