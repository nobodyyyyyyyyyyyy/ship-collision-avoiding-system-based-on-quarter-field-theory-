%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
example = 2;
switch example 
    case 1
    n = 100;
    x = linspace(-1,1,n)';
    y = 1./(1+25*x.^2) + 1e-1*randn(n,1);
    % fit a polynomail of order:
    k = 6;
    case 2 
    x = (0:24)'; n=numel(x);
    y =[110.49,  73.72,  23.39,  17.11,  20.31,  29.37,  74.74, ...
        117.02, 298.04, 348.13, 294.75, 253.78, 250.48, 239.48, ...
        236.52, 245.04, 286.74, 304.78, 288.76, 247.11, 216.73, ...
        185.78, 171.19, 171.73, 164.05]';
    % fit a polynomail of order:
    k = 8;
end 
% Visualize data points
plot(x,y,'ok')
 
%% Fit a polynomail of Kth-order to noisy data:

% Build a discrete grid for visualizing the numerical solutions
x_poly = linspace(min(x),max(x),2*n);
hold on;

% 1. Least Square Fitting using Matlab's polyfit:
    tic
    P1 = polyfit(x,y,k);
    toc
    h(1)=plot(x_poly,polyval(P1,x_poly),'-','linewidth',2);

% 2. Using Linear algebraic operations as indicated in Ref.[1]:
    tic
    % Build Vandermonde matrix
    X = Vandermonde(x,k);
    % Produce least square fitting 
    P2 = (X'*X)\(X'*y); % Eq.(16)
	toc
    h(2)=plot(x_poly,polyval(flipud(P2),x_poly),'-','linewidth',2);

% 3. Using Linear algebraic operations [1] + Matlab's LU functions [2]
    tic
    % Build Vandermonde matrix
    X = Vandermonde(x,k);
    % Produce least square fitting 
    A = (X'*X); b = (X'*y);
    [L,U] = lu(A); 
      Y   = L\b;
      P3  = U\Y;
    toc
    h(3)=plot(x_poly,polyval(flipud(P3),x_poly),'-','linewidth',2);

% 4. Using Linear algebraic operations [1] + Costum LU-decomposition :
    tic
    % Build Vandermonde matrix
    X = Vandermonde(x,k);
    % Produce least square fitting 
    A = (X'*X); b = (X'*y);
    [L,U] = LU.decomp(A); 
      Y   = LU.forward(L,b); 
      P4  = LU.backward(U,Y);
    toc
    h(4)=plot(x_poly,polyval(flipud(P4),x_poly),'--','linewidth',2);

% Finish visualization
hold off
xlabel('$x$','interpreter','latex','fontsize',20);
ylabel('$y$','interpreter','latex','fontsize',20);
title('Least squares -polynomial- fitting','interpreter','latex','fontsize',20);
legend(h,{"Matlab's polyfit",'fitting method 1','fitting method 2','fitting method 3'});
legend(h,'location','best','interpreter','latex','fontsize',12)
legend boxoff;
%% Conclusion:
%   Using Matlab's LU and backlash, to produce a least squares polynomial
%   fitting, is the fastes approach. However, our costum LU decomposition,
%   along with its Forward and Backwards methods, is faster than Matlab's
%   polyfit and is also easy to export in C/C++/Fortran. ;D
