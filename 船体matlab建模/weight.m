%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
criteria = rand(1,20);
k = 5;
sigmoid =@(x) 1/(1+exp(-k*(x-0.52)));
result = arrayfun(sigmoid, criteria);
fplot(sigmoid)