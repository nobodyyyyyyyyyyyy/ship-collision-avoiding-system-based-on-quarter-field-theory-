%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function outputFunction = doubleInputFunction(equation_)
    syms y_1 x_2 
    outputFunction.y_1 = solve(equation_(1), y_1, 'Real', true);
    outputFunction.y_1 = solve(equation_(2), y_1, 'Real', true);
end