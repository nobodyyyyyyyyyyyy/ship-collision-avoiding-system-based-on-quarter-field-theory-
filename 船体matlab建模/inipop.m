%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function pop=inipop(popsize,chromlength)
pop=round(rand(popsize,chromlength));        %产生一个一共popsize行，chromlength列的，随机产生0——1的数组