%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function [X_i, Y_i, N_i] = inertia(u,v,r,u_,v_,r_)

%r 船的角速度
%u y方向的速度
%v x方向的速度
%r_ 船的加角速度
%u_ y方向的加速度
%v_ x方向的加速度
%x 动系G xyz相对于原点坐标系的位置
%x_g 重心纵向位置
%m 船体质量大小
%
%I_z 船舶对ox轴的转动惯量
%mx 船在x方向上所增加的液体质量大小
%my 船在y方向上所增加的液体质量大小
%J_zz 船在转动方向上所增加的液体质量大小


L=7;
x_g = 0.25;
A_r=0.0539;
den=1000;
Dp=0.216;
n=0.216/0.345; 
f_a=2.747; 
wp0=0.40/0.35; 
kt0=0.2931; kt1=-0.2753; kt2=-0.1385; 
m = 3270; 
I_z=10008.25;
m_x=0.022;
m_y=0.223;
J_zz=0.011;
r0=0.01546;
Xvv=-0.04;Xvr=0.002;Xrr=0.011;Xvvvv=0.712;
Nv=-0.127;Nr=-0.049;Nvvv=-0.03;Nvvr=-0.294;Nvrr=0.049;Nrrr=-0.012;
Yv=-0.295;Yr=0.083;Yvvv=-1.682;Yvvr=0.379;Yvrr=-0.383;Yrrr=0.008;


X_i = (m+m_x)*u_-(m+m_y)*v*r-m*x_g*r^2;
Y_i = (m+m_y)*v_+(m+m_x)*u*r+m*x_g*r_;
N_i=(I_z+J_zz)*r_+m*x_g*(v_+r*u);
end


