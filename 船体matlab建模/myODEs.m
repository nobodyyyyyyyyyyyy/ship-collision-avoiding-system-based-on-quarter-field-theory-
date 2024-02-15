%Author: Zhang Ziqing
%University: Wuhan University of Technology
%Contact Email: 322980@whut.edu.cn
%Feedback and Support
%If you have any questions, suggestions, or would like to contribute, feel free to contact me 
% or raise an issue on the GitHub Issues page.
function dydt = myODEs(t, y, sigma_previous, sigma_updated, np, T_max)
    % Extract the variables from the input vector y
    u = y(1);
    v = y(2);
    r = y(3);
    t_ = t;

    L = 7;
    angular_speed = 3000;
    % load motion_equation
    if t_<=abs(T_max)
        sigma = sigma_previous + sign(T_max)*angular_speed*t_;
    else 
        sigma = sigma_updated;
    end
    V = sqrt(v^2 + u^2);
%     disp([v, u])
    beta = atan(-v/u);
    beta_p = beta - (-0.5 * r * L / V);
    beta_r = beta + 0.71 * r * L / V;
    if beta_p >= 0&&beta_r >= 0
        Flag = 1;
    elseif beta_p < 0&&beta_r >= 0
        Flag = 2;
    elseif beta_p < 0&&beta_r < 0
        Flag = 3;
    else
        Flag = 4;
    end
    if Flag == 1
    % Define your system of differential equations
    du_dt = (1099511627776*(1610*u^2 + 1610*v^2)*((539*r^2)/(1000*(u^2 + v^2)) - v^2/(25*(u^2 + v^2)) + (89*v^4)/(125*(u^2 + v^2)^2) + (7*r*v)/(500*(u^2 + v^2)) - 41/2000))/3595427212083331 + (3595648213920514*r*v)/3595427212083331 - (489383537374431765*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(2160*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(11664*np^2) - 2931/10000))/942519671084372721664 + (898850755706880*r^2)/3595427212083331 - (84250078478336*sin((pi*(sigma - (64*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(109*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*sin((pi*sigma)/180)*((5209514239900359*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/171798691840000 + (61894238684256165279*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/449428401510416375;
    dv_dt = (3024811283085532177211850752*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (19378432542558421171785885212899*r*u)/19379648516694221687771516496706 + (247074214383739837580574720*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (635461404103330550393841123328*sin((pi*(sigma - (64*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(109*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((5209514239900359*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/171798691840000 + (61894238684256165279*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/1211228032293388855485719781044125;
    dr_dt = (244993068939297688090263249818543*sin((pi*(sigma - (64*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(109*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((5209514239900359*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/171798691840000 + (61894238684256165279*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/496119002027372075206950822315673600 - (247074214383739837580574720*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (988364255149402852704649216*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (49661917091137100458229760*r*u)/9689824258347110843885758248353;
    elseif Flag == 2
        du_dt = (1099511627776*(1610*u^2 + 1610*v^2)*((539*r^2)/(1000*(u^2 + v^2)) - v^2/(25*(u^2 + v^2)) + (89*v^4)/(125*(u^2 + v^2)^2) + (7*r*v)/(500*(u^2 + v^2)) - 41/2000))/3595427212083331 + (3595648213920514*r*v)/3595427212083331 - (489383537374431765*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(2160*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(11664*np^2) - 2931/10000))/942519671084372721664 + (898850755706880*r^2)/3595427212083331 - (84250078478336*sin((pi*(sigma - (64*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(109*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*sin((pi*sigma)/180)*((5209514239900359*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/171798691840000 + (61894238684256165279*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/449428401510416375;
        dv_dt = (3024811283085532177211850752*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (19378432542558421171785885212899*r*u)/19379648516694221687771516496706 + (247074214383739837580574720*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (635461404103330550393841123328*sin((pi*(sigma - (64*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(109*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((5209514239900359*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/171798691840000 + (61894238684256165279*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/1211228032293388855485719781044125;
        dr_dt = (244993068939297688090263249818543*sin((pi*(sigma - (64*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(109*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((5209514239900359*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/171798691840000 + (61894238684256165279*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/496119002027372075206950822315673600 - (247074214383739837580574720*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (988364255149402852704649216*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (49661917091137100458229760*r*u)/9689824258347110843885758248353;
    elseif Flag == 3
        du_dt = (1099511627776*(1610*u^2 + 1610*v^2)*((539*r^2)/(1000*(u^2 + v^2)) - v^2/(25*(u^2 + v^2)) + (89*v^4)/(125*(u^2 + v^2)^2) + (7*r*v)/(500*(u^2 + v^2)) - 41/2000))/3595427212083331 + (3595648213920514*r*v)/3595427212083331 - (489383537374431765*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(2160*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(11664*np^2) - 2931/10000))/942519671084372721664 + (898850755706880*r^2)/3595427212083331 - (84250078478336*sin((pi*(sigma - (79*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(218*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*sin((pi*sigma)/180)*((32512578371218140519*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/2814749767106560000 + (61894238684256165279*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/449428401510416375;
        dv_dt = (3024811283085532177211850752*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (19378432542558421171785885212899*r*u)/19379648516694221687771516496706 + (247074214383739837580574720*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (635461404103330550393841123328*sin((pi*(sigma - (79*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(218*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((32512578371218140519*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/2814749767106560000 + (61894238684256165279*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/1211228032293388855485719781044125;
        dr_dt = (244993068939297688090263249818543*sin((pi*(sigma - (79*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(218*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((32512578371218140519*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/2814749767106560000 + (61894238684256165279*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2*((72*((1 - (729*np^2*((2753*u*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70))/(270*np) + (34625*u^2*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*(exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2))))/70 - 11/70)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/496119002027372075206950822315673600 - (247074214383739837580574720*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (988364255149402852704649216*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (49661917091137100458229760*r*u)/9689824258347110843885758248353;
    else
        du_dt = (1099511627776*(1610*u^2 + 1610*v^2)*((539*r^2)/(1000*(u^2 + v^2)) - v^2/(25*(u^2 + v^2)) + (89*v^4)/(125*(u^2 + v^2)^2) + (7*r*v)/(500*(u^2 + v^2)) - 41/2000))/3595427212083331 + (3595648213920514*r*v)/3595427212083331 - (489383537374431765*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(2160*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(11664*np^2) - 2931/10000))/942519671084372721664 + (898850755706880*r^2)/3595427212083331 - (84250078478336*sin((pi*(sigma - (79*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(218*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*sin((pi*sigma)/180)*((32512578371218140519*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/2814749767106560000 + (61894238684256165279*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/449428401510416375;
        dv_dt = (3024811283085532177211850752*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (19378432542558421171785885212899*r*u)/19379648516694221687771516496706 + (247074214383739837580574720*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (635461404103330550393841123328*sin((pi*(sigma - (79*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(218*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((32512578371218140519*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/2814749767106560000 + (61894238684256165279*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/1211228032293388855485719781044125;
        dr_dt = (244993068939297688090263249818543*sin((pi*(sigma - (79*(u^2 + v^2)^(1/2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2))))/(218*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115)^(1/2))))/180)*cos((pi*sigma)/180)*((32512578371218140519*(u^2 + v^2)*(atan(v/u) + (497*r)/(100*(u^2 + v^2)^(1/2)))^2)/2814749767106560000 + (61894238684256165279*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2*((72*((1 - (729*np^2*((2753*u*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35))/(270*np) + (34625*u^2*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2)/(1458*np^2) - 2931/1250))/(15625*u^2*pi*((3*exp(-2*abs(atan(v/u) + (7*r)/(2*(u^2 + v^2)^(1/2)))))/35 - 8/35)^2))^(1/2)/2 + 1/2)^2)/115 + 43/115))/703687441776640000))/496119002027372075206950822315673600 - (247074214383739837580574720*(1610*u^2 + 1610*v^2)*((581*r)/(1000*(u^2 + v^2)^(1/2)) - (59*v)/(200*(u^2 + v^2)^(1/2)) + (343*r^3)/(125*(u^2 + v^2)^(3/2)) - (841*v^3)/(500*(u^2 + v^2)^(3/2)) + (2653*r*v^2)/(1000*(u^2 + v^2)^(3/2)) - (18767*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (988364255149402852704649216*(11270*u^2 + 11270*v^2)*((343*r)/(1000*(u^2 + v^2)^(1/2)) + (127*v)/(1000*(u^2 + v^2)^(1/2)) + (1029*r^3)/(250*(u^2 + v^2)^(3/2)) + (3*v^3)/(100*(u^2 + v^2)^(3/2)) + (1029*r*v^2)/(500*(u^2 + v^2)^(3/2)) - (2401*r^2*v)/(1000*(u^2 + v^2)^(3/2))))/9689824258347110843885758248353 - (49661917091137100458229760*r*u)/9689824258347110843885758248353;
    end
    
    % Pack the derivatives into the output vector dydt
    dydt = [du_dt; 
            dv_dt; 
            dr_dt];
end
