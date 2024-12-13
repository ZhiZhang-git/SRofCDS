%test code for FindPoint_car.m



clear all
clc


x_ini=1.00833333333333;
y_ini=1.09166666666667;

h_tim=0.002;
n_tim=500;% n_tim=200 in example1_1.fig
tim_sta=0;
tim_end=h_tim*n_tim;

tim_vector=tim_sta:h_tim:tim_end;

%% model and rest variable of initial points
%vari(1)=x; vari(2)=y; vari(3)=theta; vari(4)=v; vari(5)=ka

df_car=@(vari) [ vari(4)*cos(vari(3));vari(4)*sin(vari(3)); vari(4)*vari(5);...
    0;0];

%The following is the information for the rest varibale of initial
%condition
theta=0;%\theta\in [-pi,pi]
v=1; %v\in (0,10]
ka=0.12;%ka\in [-0.25,0.25]


x_desti_up=1.4;%Requirement: for the destination region, we need to ensure
x_desti_lo=1.3;%that the region is near 1.
y_desti_up=1.4;
y_desti_lo=1.3;



l0=[x_ini;y_ini;theta;v;ka];

[t_vector,l]=runge_kuttabad(df_car,l0,h_tim,tim_sta,tim_end);

[t_point,x,distan]=FindPoint_car (t_vector, l, x_desti_up, x_desti_lo, y_desti_up, y_desti_lo );
       
        
figure(2)
