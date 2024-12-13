%Zhi Zhang, 24.10.2024, London
%This code is used to run the simulation of non-linear car-like model
%The details of this model: 
% \dot{x}=v \cos {\theta}, ~ \dot{y}=v\sin{\theta},~\dot{\theta}=v\kappa,
% ~\dot{v}=u_{v},~\dot{\kappa}=u_{\kappa}.

clc 
clear all

%% initial condition
x=1;%x\in [0,200]
y=1;%y\in [0,100]
theta=0;%\theta\in [-pi,pi]
v=1; %v\in (0,10]
ka=0.12;%ka\in [-0.25,0.25]

vari_ini=[x;y;theta;v;ka];%initial condition

%u_v=0;
%u_k=0;

%% parameter setting 

tim_step=0.002;
n_step=200;
tim_star=0;
tim_end=n_step*tim_step;

store_trajectory=zeros(5,n_step);%store the trajectory

% %% initial region %here, we just only consider the region for x and y, 
% %the rest of variables are the same 
% x_lo=0;
% x_up=1;
% y_lo=0;
% y_up=1;

%% model
%vari(1)=x; vari(2)=y; vari(3)=theta; vari(4)=v; vari(5)=ka

df_car=@(vari) [ vari(4)*cos(vari(3));vari(4)*sin(vari(3)); vari(4)*vari(5);...
    0;0];


[t_vec,vari_vec]=runge_kuttabad(df_car,vari_ini,tim_step,tim_star,tim_end);


figure(1)
plot(vari_vec(1,:),vari_vec(2,:));





