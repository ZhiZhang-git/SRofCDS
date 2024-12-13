%Zhi Zhang, 05. 12. 2024, London,
%In this code, we consider the offloading dynamical system from
%papar-Cascade Network Stability of Synchronized Traffic Load Balancing 
%with Heterogeneous Energy Efficiency Policies.
%The model has form: df_car=@(vari) [ vari(4)*cos(vari(3));
%vari(4)*sin(vari(3)); vari(4)*vari(5);0;0];
%
%The detailed princple of this code is as follow:
%1. starting from one inital point (when this point cannot reach the region of the speicfiction),
%let the simulation running, from this trajectory to find the closest point 
%to the region of STL specification. Then, based on the distance to the
%initial point, we can use the LC to quantify the reach region based on the
%distance to the initial point. 
%2. starting from one inital point (when this point can have some points reach
% the region of specification), let the simulation running, from this
% trajectory to find the closest point to the region of STL specification,
% and it must have many points within the inside of the region of the
% specificaiton. Then we need to sample the point where has the longest
% distance to the boundary of the region of the specification.

%Important: 1. The non-linear car model is five dimensions.
%2. However, here, we consider the region only with two dimensions
%(x,y). Thus, the boudary will be set only for the 
%3. This code can run the simulation for parition with much smaller cell


clc 
clear all
format long

%% whole continuous state space
dimension=5; %dimension of the model

%% initial region and its partitioning 

x_ini_up=1.1;% initial region's upper bound at x
x_ini_lo=1;% initial region's lower bound at x
y_ini_up=1.1;% initial region's upper bound at y
y_ini_lo=1;% initial region's lower bound at y

n_partition=3;%which means that the initial region is partitioned into 2x2 type


x_ini_range=linspace(x_ini_lo, x_ini_up, n_partition+1); % l1-axis partitions
y_ini_range=linspace(y_ini_lo, y_ini_up, n_partition+1); % l2-axis partitions

%sample the centre point from each cell of partition
%The target is to use the centre point as the initial condition and let the
%simulation run to find the closest points to the region of specification from the trajectory

x_ini_centers = (x_ini_range(1:end-1) + x_ini_range(2:end)) / 2;
y_ini_centers = (y_ini_range(1:end-1) + y_ini_range(2:end)) / 2;

n_x_ini_center=length(x_ini_centers);%this is the number of center points in x-axis
n_y_ini_center=length(y_ini_centers);%this is the number of center points in y-axis

%% store the relevant closet point to the region of specification or the deepest point
%to the region of specification

store_PoinTime=[];%zeros(dimension+1,n_partition*n_partition);%for storing the points with respect to
%the partition points which has closest distance to the region of
%specification or the deepest point to the region of specification, and
%store the relevant time moment.

store_distan=[];%zeros(1,n_partition*n_partition);

store_traj_positi=[];%zeros(8,n_tim+1); storing position of trajectory

%% generate the longest distance between the center point and relevant cell of partition
%Here we only consider the square the longest distance for each is the
%same.
radius=0.5*( abs(x_ini_range(1)-x_ini_range(2))^2+abs(y_ini_range(1)-y_ini_range(2))^2 )^(0.5);

%% region of specification and time requirement
%x_desti_up=1.4;%Requirement: for the destination region, we need to ensure
%x_desti_lo=1.3;%that the region is near 1.
%y_desti_up=1.4;
%y_desti_lo=1.3;

%second reaching region 
x_desti_up=1.4;%Requirement: for the destination region, we need to ensure
x_desti_lo=1.3;%that the region is near 1.
y_desti_up=1.2;
y_desti_lo=1.1;



h_tim=0.002;
n_tim=200;% n_tim=200 in example1_1.fig
tim_sta=0;
tim_end=h_tim*n_tim;

tim_vector=tim_sta:h_tim:tim_end;

%% model and rest variable of initial points
%environment (wind conditions)
w_x=0.3;

w_y=0.1;

w_the=0.1;


%vari(1)=x; vari(2)=y; vari(3)=theta; vari(4)=v; vari(5)=ka

df_car=@(vari) [ vari(4)*cos(vari(3))+w_x;vari(4)*sin(vari(3))+w_y; vari(4)*vari(5)+w_the;...
    0;0];

%The following is the information for the rest varibale of initial
%condition
theta=0;%\theta\in [-pi,pi]
v=1; %v\in (0,10]
ka=0.12;%ka\in [-0.25,0.25]

%% trajectory discussion based on the inital points (which are the centre points)
%Important illustration: since our system is 5 dimension, and the initial
%region is 2 dimension, the setting for the initial region and the rest of
%3 varibales are should be fixed. Through this way, we could avoid the
%complexity brought from the large value region of the rest variable
for i_l1_ini_center=1:n_x_ini_center
    for i_l2_ini_center=1:n_y_ini_center

        l0=[x_ini_centers(i_l1_ini_center);y_ini_centers(i_l2_ini_center);theta;v;ka];
        [t_vector,l]=runge_kuttabad(df_car,l0,h_tim,tim_sta,tim_end);
        %% Perform the point checking from the trajectory starting from a certain initial point
        [t_point,x,distan]=FindPoint_car (t_vector, l, x_desti_up, x_desti_lo, y_desti_up, y_desti_lo );
       
        midd=[x;t_point];

        store_PoinTime=[store_PoinTime,midd];

        store_distan=[ store_distan,distan];

        store_traj_positi=[store_traj_positi;l(1:2,:)];%

    end
end

%% figure illustration

x_rectangle_ini=[1,1.1,1.1,1,1]; %initial region
y_rectangle_ini=[1,1,1.1,1.1,1];


x_rectangle=[1.3,1.4,1.4,1.3,1.3]; %reaching area
y_rectangle=[1.1,1.1,1.2,1.2,1.1];


figure(1)
hold on 
for i_plot=1:n_partition*n_partition

     plot(store_traj_positi(2*(i_plot-1)+1,:),store_traj_positi(2*i_plot,:),' ','LineWidth', 3);

end
% 
% figure(1)
% plot(store_traj_positi(1,:),store_traj_positi(2,:));
% hold on 
% plot(store_traj_positi(3,:),store_traj_positi(4,:));
% hold on
% plot(store_traj_positi(5,:),store_traj_positi(6,:));
% hold on
% plot(store_traj_positi(7,:),store_traj_positi(8,:));
hold on 
fill(x_rectangle, y_rectangle, 'r','FaceAlpha', 0.3);
hold on 
fill(x_rectangle_ini, y_rectangle_ini, 'b','FaceAlpha', 0.3);

%axis([0 2 0 2]);
% Add labels and title
%xlabel('l1-axis');
%ylabel('l2-axis');

axis([0.9 1.5 0.9 1.25]);
%xticks(0.9:0.15:1.5); % X 轴刻度每隔 2 显示一次
%yticks(0.9:0.07:1.25);
% Add labels and title
xlabel('l_1-axis');
ylabel('l_2-axis');
box on

%% for each centre point we need to generate the reach region 

n_store_PoinTime=n_x_ini_center*n_y_ini_center; 
% the length of store_PoinTime should be the same with n_l1_ini_center*n_l2_ini_center

lc=1+h_tim;%which is calculated mannually 

radius_circle=radius*lc^(n_tim);%The radius of the circle at the checking points

n_circle=100;% the number of sample points of the circle 
store_circle=zeros(dimension*n_store_PoinTime,n_circle);
%This is used to store the circle trajectory of each checking points

% Create an array of angles from 0 to 2*pi
theta = linspace(0, 2*pi, n_circle);
% Parametric equations of the circle
l1_chec_circl = radius_circle * cos(theta) + store_PoinTime(1,:)';
l2_chec_circl = radius_circle * sin(theta) + store_PoinTime(2,:)';


hold on 
for i_store_circle=1:n_store_PoinTime

    plot(l1_chec_circl(i_store_circle,:),l2_chec_circl(i_store_circle,:),'','LineWidth', 3);

end



set(gca, 'FontSize', 16); 
axis square;


%% for inner
%axis([1.23 1.43 0.975 1.18]); % [xmin xmax ymin ymax]
%xticks(1.23:0.05:1.43); % X 轴刻度每隔 2 显示一次
%yticks(0.975:0.05:1.18);

%set(gca, 'FontSize', 26);




% for i_store_circle=1:n_store_PoinTime
% 
%     store_circle(2*(i_store_circle-1)+1,:)=l1_chec_circl(i_store_circle,:);
%     store_circle(2*i_store_circle,:)=l2_chec_circl(i_store_circle,:);
% 
% end
% 
% hold on 
% plot(store_circle(1,:), store_circle(2,:), '--g', 'LineWidth', 2);
% hold on 
% plot(store_circle(3,:), store_circle(4,:), '--y', 'LineWidth', 2);
% hold on 
% plot(store_circle(5,:), store_circle(6,:), '--c', 'LineWidth', 2);
% hold on 
% plot(store_circle(7,:), store_circle(8,:), '--m', 'LineWidth', 2);
% 












