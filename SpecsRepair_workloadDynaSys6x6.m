%Zhi Zhang, 30th. 10. 2024, Imperial College London,
%In this code, we consider the offloading dynamical system from
%papar-Cascade Network Stability of Synchronized Traffic Load Balancing 
%with Heterogeneous Energy Efficiency Policies.
%The model has form: \dot{l}_{i}=a(1-l_{i}^2)+b\sum_{j=1,j \neq i} (l_{i}-l_{j})
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
%3. In this code we draw the figure for the 2x2 parition of the initial
%region. And the 6x6 is shown in other
%code-SpecsRepari_workloadDynaSys6x6.m


clc 
clear all
format long

%% whole continuous state space
dimension=2; %dimension of the model

l1_up=2;
l1_lo=0;
l2_up=2;
l2_lo=0;

%% initial region and its partitioning 
l1_ini_up=2;
l1_ini_lo=1.8;
l2_ini_up=2;
l2_ini_lo=1.8;
%Second initial region 
% l1_ini_up=1.25;
% l1_ini_lo=1.05;
% l2_ini_up=2;
% l2_ini_lo=1.8;


n_partition=3;%which means that the initial region is partitioned into 2x2 type

l1_ini_range=linspace(l1_ini_lo, l1_ini_up, n_partition+1); % l1-axis partitions
l2_ini_range=linspace(l2_ini_lo, l2_ini_up, n_partition+1); % l2-axis partitions

%h_l1=0.25;%These two parameters will be used to 
%h_l2=0.25;%partition the intial region of the specification

%sample the centre point from each cell of partition
%The target is to use the centre point as the initial condition and let the
%simulation run to find the closest points to the region of specification from the trajectory

l1_ini_centers = (l1_ini_range(1:end-1) + l1_ini_range(2:end)) / 2;
l2_ini_centers = (l2_ini_range(1:end-1) + l2_ini_range(2:end)) / 2;

n_l1_ini_center=length(l1_ini_centers);%this is the number of center points in l1-axis
n_l2_ini_center=length(l2_ini_centers);%this is the number of center points in l2-axis

%% store the relevant closet point to the region of specification or the deepest point
%to the region of specification

store_PoinTime=[];%zeros(dimension+1,n_partition*n_partition);%for storing the points with respect to
%the partition points which has closest distance to the region of
%specification or the deepest point to the region of specification, and
%store the relevant time moment.

store_distan=[];%zeros(1,n_partition*n_partition);

store_trajectory=[];%zeros(8,n_tim+1);

%% generate the longest distance between the center point and relevant cell of partition
%Here we only consider the square the longest distance for each is the
%same.
radius=0.5*( abs(l1_ini_range(1)-l1_ini_range(2))^2+abs(l2_ini_range(1)-l2_ini_range(2))^2 )^(0.5);

%% region of specification and time requirement
l1_desti_up=1.1;%Requirement: for the destination region, we need to ensure
l1_desti_lo=0.9;%that the region is near 1.
l2_desti_up=1.1;
l2_desti_lo=0.9;

h_tim=0.002;
n_tim=500;% 
%n_tim=200; %in example1_1.fig
tim_sta=0;
tim_end=h_tim*n_tim;

tim_vector=tim_sta:h_tim:tim_end;

%% system model of workload dynamics
a=1;
b=1;
dldt=@(l) [a*( 1-l(1)^2 )+b*( l(2)-l(1) ); a*( 1-l(2)^2 )-b*( l(2)-l(1) )];


%% trajectory discussion based on the inital points (which are the centre points)
for i_l1_ini_center=1:n_l1_ini_center
    for i_l2_ini_center=1:n_l2_ini_center
        
        l0=[l1_ini_centers(i_l1_ini_center);l2_ini_centers(i_l2_ini_center)];
        [t_vector,l]=runge_kuttabad(dldt,l0,h_tim,tim_sta,tim_end);

        %% Perform the point checking from the trajectory starting from a certain initial point
        [t_point,x,distan]=FindPoint (t_vector, l, l1_desti_up, l1_desti_lo, l2_desti_up, l2_desti_lo );
        
        midd=[x;t_point];
        
        store_PoinTime=[store_PoinTime,midd];

        store_distan=[ store_distan,distan];

        store_trajectory=[store_trajectory;l];
    end
end


%% figure illustration
%x_rectangle_ini=[1.8,2,2,1.8,1.8];
%y_rectangle_ini=[1.8,1.8,2,2,1.8];
%general form
x_rectangle_ini=[l1_ini_lo,l1_ini_up,l1_ini_up,l1_ini_lo,l1_ini_lo];
y_rectangle_ini=[l2_ini_lo,l2_ini_lo,l2_ini_up,l2_ini_up,l2_ini_lo];



x_rectangle=[0.9,1.1,1.1,0.9,0.9];
y_rectangle=[0.9,0.9,1.1,1.1,0.9];

figure(1)
hold on;
for i_plot=1:n_partition*n_partition

    plot(store_trajectory(2*(i_plot-1)+1,:),store_trajectory(2*i_plot,:),'','LineWidth', 3);

end
%hold off;
% plot(store_trajectory(1,:),store_trajectory(2,:));
% hold on 
% plot(store_trajectory(3,:),store_trajectory(4,:));
% hold on
% plot(store_trajectory(5,:),store_trajectory(6,:));
% hold on
% plot(store_trajectory(7,:),store_trajectory(8,:));
% hold on 
fill(x_rectangle, y_rectangle, 'r','FaceAlpha', 0.3);
hold on 
fill(x_rectangle_ini, y_rectangle_ini, 'b','FaceAlpha', 0.3);

%axis([0 2 0 2]);
axis([0.5 2 0.5 2]);
% Add labels and title
xlabel('l1-axis');
ylabel('l2-axis');



%% generate the region for the relevant closest/deepest point to the region of specification based LC

%we will use the LC of the estimated dynamical system (numerical
%integration), and the longest distance between the centre point and the
%point in the cell of partition. Then a circle region will be generated.

%% for each centre point we need to generate the reach region 

n_store_PoinTime=n_l1_ini_center*n_l2_ini_center; 
% the length of store_PoinTime should be the same with n_l1_ini_center*n_l2_ini_center

lc=1+b*h_tim;%which is calculated mannually 

radius_circle=radius*lc^(n_tim);%The radius of the circle at the checking points

n_circle=100;% the number of sample points of the circle 
store_circle=zeros(dimension*n_store_PoinTime,n_circle);
%This is used to store the circle trajectory of each checking points


% Create an array of angles from 0 to 2*pi
theta = linspace(0, 2*pi, n_circle);
% Parametric equations of the circle
l1_chec_circl = radius_circle * cos(theta) + store_PoinTime(1,:)';
l2_chec_circl = radius_circle * sin(theta) + store_PoinTime(2,:)';
% 
% for i_store_circle=1:n_store_PoinTime
% 
%     store_circle(2*(i_store_circle-1)+1,:)=l1_chec_circl(i_store_circle,:);
%     store_circle(2*i_store_circle,:)=l2_chec_circl(i_store_circle,:);
% 
% end

hold on 
for i_store_circle=1:n_store_PoinTime

    plot(l1_chec_circl(i_store_circle,:),l2_chec_circl(i_store_circle,:),'','LineWidth', 4);

end


set(gca, 'FontSize', 16);
xticks(0:0.5:2); % X 轴刻度每隔 2 显示一次
yticks(0:0.5:2);
box on; % 确保四周都有坐标轴
axis square; % Optional: Makes the axes square
% n_time=200
% axis([1.2 1.45 1.2 1.45]); % [xmin xmax ymin ymax]
% xticks(1.2:0.125:1.45); % X 轴刻度每隔 2 显示一次
% yticks(1.2:0.125:1.45);
%n_time=500
%axis([0.95 1.25 0.95 1.25]); % [xmin xmax ymin ymax]
%xticks(0.95:0.1:1.25); % X 轴刻度每隔 2 显示一次
%yticks(0.95:0.1:1.25);

% for the second initial region
% axis([1 1.3 1.1 1.4]); % [xmin xmax ymin ymax]
% xticks(1:0.1:1.3); % X 轴刻度每隔 2 显示一次
% yticks(1.1:0.1:1.4);
% 
%n_time=500
axis([0.9 1.2 0.9 1.2]); % [xmin xmax ymin ymax]
xticks(0.9:0.1:1.2); % X 轴刻度每隔 2 显示一次
yticks(0.9:0.1:1.2);

set(gca, 'FontSize', 26);



% 
% hold on 
% plot(store_circle(1,:), store_circle(2,:), '--g', 'LineWidth', 2);
% hold on 
% plot(store_circle(3,:), store_circle(4,:), '--y', 'LineWidth', 2);
% hold on 
% plot(store_circle(5,:), store_circle(6,:), '--c', 'LineWidth', 2);
% hold on 
% plot(store_circle(7,:), store_circle(8,:), '--m', 'LineWidth', 2);











