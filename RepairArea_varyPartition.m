%zhi zhang, 04.12.2024, London, 
%This code is used to discuss the varying value of the partition number for
%the initial region of dynamical system

clear all
clc
format long


%% partition setting 

n_partition_low=1;
n_partition_up=5;
n_partition_vector=n_partition_low:n_partition_up;
% The value of the parition number is varying from n_partition_low to
% n_partition_up.
num_partition_vector=length(n_partition_vector);

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

%% region of specification and time requirement
l1_desti_up=1.1;%Requirement: for the destination region, we need to ensure
l1_desti_lo=0.9;%that the region is near 1.
l2_desti_up=1.1;
l2_desti_lo=0.9;

h_tim=0.002;
%n_tim=500;% 
n_tim=200; %in example1_1.fig
tim_sta=0;
tim_end=h_tim*n_tim;

tim_vector=tim_sta:h_tim:tim_end;

%% system model of workload dynamics
a=1;
b=1;
dldt=@(l) [a*( 1-l(1)^2 )+b*( l(2)-l(1) ); a*( 1-l(2)^2 )-b*( l(2)-l(1) )];


%% starting the loop for the parition number

area_vector=zeros(1,num_partition_vector);

for i_partition=1:num_partition_vector
    n_partition=n_partition_vector(i_partition);%which means that the initial region is partitioned into 
    % n_partitionxn_partition type
    
    l1_ini_range=linspace(l1_ini_lo, l1_ini_up, n_partition+1); % l1-axis partitions
    l2_ini_range=linspace(l2_ini_lo, l2_ini_up, n_partition+1); % l2-axis partitions

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
    %% setting about the circle setting
    n_store_PoinTime=n_l1_ini_center*n_l2_ini_center;
    % the length of store_PoinTime should be the same with n_l1_ini_center*n_l2_ini_center

    lc=1+b*h_tim;%which is calculated mannually and only decided by the model

    radius_circle=radius*lc^(n_tim);%The radius of the circle at the checking points

    n_circle=100;% the number of sample points of the circle
    store_circle=zeros(dimension*n_store_PoinTime,n_circle);
    %This is used to store the circle trajectory of each checking points

    % Create an array of angles from 0 to 2*pi
    theta = linspace(0, 2*pi, n_circle);
    % Parametric equations of the circle
    l1_chec_circl = radius_circle * cos(theta) + store_PoinTime(1,:)';
    l2_chec_circl = radius_circle * sin(theta) + store_PoinTime(2,:)';
    
    polygons=[];
    
    for i_poly=1:n_store_PoinTime

        polygons=[polygons, polyshape(l1_chec_circl(i_poly,:), l2_chec_circl(i_poly,:))];

    end

    combinedPolygon = polygons(1);

    for i_comb = 2:n_store_PoinTime
        
        combinedPolygon = union(combinedPolygon, polygons(i_comb));

    end

    % Calculate the area of the combined region
    areaUnion = area(combinedPolygon);
    
    area_vector(i_partition)=areaUnion;

end 


% Plot the Gantt chart
figure(1)
bar( n_partition_vector,area_vector,0.5, 'FaceColor', 'blue');
%hold on;
set(gca, 'FontSize', 16);
yticks(0:0.03:0.15);
%yticks(0:0.1:0.5);


xlim([0 5]);          % Set x-axis limits
ylim([0 0.15]);  
%ylim([0 0.5]);  

% Add labels and title
xlabel('Partition');
ylabel('Area');

xticklabels({'1\times 1', '2\times 2', '3\times 3','4\times 4', '5\times 5'});

axis square;