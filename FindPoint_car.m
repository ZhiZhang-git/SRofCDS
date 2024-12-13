%Zhi Zhang,24.10.2024, ondon,
%This code is used to help the code-SpecsRepair_workloadDynaSys_car.m, and 
%The aim of this code is to find/select the points which has the following
%properties: 1. if the trajectory does not reach the region of
%specification, we need to find the closet point to the region of the
%specification and the relevant time. 
%2. if the trajectory has some points in the region of the specification,
%we need to find the point where is the farest away to the boundary of the
%region of the specification. 
%Remark: The outputs are the point satisfying one of the above properties,
%and the relevant time/moment.
%The inputs are t_vector (1xn) (which is the time vector of the trajectory starting
%from the initial center point), trajectory l (2xn) (which is the solution of the
%model of workload dynamics of Base Station), and the boundary value of the
%region of specification 
%
%Important: 1. for car model, l is the trajectory vector of the non-linear
%car model, which should be 5 dimensions. l=vari_ini=[x;y;theta;v;ka];
%2. However, the region this example consider is only related to the first
%two position variables.

function [t_point,x,distan]=FindPoint_car (t_vector, l, l1_desti_up, l1_desti_lo, l2_desti_up, l2_desti_lo )

n_length=length(t_vector);%1xn 

[row,colu]=size(l);%To obtain the row of vector l. for the car model it should be 5 dimensions

%% Extract region boundaries
l1_min=l1_desti_lo;
l1_max=l1_desti_up;
l2_min=l2_desti_lo;
l2_max=l2_desti_up;

%% judgement (is used to look for the distance between the boundary of the region of the specification)
diff_l1_min=l1_min-l(1,:);
diff_l1_max=l1_max-l(1,:);
diff_l2_min=l2_min-l(2,:);
diff_l2_max=l2_max-l(2,:);


buff_insider=[];

parfor i=1:n_length

    % determine the negative or positive of the difference between the closest
    % point and the relevant boundaries of the region of the specification
    %For the l1_min;
    if diff_l1_min(i)<=0 &&  diff_l1_max(i)>=0 &&...
            diff_l2_min(i)<=0 && diff_l2_max(i)>=0

        buff_insider=[buff_insider,i];

    end
end

n_buff_insider=length(buff_insider);


%% if the buff_insider is non-empty, we need to find the point with fartest distant
%to the boudary of the region
if isempty(buff_insider)~=1
    vector_insider=zeros(row+1,n_buff_insider);
    parfor i1=1:n_buff_insider
        
        vector_insider(:,i1)=[l(:,buff_insider(i1));t_vector(buff_insider(i1))];

    end
    % the part for find the fartest distance to the boundary of the
    % specification region

    far_l1_min=l1_min-vector_insider(1,:);
    far_l1_max=l1_max-vector_insider(1,:);
    far_l2_min=l2_min-vector_insider(2,:);
    far_l2_max=l2_max-vector_insider(2,:);

    
    midd=far_l1_min.^2+far_l1_max.^2+far_l2_min.^2+far_l2_max.^2;
    [distan,index_deep]=max(midd);

    % midd=max([abs(far_l1_min);abs(far_l1_max);abs(far_l2_min);abs(far_l2_max)]);
    % [distan,index_deep]=min(midd);

    point_deepest=vector_insider(:,index_deep); %the deepest point is found and its relevant time moment

    x=point_deepest(1:row);

    t_point=point_deepest(end);

else %isempty(buff_insider) is empty logically
    %% if the buff_insder is empty, it means that the trajectory from this
    % intial point does not reach the region of the specification
    
    %Target is to find the closest point to the region of the specification

    %% First of all, we need to check whether this trajectory has reached the region of the specification
    %midd=min([abs(diff_l1_min); abs(diff_l1_max) ; abs(diff_l2_min) ; abs(diff_l2_max)]);
    midd=diff_l1_min.^2+diff_l1_max.^2+diff_l2_min.^2+diff_l2_max.^2;

    [distan,index_deep]=min(midd);

    point_closest=[l(:,index_deep);t_vector(index_deep)];

    x=point_closest(1:row);

    t_point=point_closest(end);

end




