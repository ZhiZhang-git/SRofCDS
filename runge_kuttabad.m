%normal runge_kutta function for non-precise computation
function [t,y] = runge_kuttabad(dydt1,y0,h,t0,tn)
t=t0:h:tn;
n=length(t);
y(:,1)=y0;

for i=1:n-1
    
        k1=feval(dydt1,y(:,i));

        k2=feval(dydt1,y(:,i)+h*k1/2);

        k3=feval(dydt1,y(:,i)+h*k2/2);

        k4=feval(dydt1,y(:,i)+h*k3);

        y(:,i+1)=y(:,i)+h*(k1+2*k2+2*k3+k4)/6;
    
end
        



