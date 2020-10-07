clc
hi=5;
lo=-5;
nr=100;
r=lo+(hi-lo)*rand(2,nr);
t0=0; tf=100;
x0=[];
for i=1:100
    x0=[x0;r(:,i)];
end

[t,x]=ode23(@cntrl2d2,[t0,tf],x0);

% hold on
for i=1:length(t)
    px=[];
    py=[];
    % if i==1||i==11||i==length(t)
    for j=1:2:200
        px=[px x(i,j)];
        py=[py x(i,j+1)];
    end
    % end
    plot(px,py,'o')
    
    axis([-10 70 -10 40])
    pause(.1)
    
end