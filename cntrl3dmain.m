clc
hi=5;
lo=-5;
nr=100;
r=lo+(hi-lo)*rand(3,nr);
t0=0; tf=2;
x0=[];
for i=1:100
    x0=[x0;r(:,i)];
end
hold on
[t,x]=ode23(@cntrl3d,[t0,tf],x0);

% plot(t,x(:,1))
% figure
% plot(t,x(:,2))
% figure
% plot(t,x(:,3))
% figure
% plot(t,x(:,4))

% % hold on
% for i=1:length(t)
%     px=[];
%     py=[];
%     pz=[];
%     [x y z]=sphere;
%     % if i==1||i==11||i==length(t)
%     for j=1:2:300
%         px=[px x(i,j)];
%         py=[py x(i,j+1)];
%         pz=[pz x(i,j+2)];
%     end
%     % end
%     plot3(px,py,pz)
%     
%     %axis([-10 70 -10 40])
%     pause(.1)
%     
% end