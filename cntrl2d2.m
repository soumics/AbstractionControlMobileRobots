function qdot=cntrl2d2(t,x)
q=[];
for i=1:2:length(x)
    q=[q [x(i);x(i+1)]];
end
mu1=0;
for i=1:100
    mu1=mu1+q(:,i);
end
mu=(1/100)*mu1;

E1=[0 1;1 0];
E2=[1 0;0 -1];
% E3=[0 -1;1 0];
yy=0; xx=0;
for i=1:100
    yy=yy+((q(:,i)-mu)'*E1*(q(:,i)-mu));
    xx=xx+((q(:,i)-mu)'*E2*(q(:,i)-mu));
end
theta=(1/2)*atan2(yy,xx);

R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
H1=eye(2)+R^2*E2;
H2=eye(2)-R^2*E2;
H3=R^2*E1;

st1=0; st2=0;
for i=1:100
    st1=st1+((q(:,i)-mu)'*H1*(q(:,i)-mu));
    st2=st2+((q(:,i)-mu)'*H2*(q(:,i)-mu));
end
s1=(1/(2*99))*st1;
s2=(1/(2*99))*st2;

K_mu=2*eye(2);
k_theta=2;
k_s1=2; k_s2=2;

mu_d=[3;23];
theta_d=0;
s1_d=10.8574;
s2_d=0.3518;

mudot=K_mu*(mu_d-mu);
thetadot=k_theta*(theta_d-theta);
s1dot=k_s1*(s1_d-s1);
s2dot=k_s2*(s2_d-s2);

adot=[mudot;thetadot;s1dot;s2dot];

mumat=[];
thetamat=[];
s1mat=[];
s2mat=[];
for i=1:100
    mumat=[mumat (1/100)*eye(2)];
    thetamat=[thetamat (1/(99*(s1-s2)))*((q(:,i)-mu)'*H3)];
    s1mat=[s1mat (1/99)*(q(:,i)-mu)'*H1];
    s2mat=[s2mat (1/99)*(q(:,i)-mu)'*H2];
end
phi=[mumat;thetamat;s1mat;s2mat];

qdot=phi'*inv(phi*phi')*adot;
% size(qdot)
% qw