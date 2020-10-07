function qdot=cntrl3d(t,x)
qx=[];
qy=[];
qr=[];
qz=[];
for i=1:3:length(x)
    qx=[qx x(i)];
    qy=[qy x(i+1)];
    qr=[qr sqrt(x(i)^2+x(i+1)^2)];
    qz=[qz x(i+2)];
end
qxy=[qx;qy];
qrz=[qr;qz];

muxy1=0;
murz1=0;
for i=1:100
    muxy1=muxy1+qxy(:,i);
    murz1=murz1+qrz(:,i);
end
muxy=(1/100)*muxy1;
murz=(1/100)*murz1;
%plot(t,muxy(1),'b',t,muxy(2),'g',t,murz(1),'r',t,murz(2),'k')
E1=[0 1;1 0];
E2=[1 0;0 -1];
% E3=[0 -1;1 0];
yy=0; xx=0;
rr=0; zz=0;
for i=1:100
    yy=yy+((qxy(:,i)-muxy)'*E1*(qxy(:,i)-muxy));
    xx=xx+((qxy(:,i)-muxy)'*E2*(qxy(:,i)-muxy));
    
    zz=zz+((qrz(:,i)-murz)'*E1*(qrz(:,i)-murz));
    rr=rr+((qrz(:,i)-murz)'*E2*(qrz(:,i)-murz));
end
theta=(1/2)*atan2(yy,xx);
alpha=(1/2)*atan2(zz,rr);

Rxy=[cos(theta) -sin(theta);sin(theta) cos(theta)];
Rrz=[cos(alpha) -sin(alpha);sin(alpha) cos(alpha)];
H1xy=eye(2)+Rxy^2*E2;
H1rz=eye(2)+Rrz^2*E2;
H2xy=eye(2)-Rxy^2*E2;
H2rz=eye(2)-Rrz^2*E2;
H3xy=Rxy^2*E1;
H3rz=Rrz^2*E1;

st1=0; st2=0;
st3=0; st4=0;
for i=1:100
    st1=st1+((qxy(:,i)-muxy)'*H1xy*(qxy(:,i)-muxy));
    st2=st2+((qxy(:,i)-muxy)'*H2xy*(qxy(:,i)-muxy));
    
    st3=st3+((qrz(:,i)-murz)'*H1rz*(qrz(:,i)-murz));
    st4=st4+((qrz(:,i)-murz)'*H2rz*(qrz(:,i)-murz));
end
s1=(1/(2*99))*st1;
s2=(1/(2*99))*st2;
s3=(1/(2*99))*st3;
s4=(1/(2*99))*st4;

K_muxy=2*eye(2);
K_murz=2*eye(2);
k_theta=2;
k_alpha=2;
k_s1=2; k_s2=2;
k_s3=2; k_s4=2;

muxy_d=[3;23];
murz_d=[23.1948;30];
theta_d=0;
alpha_d=0;
s1_d=10.8574;
s2_d=0.3518;
s3_d=0.3518;%10.1607;
s4_d=0.3518;

muxydot=K_muxy*(muxy_d-muxy);
murzdot=K_murz*(murz_d-murz);
thetadot=k_theta*(theta_d-theta);
alphadot=k_alpha*(alpha_d-alpha);
s1dot=k_s1*(s1_d-s1);
s2dot=k_s2*(s2_d-s2);
s3dot=k_s3*(s3_d-s3);
s4dot=k_s4*(s4_d-s4);

plot(t,s1dot,'b')

plot(t,s2dot,'r')

plot(t,s3dot,'g')

plot(t,s4dot,'k')

adot=[muxydot;thetadot;s1dot;s2dot];
bdot=[murzdot;alphadot;s3dot;s4dot];

muxymat=[];
thetamat=[];
s1mat=[];
s2mat=[];
murzmat=[];
alphamat=[];
s3mat=[];
s4mat=[];
for i=1:100
    muxymat=[muxymat (1/100)*eye(2)];
    thetamat=[thetamat (1/(99*(s1-s2)))*((qxy(:,i)-muxy)'*H3xy)];
    s1mat=[s1mat (1/99)*(qxy(:,i)-muxy)'*H1xy];
    s2mat=[s2mat (1/99)*(qxy(:,i)-muxy)'*H2xy];
    
    murzmat=[murzmat (1/100)*eye(2)];
    alphamat=[alphamat (1/(99*(s3-s4)))*((qrz(:,i)-murz)'*H3rz)];
    s3mat=[s3mat (1/99)*(qrz(:,i)-murz)'*H1rz];
    s4mat=[s4mat (1/99)*(qrz(:,i)-murz)'*H2rz];
end
phixy=[muxymat;thetamat;s1mat;s2mat];
phirz=[murzmat;alphamat;s3mat;s4mat];

qxydot=phixy'*inv(phixy*phixy')*adot;
qrzdot=phirz'*inv(phirz*phirz')*bdot;

qdot=[];
for i=1:2:200
    qdot=[qdot;qxydot(i);qxydot(i+1);qrzdot(i)];
end
%size(qdot)
%size(qrzdot)
% qw
