syms t1 t2 t3 t4 t5 t6
syms nx ny nz ox oy oz ax ay az px py pz real
syms  t1 t2 t3 t4 t5 t6
theta=[t1 t2 t3 t4 t5 t6];
w=[0 -pi/2 0 -pi/2 pi/2 -pi/2];
d=[0 0 0 770 0 0];%偏置参数
a=[0 200 600 115 0 0];%数组索引1对应元素0
T=cell(1,6);
T06_ver=cell(1,201);
for h=1:6
   T{1,h}=[cos(theta(h)),-sin(theta(h)),0,a(h);...
sin(theta(h))*int8(cos(w(h))),cos(theta(h))*int8(cos(w(h))),int8(-sin(w(h))),-d(h)*sin(w(h));...
sin(theta(h))*int8(sin(w(h))),cos(theta(h))*int8(sin(w(h))),int8(cos(w(h))),d(h)*int8(cos(w(h)));0,0,0,1];
   fprintf('The T%d%d is\n',h-1,h);
   disp(T{1,h});  
end
T06=T{1,1}*T{1,2}*T{1,3}*T{1,4}*T{1,5}*T{1,6}
for i=0:200
   %设定末端运动轨迹
%    cs=(100-i)/100;%半圆追随轨迹
%    nx=cs;     ox=sqrt(1-cs^2);ax=0;px=600+i;
%    ny=sqrt(1-cs^2);oy=cs;     ay=0;py=sqrt(10000-(px-700)^2); 
%    nz=0;          oz=0;      az=1;pz=600; 
   sita=cos(i/10)/sin(i/10);%圆柱螺线追随轨迹
   gama=atan(0.01);
   nx=1;ox=0;ax=0;px=600+i*sin(i/10);
   ny=0;oy=1;ay=0;py=600+i*cos(i/10); 
   nz=0;oz=0; az=1;pz=600+i; 
    px_ver1(i+1)=px;py_ver1(i+1)=py;pz_ver1(i+1)=pz;
   %*****逆运动学计算*******%
   %计算关节角1
   theta1(i+1)=atan2(py,px);
   %计算关节角2
   v2=px*cos(theta1(i+1))+py*sin(theta1(i+1))-a(2);
   u2=pz;
   p2=sqrt(u2^2+v2^2);
   temp2=(a(4)^2+d(4)^2-v2^2-u2^2-a(3)^2)/(2*a(3)*p2);
   theta2(i+1)=atan2(v2,u2)+atan2(temp2,-sqrt(1-temp2^2));
   %theta2(i+1)=atan2(v2,u2)+atan2(temp2,sqrt(1-temp2^2));
   %计算关节角3
   p3=sqrt(a(4)^2+d(4)^2);
   temp3=(px*cos(theta1(i+1))+py*sin(theta1(i+1))-a(2)-a(3)*cos(theta2(i+1)))/p3;
   theta3(i+1)=atan2(a(4),d(4))-atan2(temp3,-sqrt(1-temp3^2))-theta2(i+1);
   %theta3(i+1)=atan2(a(4),d(4))-atan2(temp3,sqrt(1-temp3^2))-theta2(i+1);
   %theta3(i+1)=roundn(theta3(i+1),4);
   %计算关节角4
   v4=ax*cos(theta1(i+1))*cos(theta2(i+1)+theta3(i+1))-az*sin(theta2(i+1)+theta3(i+1))+ay*sin(theta1(i+1))*cos(theta2(i+1)+theta3(i+1));
   u4=ay*cos(theta1(i+1))-ax*sin(theta1(i+1));
   theta4(i+1)=atan2(u4,v4);
   %计算关节角5
   temp5=sqrt(u4^2+v4^2);
   %theta5(i+1)=atan2(temp5,sqrt(1-temp5^2));
   theta5(i+1)=atan2(temp5,-sqrt(1-temp5^2));
   theta5(i+1)=roundn(theta5(i+1),4);
   %计算关节角6
   s6=-nx*(cos(theta1(i+1))*sin(theta4(i+1))*sin(theta2(i+1)+theta3(i+1))-sin(theta4(i+1))*cos(theta4(i+1)))-ny*(sin(theta1(i+1))*sin(theta4(i+1))*cos(theta2(i+1)+theta3(i+1))+cos(theta1(i+1))*cos(theta4(i+1)))+nz*(sin(theta2(i+1)+theta3(i+1))*sin(theta4(i+1)));
   c6=nx*(cos(theta5(i+1))*(cos(theta1(i+1))*cos(theta4(i+1))*cos(theta2(i+1)+theta3(i+1))+sin(theta1(i+1))*sin(theta4(i+1)))-cos(theta1(i+1))*sin(theta5(i+1))*sin(theta2(i+1)+theta3(i+1)))+ny*(cos(theta5(i+1))*(cos(theta4(i+1))*sin(theta1(i+1))*sin(theta2(i+1)+theta3(i+1))-cos(theta1(i+1))*cos(theta4(i+1)))-sin(theta1(i+1))*sin(theta5(i+1))*sin(theta2(i+1)+theta3(i+1)))-nz*(sin(theta5(i+1))*cos(theta2(i+1)+theta3(i+1))+cos(theta4(i+1))*cos(theta5(i+1))*sin(theta2(i+1)+theta3(i+1)));
   theta6(i+1)=atan2(s6,c6);
   %调整角的范围
   for l=1:3
      if theta1(i+1)>pi
         theta1(i+1)=theta1(i+1)-2*pi;
         elseif theta1(i+1)<-pi
                  theta1(i+1)=theta1(i+1)+2*pi;
      end
      if theta2(i+1)>pi
         theta2(i+1)=theta2(i+1)-2*pi;
         elseif theta2(i+1)<-pi
                  theta2(i+1)=theta2(i+1)+2*pi;
      end
      if theta3(i+1)>pi
         theta3(i+1)=theta3(i+1)-2*pi;
         elseif theta3(i+1)<-pi
                  theta3(i+1)=theta3(i+1)+2*pi;
      end
      if theta4(i+1)>pi
         theta4(i+1)=theta4(i+1)-2*pi;
         elseif theta4(i+1)<-pi
                  theta4(i+1)=theta4(i+1)+2*pi;
      end
      if theta5(i+1)>pi
         theta5(i+1)=theta5(i+1)-2*pi;
         elseif theta5(i+1)<-pi
                  theta5(i+1)=theta5(i+1)+2*pi;
      end
      if theta6(i+1)>pi
         theta6(i+1)=theta6(i+1)-2*pi;
         elseif theta6(i+1)<-pi
                  theta6(i+1)=theta6(i+1)+2*pi;
      end
   end
   %****正向运动学计算******%
   T06_ver{1,i+1}=eval(subs(T06,{t1 t2 t3 t4 t5 t6},{theta1(i+1) theta2(i+1) theta3(i+1) theta4(i+1) theta5(i+1) theta6(i+1)}));
   px_ver2(i+1)=T06_ver{1,i+1}(1,4);
   py_ver2(i+1)=T06_ver{1,i+1}(2,4);
   pz_ver2(i+1)=T06_ver{1,i+1}(3,4);
end
figure(1)
subplot(2,3,1)
plot(theta1);
title('θ1随时间变化');
xlabel('时间t/0.1s');
ylabel('转角θ1/rad');
subplot(2,3,2)
plot(theta2);
title('θ2随时间变化');
xlabel('时间t/0.1s');
ylabel('转角θ2/rad');
subplot(2,3,3)
plot(theta3);
title('θ3随时间变化');
xlabel('时间t/0.1s');
ylabel('转角θ3/rad');
subplot(2,3,4)
plot(theta4);
title('θ4随时间变化');
xlabel('时间t/0.1s');
ylabel('转角θ4/rad');
subplot(2,3,5)
plot(theta5);
title('θ5随时间变化');
xlabel('时间t/0.1s');
ylabel('转角θ5/rad');
subplot(2,3,6)
plot(theta6);
title('θ6随时间变化');
xlabel('时间t/0.1s');
ylabel('转角θ6/rad');
figure(2)
subplot(1,2,1)
plot3(px_ver1,py_ver1,pz_ver1);
title('原设定轨迹');
xlabel('x');ylabel('y'); zlabel('z');  
grid on;
axis square;
subplot(1,2,2)
plot3(px_ver2,py_ver2,pz_ver2);
title('逆向与正向运动学计算所得轨迹');
xlabel('x');ylabel('y'); zlabel('z');  
grid on;
axis square;
