% 模（Modulus）：quatmod(p)    %  5.4772
% 范数（Norm）：quatnorm(p)    0
% 单位化（Normalize）：quatnormalize(p)    %  0.1826    0.3651    0.5477    0.7303
% 求逆（Inverse）：quatinv(p)    %0.0333   -0.0667   -0.1000   -0.1333
% 四元数除法：quatdivide(q,p)    %0.6667         0   -0.6667   -0.3333
% 四元数乘法：quatmultiply(p,q)  % -12     6    24    12
% 共轭四元数：quatconj(p)      % 1    -2    -3    -4 
% DCM QUAT ELUR 转换
% dcm2quat quat2dcm 
% angle2dcm dcm2angle    欧拉角变dcm 反转  
% quat2angle、angle2quat 四元数和欧拉角互换的函数
% quaternion  方向余弦单位旋转
% 单位旋转 r=rotx(pi/2) ，roty rotz
% 记忆 沿着x轴旋转 alpha 
% 对应第一轴                1     0      0 
%     第二轴第一个0，
%     第二个为为自己映射cos(alpha)
%     第三个为sin(alpha) 正负取值 为z->y 所以取负号
%                          0    cos(alpha)  -sin(alpha)
%     第三轴： 第一个为0
%            第二个为sin(alpha) y->z 取正号
%            第三个取自己cos(alpha)
%            so            0     sin(alpha)  cos(alpha)
%matlab默认的角度单位为弧度，这里可以用度数作为单位
r=rotx(30)
b=rotx(30/180*3.14)
b=rotx(30/180*pi)
b=rotx(pi/2)
b=rotx(90)
%求出R等效的任意旋转变换的旋转轴矢量vec和转角theta
[theta,vec] = angvec2tr(r);
tr2angvec(r)

z=rotz(30/180*pi)
y=roty(30/180*pi)
x=rotx(30/180*pi)

% Matlab code by MulinB, Aerospace Toolbox is needed  
% Gimbal Lock experiments  
yaw1 =   45;  
pitch1 = 0;  
roll1 =  0;  
yaw2 =   0;  
pitch2 = 0;  
roll2 =  60;  
% quat2angle(p1)/pi*180  
R1 = angle2dcm(yaw1/180*pi,pitch1/180*pi,roll1/180*pi);   
R2 = angle2dcm(yaw2/180*pi,pitch2/180*pi,roll2/180*pi);  
disp(R1);disp(R2);  
p1=dcm2quat(R1);
p1=quatnormalize(p1);
p2=dcm2quat(R2);
p2=quatnormalize(p2);
a=R2*R1;
a=a';
b=quatmultiply(p2,p1);
b=quatnormalize(b);
c=quat2dcm(b)