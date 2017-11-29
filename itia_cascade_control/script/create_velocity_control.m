clear all;clc;close all;

fid=1;
%fid=fopen('test.yaml','w');

s=tf('s');

st=1e-3;
Kp=14;
Ki=250;
Kd=1;
Tfd=0.2;
Tf=0.005;
velocity_filter=1/(Tf*s+1);

velocity_controller=Kp*1+Ki/(s)+Kd*s/(Tfd*s+1);


velocity_controller_d=c2d(ss(velocity_controller),st);
velocity_filter_d=c2d(ss(velocity_controller),st);
ub=[95];
lb=[-95];
Kaw=50e-3;

indent=8;
%fprintf(fid,'      velocity_filter:\n')
%print_vel_controller(fid,velocity_filter_d,ub,lb,Kaw,indent);

fprintf(fid,'      velocity:\n')
print_vel_controller(fid,velocity_controller_d,ub,lb,Kaw,indent);

if fid~=1
  fclose(fid)
end