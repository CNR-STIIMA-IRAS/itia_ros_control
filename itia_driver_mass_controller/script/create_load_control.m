% clear all;clc;close all;
% 
% %fid=fopen('test.yaml','w');
% 
% s=tf('s');
% 
% st=1e-3;
% 
% k=1235.687013;
% J=13.8;
% h=9.8;
% fl=0;
% %tau=0.2;
% tau=0.2;
% Controller=(J*s^2 + h*s + k)/(k*s^2*tau^3 + 3*k*s*tau^2 + 3*k*tau - h);

fid=fopen('tmp.txt','w');
st=1e-3;

controller_d=c2d(ss(Controller),st);
ub=[0.2];
lb=[-0.2];
Kaw=50e-3;

indent=8;

fprintf(fid,'      controller:\n')
print_driven_mass_controller(fid,controller_d,ub,lb,Kaw,indent);

if fid~=1
  fclose(fid)
end