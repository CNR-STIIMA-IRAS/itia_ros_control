function print_filter(fid,sys,ub,lb,Kaw,indent)
spaces=repmat(' ',1,indent);
sys=canon(sys);
[A,B,C,D]=ssdata(ss(sys));
init_matrix=zeros(order(sys),1);

Baw=zeros(size(A,1),size(C,1));


if ~isempty(idx)
  
  idx2=setdiff(1:order(sys),1);
  A=A([idx idx2],[idx idx2]);
  B=B([idx idx2],:);
  C=C(:,[idx idx2]);
  B(1,1)=B(1,:)*C(1,1);
  C(1,1)=1;
  Baw(1,:)=Kaw;
  init_matrix(idx,1)=1;
  
  B=[[0;B(2:end)] [B(1);zeros(order(sys)-1,1)] zeros(order(sys),1)];
end

if isempty(A);
  A=0;
  B=zeros(1,size(D,2));
  C=zeros(size(D,1),1);
  Baw=zeros(1,size(D,1));
  B=[B zeros(order(sys),2)];
end
D=[D 0 0 ];

x0=zeros(size(A,1),1);
fprintf(fid,'%sA:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(A,indent));
fprintf(fid,'%sB:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(B,indent));
fprintf(fid,'%sBaw:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(Baw,indent));
fprintf(fid,'%sC:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(C,indent));
fprintf(fid,'%sD:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(D,indent));
fprintf(fid,'%smax_output:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(ub,indent));
fprintf(fid,'%smin_output:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(lb,indent));
fprintf(fid,'%sinitial_state:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(x0,indent));
fprintf(fid,'%sinitialization_matrix:\n',spaces);
fprintf(fid,'%s',save_matrix_to_yaml(init_matrix,indent));
end