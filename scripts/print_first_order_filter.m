function print_first_order_filter(fid,time_constant,st,indent)

A=exp(-st/time_constant);
B=1-A;
C=1;
D=0;
init_matrix=1;
x0=0;

Baw=0;
ub=1e8;
lb=-1e8;
spaces=repmat(' ',1,indent);
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