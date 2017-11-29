function create_load_controller(fid,Controller,ub,lb,Kaw,indent,st)


controller_d=c2d(ss(Controller),st);

fprintf(fid,'%scontroller:\n',repmat(' ',1,indent));
print_driven_mass_controller(fid,controller_d,ub,lb,Kaw,indent+2);

