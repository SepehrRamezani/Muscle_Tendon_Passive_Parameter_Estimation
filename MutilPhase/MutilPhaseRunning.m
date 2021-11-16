
clear all

model_path  = fullfile(cd,'gait2392_simbody_scaled.osim');
Misc.IKfile = {fullfile('IK.mot'),fullfile('IK.mot')};
Misc.IDfile = {fullfile('inverse_dynamics.sto'),fullfile('inverse_dynamics.sto')};
time=[0.516 1.95;0.516 1.95];
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r','hip_adduction_r','hip_rotation_r','ankle_angle_l','knee_angle_l','hip_flexion_l','hip_adduction_l','hip_rotation_l'}; 

Out_path = fullfile(cd);        
Misc.PlotBool = 0;
Misc.OutName = 'Knee_Flexion_';
Misc.MRSBool=1;
[Results,DatStore] = solveMuscleRedundancy(model_path,time,Out_path,Misc);
Datafolder=cd;
F_fnames='controls.sto';
% Title=["version=1","nRows=%d","nColumns=%d","InDegrees=no","endheader"];
% Title='\nversion=1\nnRows=%d\nnColumns=%d\nInDegrees=no\nendheader\n';
MDatadata=[Results.Time.genericMRS (Results.MActivation.genericMRS)'];
delimiterIn='\t';
Dataheader=Results.MuscleNames;
Dataheader=[{'time'} Dataheader];
% makefile(Datafolder,F_fnames,Dataheader,MDatadata,5,delimiterIn);

