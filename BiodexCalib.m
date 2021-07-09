function [PTorque,P_Position]= BiodexCalib(dfolder,subjectname,plotflage)
Fname =append(subjectname,"_BiodexCalib_Angle_");
Names=["Knee_H","Knee_V"];
BiodexAngle=[];
HipGnCalibdata=[];
KneeGnCalibdata=[];
PTorque=[-141.81 25.047];
for i=1:length(Names)
    BiodexCaldatadir=importdata(append(dfolder,"\",Fname,Names(i),".csv"));
    [Gr,Anglecolumnindx]=find(contains(BiodexCaldatadir.textdata,'.B'));
    %     [Tr,Tc]=find(strncmp(GnCaldatadir.textdata,'X',1));
%     Tc=[1,3];
%     if length (BiodexCaldatadir.textdata)<4
%         Tc=[1 1];
%     end
    AngleRowindx=find(BiodexCaldatadir.data(:,Anglecolumnindx-1)<=2.5 & BiodexCaldatadir.data(:,Anglecolumnindx-1)>=1.5);
    BiodexAngle=[BiodexAngle mean(BiodexCaldatadir.data(AngleRowindx,Anglecolumnindx(1)))];   
end
y=[0,90];
y=y./180*pi();
x=BiodexAngle;
Pa = polyfit(x,y,1);
P_Position = polyval(Pa,x(:,2));
end