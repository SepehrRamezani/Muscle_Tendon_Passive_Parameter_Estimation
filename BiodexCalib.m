function [Weight_Coefficient,P_Biodex_Torque,P_Biodex_Knee_Angle,P_Biodex_Ankle_Angle]= BiodexCalib(Datafolder,psname,Calibration_Limits,plotflage)
Fname =append(psname,"BiodexCalib_Angle_");
Names=["Knee_H","Knee_V","Ankle_H","Ankle_V"];
BiodexKneeAngle=[];
BiodexAnkleAngle=[];
P_Biodex_Torque=[-141.81 25.047];
for i=1:length(Names)
    BiodexCaldatadir=importdata(append(Datafolder,"\",Fname,Names(i),".csv"));
    [Gr,Anglecolumnindx]=find(contains(BiodexCaldatadir.textdata,'.B')&contains(BiodexCaldatadir.textdata,'Biodex'));
    [Gr,Torquecolumnindx]=find(contains(BiodexCaldatadir.textdata,'.A')&contains(BiodexCaldatadir.textdata,'Biodex'));
    AngleRowindx=find(BiodexCaldatadir.data(:,Anglecolumnindx-1)<=2.5 & BiodexCaldatadir.data(:,Anglecolumnindx-1)>=1.5);
    ToqueRowindx=find(BiodexCaldatadir.data(:,Torquecolumnindx-1)<=2.5 & BiodexCaldatadir.data(:,Torquecolumnindx-1)>=1.5);
     if contains(Names(i),"Knee")
        BiodexKneeAngle=[BiodexKneeAngle mean(BiodexCaldatadir.data(AngleRowindx,Anglecolumnindx))];
       if contains(Names(i),"H")
        BiodexKneeTorque=mean(BiodexCaldatadir.data(ToqueRowindx,Torquecolumnindx));
        Weight_Coefficient=polyval(P_Biodex_Torque,BiodexKneeTorque);
       end
    elseif contains(Names(i),"Ankle")
        BiodexAnkleAngle=[BiodexAnkleAngle mean(BiodexCaldatadir.data(AngleRowindx,Anglecolumnindx(1)))];
     else
         warning('Invalid Data') 
     end
end
y=[0,90];
y=y./180*pi();
x=BiodexKneeAngle;
P_Biodex_Knee_Angle = polyfit(x,y,1);
y1 = polyval(P_Biodex_Knee_Angle,x);
if plotflage
    plot(x,y,'*',x,y1);
    title('Biodex Angle Calibration Knee')
end

y=[Calibration_Limits(1),Calibration_Limits(2)];
y=y./180*pi();
x=BiodexAnkleAngle;
P_Biodex_Ankle_Angle = polyfit(x,y,1);
y1 = polyval(P_Biodex_Ankle_Angle,x);
if plotflage
    figure
    plot(x,y,'*',x,y1);
    title('Biodex Angle Calibration Ankle')
end
end