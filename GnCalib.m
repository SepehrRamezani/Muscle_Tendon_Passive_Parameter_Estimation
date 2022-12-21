function [P_Gonio_H,P_Gonio_K,P_Gonio_A,P_Bidoex_Torque_Calibration,Torqueref,P_Bidoex_Motion_Calibration]= GnCalib (Datafolder,psname,Info,plotflage)

Fname =append(psname,"_GnCalib_");
Dstime=Info.DStime/5;
Names=[append("AnkleDorsi",string(Info.MaxDorsi_Calib)),"AnkleDorsi10"...
    ,"Ankle0","AnklePlant10","AnklePlant30",append("AnklePlant",string(Info.MaxPlant_Calib))...
    ,"Hip0","Hip30","Hip60","Hip90"...
    ,"Knee0","Knee15","Knee30","Knee45","Knee75","Knee90"];
AnkelGnCalibdata=[];
HipGnCalibdata=[];
KneeGnCalibdata=[];

for i=1:length(Names)
    GnCaldatadir=importdata(append(Datafolder,"\",Fname,Names(i),".csv"));
    [GnCaldatadir.data,Gheader]= ReshapingData(GnCaldatadir,Dstime);
    [Gr,Gc]=find(contains(Gheader,'Goni'));
    %     [Tr,Tc]=find(strncmp(GnCaldatadir.textdata,'X',1));
%     Tc=Gc-1;
%     if length (GnCaldatadir.textdata)<4
%         Tc=[1 1];
%     end
    Chindx=find(GnCaldatadir.data(:,1)<=2.5 & GnCaldatadir.data(:,1)>=1.5);
    ChA=mean(GnCaldatadir.data(Chindx,Gc(1)));
    ChB=mean(GnCaldatadir.data(Chindx,Gc(2)));
    
    if contains(Names(i),"Ankle")
        AnkelGnCalibdata=[AnkelGnCalibdata;ChA ChB];
    elseif contains(Names(i),"Hip")
        HipGnCalibdata=[HipGnCalibdata;ChA ChB];
    elseif contains(Names(i),"Knee")
        KneeGnCalibdata=[KneeGnCalibdata;ChA ChB];
    end
    
end
ydeg=[20,10,0,-10,-30,-50];
y=ydeg./180*pi();
xa=AnkelGnCalibdata;
P_Gonio_A = polyfit(xa(:,1)',y,1);
new_xa=linspace(xa(1,1),xa(end,1),20);
y1 = polyval(P_Gonio_A,new_xa);
if plotflage
    plot(xa(:,1),ydeg,'*',xa(:,2),ydeg,'*',new_xa,y1*180/3.14);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Ankle');
end


ydeg=[0,30,60,90];
y=ydeg./180*pi();
xh=HipGnCalibdata;
P_Gonio_H = polyfit(xh(:,2)',y,2);
new_xh=linspace(xh(1,2),xh(end,2),20);
y1 = polyval(P_Gonio_H,new_xh);
if plotflage
    figure
    plot(xh(:,1),ydeg,'*',xh(:,2),ydeg,'*',new_xh,y1*180/3.14);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Hip');
end

%% 2 zeros added for regression since zero is important angle
y=[0,10,20,40,60,90];
y=y./180*pi();
% xk=[KneeGnCalibdata(1,:);KneeGnCalibdata([1,3,4,5],:)];
xk=KneeGnCalibdata;
P_Gonio_K = polyfit(xk(:,2)',y,2);
new_xk=linspace(xk(1,2),xk(end,2),20);
y1 = polyval(P_Gonio_K,new_xk);
if plotflage
    figure
    plot(xk(:,1),y,'*',xk(:,2),y,'*',new_xk,y1);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Knee');
end

%% calibratio of biodex for Knee
Fname =append(psname,"_BiodexCalib_Knee");
BiodexCaldata=importdata(append(Datafolder,"\",Fname,".csv"));
[Gdata,Gheader]= ReshapingData(BiodexCaldata,Dstime);
[Gr,Gc]=find(strncmp(Gheader,'Biodex',6));
Event=EventDetection(Dstime,Gdata(:,Gc(2)),0.1);
timeindx=Event.ConstantTime;
VolTorqueref=mean(Gdata(timeindx,Gc(1)));
HAngle=mean(Gdata(timeindx,Gc(2)));
Fname =append(psname,"_BiodexCalib_Knee_V");
BiodexMotionCaldata=importdata(append(Datafolder,"\",Fname,".csv"));
[BiodexMdata,BiodexMheader]= ReshapingData(BiodexMotionCaldata,Dstime);
Mtimeindx=(BiodexMdata(:,1)>=1.5&BiodexMdata(:,1)<=2.5);
VAngle=mean(BiodexMdata(Mtimeindx,Gc(2)));
Angle=[0 90];
x=[HAngle VAngle];
P_Bidoex_Motion_Calibration_Knee=polyfit(x,Angle,1);
P_Bidoex_Torque_Calibration=[139.01,-24.46];
Torqueref_KneeArm=polyval(P_Bidoex_Torque_Calibration,VolTorqueref);
%% Calibration of Biodex for ankle  
Fname =append(psname,"_BiodexCalib_Ankle");
BiodexCaldata=importdata(append(Datafolder,"\",Fname,".csv"));
[GAdata,Gheader]= ReshapingData(BiodexCaldata,0.05);
[Gr,Gc]=find(strncmp(Gheader,'Biodex',6));
Event=EventDetection(Dstime,GAdata(:,Gc(2)),0.02);
timeindx=Event.ConstantTime;
VolTorqueref_A=mean(GAdata(timeindx,Gc(1)));
HAngle_Aakle=mean(GAdata(timeindx,Gc(2)));
Biodex_Ankle_offset=-1*P_Bidoex_Motion_Calibration_Knee(1)*HAngle_Aakle;
Torqueref_AnkleArm=polyval(P_Bidoex_Torque_Calibration,VolTorqueref_A);
P_Bidoex_Motion_Calibration=[P_Bidoex_Motion_Calibration_Knee,Biodex_Ankle_offset];
Torqueref=[Torqueref_KneeArm,Torqueref_AnkleArm];
end