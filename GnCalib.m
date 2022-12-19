function [P_Gonio_H,P_Gonio_K,P_Gonio_A,P_Bidoex_Torque_Calibration,Torqueref,P_Bidoex_Motion_Calibration]= GnCalib (Datafolder,psname,Info,plotflage)

Fname =append(psname,"_GnCalib_");

Names=[append("AnkleDorsi",string(Info.MaxDorsi_Calib)),"AnkleDorsi10"...
    ,"Ankle0","AnklePlant10","AnklePlant30",append("AnklePlant",string(Info.MaxPlant_Calib))...
    ,"Hip0","Hip30","Hip60","Hip90"...
    ,"Knee0","Knee15","Knee30","Knee45","Knee75","Knee90"];
AnkelGnCalibdata=[];
HipGnCalibdata=[];
KneeGnCalibdata=[];

for i=1:length(Names)
    GnCaldatadir=importdata(append(Datafolder,"\",Fname,Names(i),".csv"));
    [GnCaldatadir.data,Gheader]= ReshapingData(GnCaldatadir,Info.DStime/5);
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
Fname =append(psname,"_BiodexCalib_Knee");
BiodexCaldata=importdata(append(Datafolder,"\",Fname,".csv"));
[Gdata,Gheader]= ReshapingData(BiodexCaldata,0.05);
[Gr,Gc]=find(strncmp(Gheader,'Biodex',6));
normAngle=Gdata(:,Gc(2))/max(abs(Gdata(:,Gc(2))));
timeindx=find((abs(diff(normAngle)./diff(Gdata(:,1))))<=0.02);
selecttime=find(diff(timeindx)>1);
timeindx=timeindx(1:selecttime(1)-1);
VolTorqueref=mean(Gdata(timeindx,Gc(1)));
HAngle=mean(Gdata(timeindx,Gc(2)));
Fname =append(psname,"_BiodexCalib_Knee_V");
BiodexMotionCaldata=importdata(append(Datafolder,"\",Fname,".csv"));
[BiodexMdata,BiodexMheader]= ReshapingData(BiodexMotionCaldata,Info.DStime);
Mtimeindx=(BiodexMdata(:,1)>=1.5&BiodexMdata(:,1)<=2.5);
VAngle=mean(BiodexMdata(Mtimeindx,Gc(2)));
Angle=[0 90];
x=[HAngle VAngle];
P_Bidoex_Motion_Calibration=polyfit(x,Angle,1);
P_Bidoex_Torque_Calibration=[139.01,-24.46];

Torqueref=polyval(P_Bidoex_Torque_Calibration,VolTorqueref);

end