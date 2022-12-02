function [P_Gonio_H,P_Gonio_K,P_Gonio_A,P_Bidoex_Calibration,Torqueref]= GnCalib (Datafolder,psname,DStime,plotflage)
P_Bidoex_Calibration=[139.01,-24.46];
Fname =append(psname,"_GnCalib_");
Names=["AnkleDorsi15","AnkleDorsi10","Ankle0","AnklePlant10","AnklePlant30","AnklePlant50"...
    ,"Hip0","Hip30","Hip60","Hip90"...
    ,"Knee0","Knee30","Knee45","Knee60","Knee90"];
AnkelGnCalibdata=[];
HipGnCalibdata=[];
KneeGnCalibdata=[];

for i=1:length(Names)
    GnCaldatadir=importdata(append(Datafolder,"\",Fname,Names(i),".csv"));
    [Gr,Gc]=find(strncmp(GnCaldatadir.textdata,'Gn',2));
    %     [Tr,Tc]=find(strncmp(GnCaldatadir.textdata,'X',1));
    Tc=[1,3];
    if length (GnCaldatadir.textdata)<4
        Tc=[1 1];
    end
    ChAindx=find(GnCaldatadir.data(:,Tc(1))<=2.5 & GnCaldatadir.data(:,Tc(1))>=1.5);
    ChA=mean(GnCaldatadir.data(ChAindx,Gc(1)));
    ChBindx=find(GnCaldatadir.data(:,Tc(2))<=2.5 & GnCaldatadir.data(:,Tc(2))>=1.5);
    ChB=mean(GnCaldatadir.data(ChBindx,Gc(2)));
    
    if contains(Names(i),"Ankle")
        AnkelGnCalibdata=[AnkelGnCalibdata;ChA ChB];
    elseif contains(Names(i),"Hip")
        HipGnCalibdata=[HipGnCalibdata;ChA ChB];
    elseif contains(Names(i),"Knee")
        KneeGnCalibdata=[KneeGnCalibdata;ChA ChB];
    end
    
end
y=[20,10,0,-10,-30,-50];
y=y./180*pi();
xa=AnkelGnCalibdata;
P_Gonio_A = polyfit(xa(:,2)',y,1);
y1 = polyval(P_Gonio_A,xa(:,2));
if plotflage
    plot(xa(:,1),y,'*',xa(:,2),y,'*',xa(:,2),y1);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Ankle');
end


ydeg=[0,30,60,90];
y=ydeg./180*pi();
xh=HipGnCalibdata;
P_Gonio_H = polyfit(xh(:,2)',y,2);
new_xh=xh(1,2):xh(1,2)/20:xh(end,2);
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
y=[0,45,60,90];
y=y./180*pi();
% xk=[KneeGnCalibdata(1,:);KneeGnCalibdata([1,3,4,5],:)];
xk=KneeGnCalibdata([1,3,4,5],:);
P_Gonio_K = polyfit(xk(:,2)',y,2);
new_xk=xk(1,2):xk(end,2)/20:xk(end,2);
y1 = polyval(P_Gonio_K,new_xk);
if plotflage
    figure
    plot(xk(:,1),y,'*',xk(:,2),y,'*',new_xk,y1);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Knee');
end
Fname =append(psname,"_BiodexCalib");
BiodexCaldata=importdata(append(Datafolder,"\",Fname,".csv"));
[Gdata,Gheader]= ReshapingData(BiodexCaldata,DStime);
[Gr,Gc]=find(strncmp(Gheader,'Biodex',6));
timeindx=(Gdata(:,1)>=2&Gdata(:,1)<=4);
VolTorqueref=mean(Gdata(timeindx,Gc(1)));
Torqueref=polyval(P_Bidoex_Calibration,VolTorqueref);

end