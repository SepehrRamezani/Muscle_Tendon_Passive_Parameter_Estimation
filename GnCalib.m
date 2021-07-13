function [P_Gonio_H,P_Gonio_K,P_Gonio_A]= GnCalib (Datafolder,psname,plotflage)

Fname =append(psname,"GnCalib_");
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
y=[15,10,0,-10,-30,-50];
y=y./180*pi();
x=AnkelGnCalibdata;
P_Gonio_A = polyfit(x(:,2)',y,1);
y1 = polyval(P_Gonio_A,x(:,2));
if plotflage
    plot(x(:,1),y,'*',x(:,2),y,'*',x(:,2),y1);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Ankle');
end


ydeg=[0,30,60,90];
y=ydeg./180*pi();
x=HipGnCalibdata;
P_Gonio_H = polyfit(x(:,2)',y,2);
y1 = polyval(P_Gonio_H,x(:,2));
if plotflage
    figure
    plot(x(:,1),ydeg,'*',x(:,2),ydeg,'*',x(:,2),y1*180/3.14);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Hip');
end


y=[0,0,45,60,90];
y=y./180*pi();
x=[KneeGnCalibdata(1,:);KneeGnCalibdata([1,3,4,5],:)];
P_Gonio_K = polyfit(x(:,2)',y,2);
y1 = polyval(P_Gonio_K,x(:,2));
if plotflage
    figure
    plot(x(:,1),y,'*',x(:,2),y,'*',x(:,2),y1);
    xlabel('Biodex Output (Voltage)');
    ylabel('Angle (Degree)');
    legend('Ch A','Ch B');
    title('Goinometer Calibration Data for Knee');
end

end