function [Ph,Pk,Pa]= GnCalib (dfolder,subjectname,plotflage)

Fname =append(subjectname,"_GnCalib_");
Names=["AnkleDors20","AnkleDors10","Ankle0","AnklePlant10","AnklePlant30","AnklePlant50"...
    ,"Hip0","Hip30","Hip60","Hip90"...
    ,"Knee0","Knee30","Knee45","Knee60","Knee90"];
AnkelGnCalibdata=[];
HipGnCalibdata=[];
KneeGnCalibdata=[];

for i=1:length(Names)
    GnCaldatadir=importdata(append(dfolder,"\",Fname,Names(i),".csv"));
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
x=AnkelGnCalibdata;
Pa = polyfit(x(:,2)',y,1);
y1 = polyval(Pa,x(:,2));
if plotflage
    plot(x(:,1),y,'*',x(:,2),y,'*',x(:,2),y1);
    legend('A','B')
    title('Ankle')
end


y=[0,30,60,90];
y=y./180*pi();
x=HipGnCalibdata;
Ph = polyfit(x(:,2)',y,2);
y1 = polyval(Ph,x(:,2));
if plotflage
    figure
    plot(x(:,1),y,'*',x(:,2),y,'*',x(:,2),y1);
    legend('A','B')
    title('Hip')
end


y=[0,30,45,60,90];
y=y./180*pi();
x=KneeGnCalibdata;
Pk = polyfit(x(:,2)',y,1);
y1 = polyval(Pk,x(:,2));
if plotflage
    figure
    plot(x(:,1),y,'*',x(:,2),y,'*',x(:,2),y1);
    legend('A','B')
    title('Knee')
end

end