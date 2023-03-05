clear all
close all
import org.opensim.modeling.*;
Trc_path=['C:\MyCloud\OneDriveUcf\Real\Simulation\Passive_Parameter_prediction\Robotic_Leg\'];
listing = dir(Trc_path);

plotfalg=0;
IKflag=0;
Mnames=[];
Tnames=[];
torquoptflag=0;
% myLog = JavaLogSink();
% Logger.addSink(myLog);

runver="E19";
% SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
SubjectNumber=["10"];
stiffnessLbs=[0.207 0.128 0.117 0.088]';%lbs/mm

Data.stiffness=stiffnessLbs*4.44*1000; %N/m
% Data.stiffness=[920.7, 569.3, 520.4, 391.1]';  %[VASINT, RECFEM, BFLH, BFSH]
Data.Stiffnessboundary=[500 1500;300 800;200 800;200 600];
% Data.Stiffnessboundary=[Data.stiffness*.70 Data.stiffness*1.2];
PretensionsLb=[3.56 6.53 6.01 1.5]'; % lbs
MaxLoad=[28.55 26.4	44.96 12.84]';% lbs
Data.Maxdeformation=(MaxLoad./stiffnessLbs)/1000; % m
IniLofPretension=(PretensionsLb./stiffnessLbs)/1000; % m
% Data.stiffness=[900 0.0001 500 300]';%lbs/mm

IniL=[0.33,0.58,0.38,0.20]'; % m
Data.restingpos=IniL-IniLofPretension; 
% Data.restingpos=[0.35,0.56,0.42,0.20];
Data.Foot.mass=0.76;
Footlength=0.32;
MemberWidth=0.03;
const=1/12*Data.Foot.mass;
Data.Foot.Iner=[const*(2*MemberWidth^2) const*(Footlength^2+MemberWidth^2) const*(Footlength^2+MemberWidth^2) 0 0 0];
Data.Shank.mass=0.81;
Shanklength=0.4;
const2=1/12*Data.Shank.mass;
Data.Shank.Iner=[const2*(Shanklength^2+MemberWidth^2) const2*(2*MemberWidth^2) const2*(Shanklength^2+MemberWidth^2) 0 0 0];

Data.restingposboundary=[0.2 0.8;0.4 0.65;0.2 0.55;0.15 0.3];
Data.dissipation=[0.05,0.05,0.05,00.05];
SubjectNumber=append("T0",SubjectNumber);
Data.joints=["ground_pelvis","Hip_Joint","Knee_Joint","Ankle_Joint"];
Data.Weldjoints=["ground_pelvis","Hip_Joint","Ankle_Joint"];
Data.bodies=["Hip_Bone","Thigh","Shank","Foot"];
Data.maxpanlt=-deg2rad(15);
Data.DeGrooteflage=1;
Data.TorqueSolverinterval=30;
Data.ParamSolverinterval=50;
load(fullfile(Trc_path,'Data','Second','ExpData.mat'));
Data.optForce=1;
psname=["Hip90"];
for S=length(psname)
    combinedname=psname(S);
    Data.(combinedname).ModelPath=fullfile(Trc_path,'Model',append(combinedname,runver,'.osim'));
    Data.(combinedname).RefModelpath=fullfile(Trc_path,'Model','FinalModel.osim');
    Data.(combinedname).RefStatepath=fullfile(Trc_path,'Data','Second',append(combinedname,'.sto'));
    Data.(combinedname).RefControlpath=fullfile(Trc_path,'Data','Second',append(combinedname,'_Torque.sto'));
    Data.(combinedname).TorqeSimulPath=fullfile(Trc_path,'Result',append(combinedname,'_TorqueOpt',runver,'.sto'));
    Data.(combinedname).ParmSimulPath=fullfile(Trc_path,'Result',append(combinedname,'_ParamOpt',runver,'.sto'));
    Data.(combinedname).Hipangle=deg2rad(0);
    Data.(combinedname).Kneeangle=deg2rad(90);
    Data.(combinedname).Ankleangle=0;
    Data.ActiveCoordinates=["Knee"];
    Data.ActiveAct=["VASINT","RECFEM","BFLH","BFSH"];
%     Data.ActiveAct=["BFLH"]
    Refmmodel = Model(Data.(combinedname).RefModelpath);

    [osimmodel,Data]=Robotic_Modelcreator(combinedname,Data,Refmmodel);
    
    osimmodel = Model(Data.(combinedname).ModelPath);
    %% processing data
    StateDataTable=TableProcessor(Data.(combinedname).RefStatepath).process;
    r=StateDataTable.getNumRows();
    c=StateDataTable.getNumColumns();
    %     k=0;
    %     for i=0:c-1
    %         clname=StateDataTable.getColumnLabel(k);
    %         if ~any(contains(Data.ActiveCoordinates,string(clname)))
    %             StateDataTable.removeColumnAtIndex(k)
    %             %             c=c-1;
    %         else
    %             k=k+1;
    %         end
    %     end
    %     StateDataTable.removeTableMetaDataKey('nColumns')
    %     StateDataTable.addTableMetaDataString('nColumns','3')
    Newstate=TimeSeriesTable();


    %%% getting data
%     HipAngle=StateDataTable.getDependentColumnAtIndex(1).getAsMat();
%     KneeAngle=StateDataTable.getDependentColumnAtIndex(0).getAsMat();
%     act=StateDataTable.getDependentColumnAtIndex(2).getAsMat();
HipAngle= ExpData.Data90(:,2);
    KneeAngle= ExpData.Data90(:,1);
    act=ExpData.Data90(:,3)/100;
%     KneeAngle=(90:(0-90)/(r-1):0)';
    timejav=StateDataTable.getIndependentColumn();
    for tt=1:r
        time(tt,1)=double(timejav.get(tt-1));
    end
    time=ExpData.Data90time;
    %%% filtering
    [bb,aa] = butter(1, 0.08,'low');
    Hipfilt=filtfilt(bb,aa,HipAngle);
    Kneefilt=filtfilt(bb,aa,KneeAngle);
    Kneefilt=KneeAngle;
    actfilt=filtfilt(bb,aa,act);
    actfilt=act;
    HipSpeed=diff(Hipfilt)./diff(time);
    KneeSpeed=diff(Kneefilt)./diff(time);
    KneeSpeed=[KneeSpeed ;KneeSpeed(1)];
% % % % %     NewData=[Hipfilt(1:end-1),HipSpeed,Kneefilt(1:end-1),KneeSpeed,actfilt(1:end-1)];
    % newtime=time(1):(time(end-1)-time(1))/300:time(end-1);
%     newtime=time(1):(time(end)-time(1))/300:time(end);
    newtime=time;
% % % %     NewDatalowsamp=interp1(time(1:end-1),NewData, newtime,"linear","extrap");
% % % %     Data.(combinedname).Hip=NewDatalowsamp(:,[1,2]);
% % % %     Data.(combinedname).Knee=NewDatalowsamp(:,[3,4]);
% % % %     Data.(combinedname).Act=NewDatalowsamp(:,5);
% % % %     Data.(combinedname).Hip=NewDatalowsamp(:,[1,2]);
    Data.(combinedname).Knee=[Kneefilt KneeSpeed];
    Data.(combinedname).Act=[act];
% % % %     [nrow,ncol]=size(NewDatalowsamp);
    [nrow,ncol]=size(Kneefilt);
    % making state table
    collabels =  StdVectorString();
    Lact=length(Data.ActiveCoordinates);
    for i=1:Lact
        corrval=append("/jointset/",Data.ActiveCoordinates(i),"_Joint/",Data.ActiveCoordinates(i),"/value");
        corrspval=append("/jointset/",Data.ActiveCoordinates(i),"_Joint/",Data.ActiveCoordinates(i),"/speed");
        collabels.add(corrval);
        collabels.add(corrspval);
    end
    for i=1:Lact
        corrval=append("/forceset/",Data.ActiveCoordinates(i),'_act');
        collabels.add(corrval);
    end

    Newstate.setColumnLabels(collabels);
    ncol=double(collabels.size());
    row = RowVector(ncol, 0);
    for iRow=1:nrow
        for iCol = 1 :Lact
            row.set(iCol-1,Data.(combinedname).(Data.ActiveCoordinates(iCol))(iRow,iCol));
            row.set(iCol,Data.(combinedname).(Data.ActiveCoordinates(iCol))(iRow,iCol+1));

        end
            row.set(2,Data.(combinedname).Act(iRow,1));
        Newstate.appendRow(iRow-1, row);
    end

    timeColumn = Newstate.getIndependentColumn();
    for i = 1 : nrow
        timeColumn.set(i-1, newtime(i));
    end
    Kyes=StateDataTable.getTableMetaDataKeys();
    for i=1:1:Kyes.size()
        if contains(string(Kyes.get(i-1)),"nColumns")
            Newstate.addTableMetaDataString(Kyes.get(i-1),string(ncol+1));
        elseif contains(string(Kyes.get(i-1)),"nRows")
            Newstate.addTableMetaDataString(Kyes.get(i-1),string(nrow));
        else
            Keval=StateDataTable.getTableMetaDataString(Kyes.get(i-1));
            Newstate.addTableMetaDataString(Kyes.get(i-1),Keval);
        end
    end
    %%


    Data.(combinedname).Stime=newtime(1);
    Data.(combinedname).Etime=newtime(end-1);
    if torquoptflag
        [TqkneeTrackingSolution]=Roboticleg_TorqueSimulation(Newstate,osimmodel,combinedname,Data);


        TorqStateDataTable=TableProcessor(Data.(combinedname).TorqeSimulPath).process;


        xlabel('Knee(deg)')
        ylabel('Torque(N.m)')
        
%         plot(deg2rad(NewDatalowsamp(:,3)),deg2rad(NewDatalowsamp(:,4)),TorqStateDataTable.getDependentColumnAtIndex(0).getAsMat(),TorqStateDataTable.getDependentColumnAtIndex(1).getAsMat())
        hold on
        %         figure
        plot(90-rad2deg(TorqStateDataTable.getDependentColumnAtIndex(0).getAsMat()),TorqStateDataTable.getDependentColumnAtIndex(2).getAsMat()*100);
        hold on
        Torqtimjav=TorqStateDataTable.getIndependentColumn();
        for tt=1:Torqtimjav.size()
            TorqOpttime(tt,1)=double(Torqtimjav.get(tt-1));
        end
    else
        TorqStateDataTable=Newstate                                                                                                                                                                                                                                                       ;
        
    end

%     [ParkneeTrackingSolution]=Roboticleg_ParamOpt(TorqStateDataTable,osimmodel,combinedname,Data);

    ParamStateDataTable=TableProcessor(Data.(combinedname).ParmSimulPath).process;
    paramtimjav=ParamStateDataTable.getIndependentColumn();
    for tt=1:paramtimjav.size()
        ParamOpttime(tt,1)=double(paramtimjav.get(tt-1));
    end
    load(fullfile(Trc_path,'Data','Second','ExpData.mat'));
    plot(90-ExpData.Data90(:,1),ExpData.Data90(:,3),'*');
%     title('Torque-Knee angle');
    hold on
%     plot(90-Newstate.getDependentColumnAtIndex(0).getAsMat(),Newstate.getDependentColumnAtIndex(2).getAsMat()*100);
    plot(90-rad2deg(ParamStateDataTable.getDependentColumnAtIndex(0).getAsMat()),ParamStateDataTable.getDependentColumnAtIndex(2).getAsMat()*100);
    legend('Exp','Opt')
    xlabel('Knee (deg)')
    ylabel('Torque (N)')
%     plot(newtime,deg2rad(Data.(combinedname).Knee(:,1)));
%     hold on
%     figure
%     plot(ParamOpttime,ParamStateDataTable.getDependentColumnAtIndex(0).getAsMat());
%     hold off
%     legend('exp','param')
    figure 
%     title('Motion-time')
    plot(newtime,90-Data.(combinedname).Knee(:,1));
    hold on
%     plot(TorqOpttime,TorqStateDataTable.getDependentColumnAtIndex(2).getAsMat()*100);
    plot(ParamOpttime,90-rad2deg(ParamStateDataTable.getDependentColumnAtIndex(0).getAsMat()));
    legend('Experiment','Opt')
    xlabel('Time (s)')
    ylabel('Torque (N.m)')
    hold off
    for i=1:length(Data.ActiveAct)
        OptStiffness(i,S)=ParamStateDataTable.getDependentColumnAtIndex(2*i+1).get(0);
        OptRestingPos(i,S)=ParamStateDataTable.getDependentColumnAtIndex(2*i+2).get(0);
    end

end
StiffErro=(OptStiffness-Data.stiffness)./Data.stiffness*100;
RestingPosErro=(OptRestingPos-Data.restingpos)./Data.restingpos*100;
figure
MarkerSize=15;
tiledlayout(1,2)
nexttile
X2 = categorical(Data.ActiveAct');
plot(X2,StiffErro,'.','MarkerSize',MarkerSize)
ylabel ('Stiffness Estimation Error(%)')
nexttile
plot(X2,RestingPosErro,'.','MarkerSize',MarkerSize)
ylabel ('Resting Position Estimation Error(%)')

% Logger.removeSink(myLog);